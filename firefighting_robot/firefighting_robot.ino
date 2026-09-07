// ================================================================================
// RB_2026 - Autonomous fire-fighting robot
// Controller for the RB kit, using a behaviour-based architecture and a
// finite state machine (FSM).
//
// Based on the course template by Andre Monteiro (rev. 25/03/2026).
// ================================================================================

// ============ LIBRARIES ==============
#include "Motor.h"
#include "FlameSensor.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
#include <NewPing.h>

// ============ PINS ==============
#define R_MOTOR_ENABLE_PIN 7
#define R_MOTOR_DIR1_PIN 6
#define R_MOTOR_DIR2_PIN 5

#define L_MOTOR_ENABLE_PIN 2
#define L_MOTOR_DIR1_PIN 3
#define L_MOTOR_DIR2_PIN 4

#define LINE_SENSOR_A_PIN A0
#define LINE_SENSOR_D_PIN 53

#define L_SONAR_PIN 16
#define F_SONAR_PIN 15
#define R_SONAR_PIN 14

#define START_BUTTON_PIN 46
#define STOP_BUTTON_PIN 45

#define LED_PIN 13

#define L_BUMPER_PIN 10
#define R_BUMPER_PIN 11

#define FAN_PIN 9

#define UVTRON_D_PIN 50


// ============ CONSTANTS ==============
// State indentifiers
const int WAIT = 0;
const int NAVIGATE = 1;
const int CENTER = 2;
const int PUT_OUT = 3;
const int TEST = 99;

// Floor markers identifiers
const int NO_MARKER = 1;
const int LINE_MARKER = 1;
const int CIRCLE_MARKER = 2;

// Motor control configuration
const int MAX_POWER = 32;
const int MIN_PWM_RANGE = 100; 
const int MAX_PWM_RANGE= 200;

const int ROTATE_POWER = 16;
const int BASE_POWER = 18;

// Icons
const unsigned char batteryIcon8x8 [] PROGMEM = {
  0b00111100, 0b01000010, 0b01011010, 0b01011010, 0b01011010, 0b01000010, 0b00111100, 0b00011000
};

// SONAR configuration
const int MAX_SONAR_DIST = 100;

// Navigation configuration
const int TARGET_WALL_DIST = 17;
//const int WALL_DIST_TOLERANCE = 3;
const int NO_WALL_DIST = 30;

const int FRONT_LIMIT_DIST = 15;
//const int ALIGN_TOLERANCE = 3;
const int MAX_DELTA = 10;
const int MIN_RIGHT_DIST = 5;

const float KP = 1.0;


 

// ============ OBJECTS ==============
Motor rightMotor(R_MOTOR_ENABLE_PIN, R_MOTOR_DIR1_PIN, R_MOTOR_DIR2_PIN, MAX_POWER, MIN_PWM_RANGE, MAX_PWM_RANGE);
Motor leftMotor(L_MOTOR_ENABLE_PIN, L_MOTOR_DIR1_PIN, L_MOTOR_DIR2_PIN, MAX_POWER, MIN_PWM_RANGE, MAX_PWM_RANGE);

FlameSensor flameSensor(3);

Adafruit_SH1106 display(-1);    //No reset pin

NewPing frontSONAR(F_SONAR_PIN, F_SONAR_PIN, MAX_SONAR_DIST);
NewPing leftSONAR(L_SONAR_PIN, L_SONAR_PIN, MAX_SONAR_DIST);
NewPing rightSONAR(R_SONAR_PIN, R_SONAR_PIN, MAX_SONAR_DIST);

//Motor leftMotor();
//rightMotor.setPower(16);
// ============ VARIABLES ==============
int state;
int power = 0;

bool flameisInRoom = false;
int room = 0;


// ============ SETUP ==============
void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(1000);
  flameSensor.begin();

  // Pins configuration
  pinMode(LINE_SENSOR_D_PIN, INPUT);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);

  pinMode(L_BUMPER_PIN, INPUT_PULLUP);
  pinMode(R_BUMPER_PIN, INPUT_PULLUP);

  pinMode(FAN_PIN, OUTPUT);
  turnOFFFan();

  pinMode(UVTRON_D_PIN, INPUT_PULLUP);


  // OLED configuration
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.display();

  state = WAIT;
  //state = TEST;
}

// ============ FINITE STATE MACHINE LOOP ==============
void loop() {
  switch (state){
    case WAIT:
      waitState();
      break;
    case NAVIGATE:
      navigateState_1();
      break;
    case CENTER:
      centerState();
      break;
    case PUT_OUT:
      putOutState();
      break;
    case TEST:
      testState();
      break;
  }
  // Local in the code to implement the state transition condition of the STOP button
  // Best local to do it, because is common to all the states 
  if(digitalRead(STOP_BUTTON_PIN) == LOW){
    stop();
    state = WAIT;
  }

}


// ============ STATE FUNCTIONS ==============
void waitState(){
  // State where the robot waits for the start button to be pressed to start the mission.
  
  screen1("WAIT", "RB 2026", "Press START");
  Serial.println("WAIT");
  Serial.println("Press START");

  while(digitalRead(START_BUTTON_PIN) == HIGH);

  //Initializations 
  flameisInRoom = false;
  room = 0;

  maneuverToExitWhiteCircle();

  //state = TEST;
  state = NAVIGATE;



}

void navigateState_1(){
  // State responsible for navigating the arena while searching for the candle flame.
  // In this basic implementation, the robot continuously explores the environment,
  // including entering rooms, while monitoring the flame sensor.
  //
  // The navigation strategy is based on right-wall following.

  // Measure the distance to obstacles in front of the robot.
  int frontDist = getDistance(frontSONAR);
  //int frontDist = getFilteredDistance(frontSonar);
  delay(15);
  // Measure the distance to the wall on the robot's right side.
  int rightDist = getDistance(rightSONAR);
  //int rightDist = getFilteredDistance(rightSonar);
  delay(15);

  // If one of the bumpers is pressed, the robot briefly moves backwards
  // while turning away from the obstacle.
  if (digitalRead(L_BUMPER_PIN) == LOW) {
    moveTime(-BASE_POWER, -10, 600);
    return;
  }  
  if (digitalRead(R_BUMPER_PIN) == LOW) {
    moveTime(-BASE_POWER, 10, 600);
    return;
  } 
  
  // If the robot gets too close to the right wall,
  // rotate slightly left until a safe distance is restored.
  if (rightDist < MIN_RIGHT_DIST) {
    screen1("N-Min-Right", String(frontDist), String(rightDist));
    rotateWhile(ROTATE_POWER, rightSONAR, MIN_RIGHT_DIST);
    return;
  }

  // Obstacle avoidance maneuver.
  // If an obstacle is detected in front of the robot, rotate left
  // until the path ahead becomes clear.
  
  if (frontDist < FRONT_LIMIT_DIST) {
    screen1("N-Front", String(frontDist), String(rightDist));
    rotateWhile(ROTATE_POWER, frontSonar, FRONT_LIMIT_DIST/2);
    return;
  } 
  
  
  if (frontDist < FRONT_LIMIT_DIST) {
    //screen1("N-Front", String(frontDist), String(rightDist));
    avoidFrontObstacle(FRONT_LIMIT_DIST);
    return;
  }
  
  // Maneuver executed when the right wall disappears,
  // indicating a possible corridor or room entrance.    
  if (rightDist > NO_WALL_DIST) {
    //screen1("Reacquire RW", String(frontDist), String(rightDist));
    reacquireRightWall();
    return;
  }

  // Compute the wall-following error:
  // positive error  -> robot is too far from the wall
  // negative error  -> robot is too close to the wall
  int distanceError = TARGET_WALL_DIST - rightDist;

  // Compute the steering correction (delta) using proportional control.
  // The larger the error, the larger the steering correction applied.
  int delta = round(KP * distanceError);

  // Limit the steering correction to prevent excessive turning
  // and unstable robot behaviour.
  delta = limitValue(delta, -MAX_DELTA, MAX_DELTA);

  // Move the robot forward while applying the steering correction
  // needed to maintain the target distance to the wall.
  move(BASE_POWER, delta);
  screen1("Nav-Move", String(delta), "");

  int FloorMarker = getFloorMarker();
  if(FloorMarker == LINE_MARKER) {
    room ++;
    brake();
    delay(2000);

    //flameSensor.update();
    //int.flameDireciton = flameSensor.getFlameDireciton();

    int uvTron = digitalRead(UVTRON_D_PIN);

    screen1("Room: " + String(room), "", "UVTron:" + String(uvTron));
    delay(3000);

    flameSensor.update();
    int flameDirection = flameSensor.getFlameDirection();

    //if (flameDirection != 0) 
    if(uvTron == HIGH)
     flameisInRoom = true;
    else
      maneuverToGoToNextRoom();

  }
  //else if((FloorMarker == CIRCLE_MARKER) && (room == 4) ) {
  else if((FloorMarker == CIRCLE_MARKER) && (room == 4) ) 
     maneuverToGoToIslandRoom();
   }

if(flameisInRoom) {
  // Update the flame sensor readings.
  flameSensor.update();

  // If a flame is detected, switch to the CENTER state,
  // where the robot will attempt to align with the flame.
  if (flameSensor.getFlameDirection() != 0)
    state = CENTER;
}

void navigateState(){
  // State where the robot navigates the arena searching for the flame
  // using a right-wall-following algorithm

  int frontDist = getDistance(frontSONAR);
  int rightFrontDist = getDistance(rightFrontSONAR);
  int rightRearDist = getDistance(rightRearSONAR);

  // 1. If there is a wall ahead, the robot turns left
  // as long as the front remains blocked

  if(frontDist < FRONT_LIMIT_DIST){
    // Turn left...
  }

  // 2. If the right wall ends, the robot turns to the right
  // untilit detects another wall on the right side

  if(rightFrontDist >= NO_WALL_DIST && rightRearDist > NO_WALL_DIST){
    // Turn right...
  }

  // 3. Normal follow-up of the right wall

  // Keep the robot aligned with the wall
  int alignmentError = rightFrontDist - rightRearDist;
  if(abs(alignmentError) > ALIGN_TOLERANCE){
    // Align the robot with the right wall..
  }

  // Make the robot move forward, keeping it a safe distance from the wall
  int averageRightDist = (rightFrontDist + rightRearDist) / 2;
  int distanceError = averageRightDist - TARGET_WALL_DIST;

  int delta = 0;

  if(abs(distanceError) > WALL_DIST_TOLERANCE){
    delta = -(distanceError * KP_DIST);
  }

  delta = limitValue(delta, -MAX_DELTA, MAX_DELTA);

  move(BASE_POWER, delta);

  // State trasition condition of the NAVIGATE state
  flameSensor.update();
  if(flameSensor.getFlameDirection() != 0)
    // Flame is detected
    state = CENTER;

  /*
  if(getFloorMarker() == LINE_MARKER){
    stop();
    //room = room + 1;

    // Verify if flame is in room
    // . . .

  }
  */
  //screen1("NAVIGATE", "" , "");
//}


void centerState() {

  // State responsible for approaching the candle flame while keeping
  // the robot aligned with the center direction of the flame.
  //
  // In this state, the robot continuously adjusts its orientation
  // according to the flame sensor readings until it gets close enough
  // to attempt extinguishing the flame.

  // Turn on the status LED to indicate that the flame is currently detected.
  turnOnLED();

  // Update the flame sensor readings and determine the flame direction.
  //
  // Direction codes:
  // 0 -> flame not detected
  // 1 -> flame detected on the right
  // 2 -> flame detected in the center
  // 3 -> flame detected on the left
  flameSensor.update();

  int dir = flameSensor.getFlameDirection();
  int dist = 0;

  // Select the robot maneuver according to the flame direction.
  switch(dir) {

    case 0:  // Flame not detected.

      // In this simple implementation, losing the flame usually means
      // that the robot is already very close to it and the flame has
      // left the sensor's field of view.
      //
      // Therefore, losing the flame can also used as a condition
      // to transition to the PUT_OUT state.
      //
      // Alternative implementations could instead return to the
      // NAVIGATE state to search for the flame again.
      //
      // What other ideas can we implement?
      
      //turnOfLED();
      //state = NAVIGATE;
      //brake();

      state = PUT_OUT;
      return;

    break;

    case 1: // Flame detected on the right side.

      // Rotate right in order to align the robot with the flame.
      rotate(-5);

    break;

    case 2: // Flame detected approximately in front of the robot.

      // Move forward while keeping the current heading.
      move(BASE_POWER, 0);

      // Measure the distance to the obstacle/flame area ahead.
      dist = getDistance(frontSONAR);

      // If the robot is sufficiently close to the flame,
      // stop the robot and transition to the PUT_OUT state.
      if(dist < 15) {
        brake();
        state = PUT_OUT;
        return;
      }

    break;

    case 3: // Flame detected on the left side.

      // Rotate left in order to align the robot with the flame.
      rotate(5);

    break;
  }

  // Display debugging information on the OLED screen.
  screen1("CENTER", "Dir: " + String(dir), "Dist: " + String(dist));
}

void putOutState() {
  // State responsible for extinguishing the candle flame.
  //
  // In this state, the robot activates the fan for a predefined period
  // of time in an attempt to extinguish the flame.
  //
  // After the extinguishing attempt, the robot moves slightly backwards
  // and checks whether the flame is still detected.

  // Display the current state on the OLED screen.
  screen1("PUT OUT", "", "");

  // Activate the fan for 3 seconds in order to extinguish the flame.
  turnONFan();
  delay(3000);
  turnOFFFan();

  // Move slightly backwards to improve visibility of the flame
  // and avoid remaining too close to the candle.
  moveTime(-BASE_POWER, 0, 600);

  // Update the flame sensor readings after the extinguishing attempt.
  delay(1000);
  flameSensor.update();

  // If the flame is still detected, return to the CENTER state
  // to realign the robot and attempt extinguishing again.
  // What other ideas can we implement?
  if(flameSensor.getFlameDirection() != 0) {
    state = CENTER;
    return;
  }

  // If the flame is no longer detected, assume that the flame
  // has been successfully extinguished.

  // Turn off the status LED, since the flame is no longer detected.
  turnOfLED();

  // Transition to the WAIT state.
  state = WAIT;

  // Instead of transition to WAIT state, the robot could attempt to navigate
  // back to the starting position.
  //state = RETURN;
}


void testState(){
  //testFlameSensor();
  //testLineSensor();
  //testDisplay();

  //testSONAR();
  //testButton();

  //testSensorA();
  //testSensorA();

  testPWMRange();
}

// ============ TEST FUNCTIONS ==============
void testPWMRange() {
  // Test to find the minPWMRange and maxPWMrange to use with the Motor class
  // See the implementantion of the setPower() method of the motor class to know about
  // minPWMRange and maxPWMrange.
  // The test aims to find the minimum and maximum value of the PWM signal most suitable for
  // the robot in question. The minimum value corresponds to the value that makes the motors
  // start to rotate and the maximum value corresponds to the maximum speed that we want to
  // consider suitable for the robot.

  // Verify direction of rotation for your robot.
  digitalWrite(L_MOTOR_DIR1_PIN, HIGH);
  digitalWrite(L_MOTOR_DIR2_PIN, LOW);
  analogWrite(L_MOTOR_ENABLE_PIN, power);

  digitalWrite(R_MOTOR_DIR1_PIN, LOW);
  digitalWrite(R_MOTOR_DIR2_PIN, HIGH);
  analogWrite(R_MOTOR_ENABLE_PIN, power);
  
  delay(500);
  power++;
  Serial.println(power);
  screen1(String(power), "", "");
  if (power > 255)
    state = WAIT;
}

void testFlameSensor(){
  flameSensor.update();

  int flameX = flameSensor.getFlameX();
  int flameY = flameSensor.getFlameY();
  int flameS = flameSensor.getFlameSize();
  int flameD = flameSensor.getFlameDirection();

  Serial.println(" D : " + String(flameD) + " S :" + String(flameS) + " X :" + String(flameX) + " Y :" + String(flameY));
}

void testLineSensor(){
  //Serial.println("Line sensor (analog):" + String(analogRead(LINE_SENSOR_A_PIN)));
  //Superficie preta 772
  //Superficie branca 74

  Serial.println("Line sensor (digital):" + String(digitalRead(LINE_SENSOR_D_PIN)));
  //
}

void testDisplay(){
  static int room;
  screen1("NAVIGATE", "Room: " + String(room++), "Test");
}

void testSONAR(){
  /*
  Serial.print("Front sonar: " + String(frontSONAR.ping_cm()));
  Serial.print('\t');
  Serial.println("Left sonar: " + String(leftSONAR.ping_cm()));
  delay(50);
  */

  Serial.print("Front sonar: " + String(getDistance(frontSONAR)));
  Serial.print('\t');
  Serial.println("Left sonar: " + String(getDistance(leftSONAR)));
  delay(50);
}

void testButton(){
  Serial.println(digitalRead(START_BUTTON_PIN));
}

// ============ ACTUATOR FUNCTIONS ==============
void stop(){
  // Function to stop the robot
  leftMotor.stop();
  rightMotor.stop();
}

void brake(){
  // Function to brake the robot
  leftMotor.brake();
  rightMotor.brake();

}

void moveStraight(int power){
  // Function to move straight at spedd = power.
  // power > 0 - move forward
  // power < 0 . move backwards
  leftMotor.setPower(power);
  rightMotor.setPower(power);
}

void moveStraightTime(int power, int time){
  // Function to move straight at spedd = power, but limited by time
  // power > 0 - move forward
  // power < 0 . move backwards
  moveStraight(power);
  delay(time);
  brake();
}

void moveStraightDist(int power, int dist){
  // Function to move straight at spedd = power, but limited by distance
  // Tp be implemented in the future based on adometry and shaft encoders
  // . . .
}

void move(int power, int delta){
  // Function to move the robot around a curve
  // power > 0 - move forward
  // power < 0 . move backwards
  // delta - controls the direction and the radius of the curve
  // if power > 0 and delta > 0 - Turn left
  // if power > 0 and delta < 0 - Turn right
  // if power < 0 and delta > 0 - Turn right
  // if power < 0 and delta < 0 - Turn left

  leftMotor.setPower(power - delta);
  rightMotor.setPower(power + delta);
}

void moveTime(int power, int delta, int time){
  // Function to implement move(), but limited by time 

  move(power, delta);
  delay(time);
  brake();
}

void rotate(int power){
  // Function to rotate around itself
  // power > 0 -> rotate left
  // power < 0 -> rotate right
  leftMotor.setPower(-power);
  rightMotor.setPower(power);
}

void rotateTime(int power, int time){
  // Function to rotate around itself, but limited by time
  rotate(power);
  delay(time);
  brake(); 
}

void rotateLeft90(){
  // Function to rotate left 90º
  rotateTime(ROTATE_POWER, 700);  // Time must be calibrated for a specific power and floor 
}

void rotateRight90(){
  // Function to rotate right 90º
  rotateTime(-ROTATE_POWER, 700);  // Time must be calibrated for a specific power and floor 
}

void rotateAngle(int ang){
  // Function to roate a specific angle
  // To be implemented in the future based on odometry or based on a gyroscope sensor

}

void rotateWhile(int power, NewPing& sonar, int dist){
  // Function to rotate at speed = power, while the distance measured by a sonar is less than dist
  rotate(power);
  while(getDistance(sonar) < dist);
}



void turnONFan() {
  digitalWrite(FAN_PIN, LOW);
}

void turnOFFFan() {
  digitalWrite(FAN_PIN, HIGH);
}

// ============ SENSOR FUNCTIONS ==============
int getDistance(NewPing& sonar){
  
  int dist = sonar.ping_cm();
  if(dist == 0) dist = MAX_SONAR_DIST;

  return dist;
}

int getFloorMarker(){
  if(digitalRead(LINE_SENSOR_D_PIN) == LOW){ // the sensor is seeing with
    moveTime(BASE_POWER, 0, 400); // Move forward ~ 5cm (parameter "time = 400" needs calibration)
    if(digitalRead(LINE_SENSOR_D_PIN) == LOW){ // the sensor is seeing with
      return CIRCLE_MARKER;
    }
    return LINE_MARKER;
  }
  return NO_MARKER;
}

// ============ INTERFACE FUNCTIONS ==============
void turnOnLED(){
  digitalWrite(LED_PIN, HIGH);
}

void turnOfLED(){
  digitalWrite(LED_PIN, LOW);
}

void screen1(String s1, String s2, String s3){
  display.clearDisplay();
  display.setTextSize(2);
  //display.setTextColor(WHITE);

  display.setCursor(0, 0);
  display.print(s1);

  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print(s2);

  display.setCursor(0, 40);
  display.print(s3);

  display.drawCircle(95, 40, 15, WHITE);
  display.drawTriangle(80, 50, 110, 50, 95, 20, WHITE);
  display.drawBitmap(119, 2, batteryIcon8x8, 8, 8, WHITE);

  display.display();
}

// ============ MANUEVER FUNCTIONS ==============
void avoidFrontObstacle(int frontClearDist) {

  // Maneuver used when an obstacle is detected in front of the robot.
  //
  // The robot rotates left while continuously checking the front sonar
  // until the path ahead becomes clear again.
  //
  // A timeout is used to prevent the robot from getting stuck forever
  // if the obstacle cannot be avoided.

  unsigned long startTime = millis();

  while (millis() - startTime < 2000) {

    // Measure distances.
    int frontDist = getDistance(frontSONAR);
    delay(15);

    int rightDist = getDistance(rightSONAR);
    delay(15);

    screen1("Avoid FO", String(frontDist), String(rightDist));

    // Stop rotating if the path ahead becomes sufficiently clear.
    //if (frontDist > FRONT_CLEAR_DIST) {
    if (frontDist > frontClearDist) {
      brake();
      return;
    }

    // If the robot gets too close to the right wall while rotating,
    // stop the maneuver to avoid colliding with the wall.
    if (rightDist < MIN_RIGHT_DIST) {
      brake();
      return;
    }

    // Rotate left.
    rotate(ROTATE_POWER);
  }

  // Stop the robot if the timeout expires.
  brake();
}

void reacquireRightWall() {
  unsigned long startTime = millis();

  while (millis() - startTime < 2000) {

    int frontDist = getDistance(frontSONAR);
    delay(15);

    int rightDist = getDistance(rightSONAR);
    delay(15);

    screen1("Reacquire RW", String(frontDist), String(rightDist));

    // If an obstacle appears in front, stop this maneuver.
    if (frontDist < FRONT_LIMIT_DIST) {
      brake();
      return;
    }

    // If the right wall is detected again, stop the maneuver.
    //if (rightDist < WALL_FOUND_DIST) {
    if (rightDist < TARGET_WALL_DIST) {
      brake();
      return;
    }

    // Move forward while curving to the right.
    // The sign of RIGHT_TURN_DELTA must be adjusted to your move() function.
    move(BASE_POWER, -MAX_DELTA);
  }

  brake();
}


void maneuverToGoToNextRoom() { 
switch(room) {
   case 1:
     moveTime(-BASE_POWER, 0, 500);
     rotateLeft90();
     move(BASE_POWER, 0);
     while(getDistance(frontSonar) > 15);
    break;
   case 2:
   break;
   case 3:
   break; 
   }
}

 
void maneuverToGoToIslandRoom() {
   move(-BASE_POWER, 0);
   while(getDistance(leftSonar) > 50);
   rotateTime(ROTATE_POWER, 700);
   //rotateWhile(ROTATE_POWER, rightSonar, 15 );
}


void maneuverToExitWhiteCircle() {
  move(BASE_POWER, 0);
  while(digitalRead(LINE_SENSOR_D_PIN) == LOW);
}




// ============ AUXILIAR FUNCTIONS ==============
int limitValue(int value, int min, int max){
  if(value < min) return min;
  if(value > max) return max;
  return value;
}