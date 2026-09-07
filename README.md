# Autonomous Fire-Fighting Robot (RB 2026)

Arduino-based autonomous robot that navigates a maze of rooms, detects an open
flame and extinguishes it with a fan. Built for the Robotics course during an
Erasmus exchange in Portugal.

## How it works

The controller is a **finite state machine** with a behaviour-based architecture:

| State | Behaviour |
|---|---|
| `WAIT` | Idle on the white start circle until the START button is pressed |
| `NAVIGATE` | Right-wall following with a proportional (P) controller, obstacle avoidance, room-to-room manoeuvres |
| `CENTER` | Once a flame is detected, align the robot with the flame using the IR camera |
| `PUT_OUT` | Drive in and run the fan until the flame is gone |
| `TEST` | Bench-test routines for each subsystem (motors, sonar, sensors, display, buttons) |

A STOP button check runs outside the switch statement, so any state can be
aborted immediately.

## Hardware

- Arduino Mega 2560
- 2 × DC motors with H-bridge (PWM speed control, coast and active braking)
- 3 × HC-SR04 ultrasonic sensors (front, left, right) via `NewPing`
- DFRobot IR camera used as a flame sensor (I²C, 3 zones)
- UVTron flame detector
- Line sensor (analog + digital) for floor markers
- 2 × bumper switches
- SH1106 OLED display (128×64) for state feedback
- Fan for extinguishing
- START / STOP buttons, status LED

## Software layout

```
firefighting_robot/
├── firefighting_robot.ino   # FSM, navigation, manoeuvres, test routines
├── Motor.h / Motor.cpp      # H-bridge motor driver: setPower, stop, brake
└── FlameSensor.h / .cpp     # DFRobot IR camera wrapper: position, size, direction
```

### Navigation

Wall following uses a proportional controller on the right-hand sonar:

```
error  = TARGET_WALL_DIST - rightDistance
delta  = KP * error        // differential applied to the two motors
```

with `TARGET_WALL_DIST = 17 cm`, front clearance limit `15 cm`, and a
`NO_WALL_DIST` threshold of `30 cm` used to detect doorways and corners.

## Dependencies

Install through the Arduino Library Manager:

- `Adafruit GFX Library`
- `Adafruit SH1106` (SH1106 OLED driver)
- `NewPing`

## Building

Open `firefighting_robot/firefighting_robot.ino` in the Arduino IDE, select
**Arduino Mega 2560**, and upload.

## Credits

Built on the RB kit course template by André Monteiro (rev. 25/03/2026).
