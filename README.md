# Mesmerize_01

LineFollower
A QTR sensor based Line follower made for AXIS VNIT 24
Our bot secured #3 position in VNIT's annual techfest, AXIS
The bot uses QTR sensors to detect the line, and then uses
PID algorithm to follow the line

Pins declarations
Calibration button: A6
Start button: A7
Left motor forward: 6
Left motor enable: 9
Right motor forward: 3
Right motor enable: 5

QTR Sensors (From right to left of array): 8, A0, A1, A2, A3, A4, A5, 7

Libraries used
QtrSensor.h (by polulu)

Algorithm Description for Line-Following Bot:
Sensor Setup & Calibration:

Initialize an 8-sensor array using the QTR sensors.
The sensors are calibrated by moving the bot along the line for 10 seconds during the setup phase.
Motor Initialization:

Two motors are controlled using the DRV8835 motor driver.
Motor pins are initialized, allowing forward and backward movement.
Buttons for Start and Calibration:

A button is assigned to start calibration, and another is used to start/stop the robot.
The robot won’t start unless it is calibrated and the start button is pressed.
PID Control Mechanism:

The PID control algorithm uses three terms (Proportional, Integral, and Derivative) to compute an optimal correction for motor speeds based on the sensor readings.
The error is calculated as the deviation from the ideal line position.
The correction adjusts the motor speeds to keep the bot centered on the line.
Movement Commands:

Depending on the PID output, the bot adjusts its motor speeds to either move straight or turn left/right when deviations are detected.
When the bot loses the line completely, it rotates in the direction it last sensed the line.
Binary Conversion of Sensor Values:

The sensor values are converted to binary (0 or 1) based on a threshold for easier interpretation of line detection.
Open-ended PID:

If the bot leaves the line, it continues rotating until it realigns with the line.
Forward Brake:

A helper function that controls motor direction and speed based on the inputs from the PID control.
