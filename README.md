# Mesmerize_01

LineFollower
A QTR sensor based Line follower made for TechFest of IIT Bombay.
The bot uses QTR sensors to detect the line, and then uses
PID algorithm to follow the line

Pins declarations

QTR Sensors (From left to right of array): 13, 12, 14, 27, 26, 25, 33, 32 (pins of esp32)

Libraries used
QtrSensor.h (by polulu)

Algorithm Description for Line-Following Bot:
Sensor Setup & Calibration:

Initialize an 8-sensor array using the QTR sensors.
The sensors are calibrated by moving the bot along the line for 10 seconds during the setup phase.
Motor Initialization:

Two motors are controlled using the TB6612FNG motor driver.
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

The sensor values are converted  (0 or 1000) based on a threshold for easier interpretation of line detection.
Open-ended PID:

If the bot leaves the line, it continues rotating until it realigns with the line.
Forward Brake:

A helper function that controls motor direction and speed based on the inputs from the PID control.

<h2>pahses:</h2>
<h4>1) Line-Follower</h4>
<p>
  for line follower i have called only pid_control function.
</p>
<img src='https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.elektroda.com%2Frtvforum%2Ftopic2325660.html&psig=AOvVaw3ngkYUd5s6U_4RzgD0nGMl&ust=1743156836551000&source=images&cd=vfe&opi=89978449&ved=0CBQQjRxqFwoTCJCGibODqowDFQAAAAAdAAAAABAO'>
<h4>1) Maze-Solver</h4>
<p>
  for Maze-solver. i have used <b>SRB(Left-Straight-Right-Back) and RSLB(Right-Straight-Left-Back) priority Algorithm </b>
</p>
<img src='[https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.elektroda.com%2Frtvforum%2Ftopic2325660.html&psig=AOvVaw3ngkYUd5s6U_4RzgD0nGMl&ust=1743156836551000&source=images&cd=vfe&opi=89978449&ved=0CBQQjRxqFwoTCJCGibODqowDFQAAAAAdAAAAABAO](https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.instructables.com%2FLine-Follower-Bot-and-Mesh-Solver-for-Meshmerize-C%2F&psig=AOvVaw0y3lT7LKwpwZGxkuiCzgoR&ust=1743156981520000&source=images&cd=vfe&opi=89978449&ved=0CBQQjRxqFwoTCJj7iP6DqowDFQAAAAAdAAAAABAJ)'>

