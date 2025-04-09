# Laboratory 9. Take Control: The PID Feedback Controller
Megan Fister, Aashika Uppala
4/9/2025

## Introduction or Summary
Project Goal:
The objective of this lab is to implement a PID (Proportional-Integral-Derivative) feedback controller on an Arduino-based robot to maintain a specified distance from a wall or object using an ultrasonic sensor.
Summary of Work Performed:
We began by installing the PID_V2 library in the Arduino IDE and modifying the existing sketch to define and initialize the PID controller. Using sensor data from an ultrasonic module, we calculated the error between the setpoint and the actual distance to control the motor speeds accordingly. The output of the PID controller was mapped to adjust motor power and direction to maintain or correct the robot’s position. In the second part, we developed a function that allowed the robot to respond to distance changes by moving forward or backward. In the final part, we implemented a wall-following feature by adjusting each motor’s speed independently based on PID output, enabling the robot to follow a wall while maintaining a fixed lateral distance.
Summary of Outcome:
The PID controller was successfully implemented and tuned to allow the robot to stabilize at a set distance from a wall. Furthermore, the wall-following functionality enabled the robot to dynamically adjust its movement and follow a path while maintaining a consistent distance from the side barrier.


## Methods
Part 1 – PID Use
In this section, you will make modifications to your Arduino sketch to implement a PID control
using the distance measurement from the ultrasonic sensor.
1. Open the Arduino IDE and install the PID_v2 Library.
a. Open the Library Manager (Tools>ManageLibraries) and search for PID_V2.
b. Install the latest library by Brett Beauregard.
2. Open your robot sketch and modify it according to the following steps:
a. Include the library (#include <PID_V2.h>).
b. Define the setpoint, measurement, output, Kp, Ki, and Kd variables as type double.
c. Before the setup function, create the PID instance:
PID myPID(&measurement, &output, &setpoint, Kp, Ki, Kd, DIRECT);
The measurement, output, and setpoint have an ampersand (&) before them because
these variables are pointers.
d. Within the setup function, initialize the PID as follows:
myPID.SetTunings(Kp, Ki, Kd);
myPID.SetMode(AUTOMATIC);
e. Within the loop function, run the PID after obtaining the distance:
myPID.Compute();
This line will compute the output of the PID according to the tuning and inputs and write
the output to its pointer.
f. Write the setpoint, measurement, and output values to the serial port to verify the
operation.
g. Verify your code and solve any compilation errors.
Part 2 – Keep Your Distance
In this section, you will implement the PID controller into the robot movement such that given
an object in front, the robot will move forward or backward to maintain the desired distance.
1. Write a function to indicate the robot to move back or forward depending on the measured
distance.
a. The PID library will return an output value from 0 to 255. Implement a function that
obtains this output and moves forward or backward and with a fast or slow speed.
Hint: Use the map function (map(variable, minValue, maxValue,
newminValue, newmaxValue)) and assign the speed of the motors a value
between -255 to 255.
2. Tune your system by modifying Kp, Ki, and Kd, and test your system with the
instructor.
Lab 7 – Take Control
BAE 305
Page 3 of 3
Part 3 – Wall follower
A wall follower is a typical application for robot navigation. In this section, you will implement
and modify the PID controller and the orientation of the ultrasonic sensor to maintain the same
distance from a wall while the robot moves forward or backward.
1. So far, we have set the wheels to move at the same speed (even if they are moving in
different directions). To make the robot follow a wall at a constant distance, you must
modify your program such that you can move the left and right wheels at different speeds.
2. Set the sketch to modify the right and left speeds depending on the output of the PID. Hint:
Modify the map function and use basic arithmetic to control the motors.
3. Tune the PID controller while the car is on the bench.
4. The instructor will provide you with a distance sensor and double-sided tape to place on the
side of your robot. Your robot should go as fast as it can while maintaining a distance from a
wall


## Discussion
_What can you learn from the results obtained?_

The results demonstrated how effective PID control can be in achieving stable and adaptive movement in robotics. By tuning the Kp, Ki, and Kd parameters, we observed changes in system responsiveness, overshoot, and stability. The proportional term corrected the current error, the integral addressed accumulated errors over time, and the derivative helped dampen oscillations. This showed how PID parameters influence dynamic control.

_What happens if only the proportional term is used?_

The system may respond quickly to errors but could oscillate around the setpoint without reaching a steady state.

_What effect does the integral term have?_

It helps eliminate steady-state errors by accumulating past errors but may cause overshoot if too high.

_What effect does the derivative term have?_

It dampens the response and reduces overshoot by responding to the rate of error change, improving system stability.

_How does tuning affect the performance?_

Proper tuning improves accuracy, speed, and stability. Poorly tuned parameters lead to instability or slow reaction.

## Conclusion
This lab introduced the practical application of a PID controller using Arduino and sensor feedback for robot navigation. Key lessons included understanding the function and importance of each PID component, the importance of real-time sensor data in control systems, and how tuning directly affects system behavior. Implementing wall-following behavior emphasized how precise control logic and motor adjustments can achieve complex robotic tasks. Overall, this lab reinforced foundational control theory with hands-on experience, preparing us for more advanced automation and robotics systems.
