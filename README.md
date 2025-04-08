# Laboratory 9. Take Control: The PID Feedback Controller
Megan Fister, Aashika Uppala
4/9/2025

Introduction or Summary
Project Goal:
The objective of this lab is to implement a PID (Proportional-Integral-Derivative) feedback controller on an Arduino-based robot to maintain a specified distance from a wall or object using an ultrasonic sensor.
Summary of Work Performed:
We began by installing the PID_V2 library in the Arduino IDE and modifying the existing sketch to define and initialize the PID controller. Using sensor data from an ultrasonic module, we calculated the error between the setpoint and the actual distance to control the motor speeds accordingly. The output of the PID controller was mapped to adjust motor power and direction to maintain or correct the robot’s position. In the second part, we developed a function that allowed the robot to respond to distance changes by moving forward or backward. In the final part, we implemented a wall-following feature by adjusting each motor’s speed independently based on PID output, enabling the robot to follow a wall while maintaining a fixed lateral distance.
Summary of Outcome:
The PID controller was successfully implemented and tuned to allow the robot to stabilize at a set distance from a wall. Furthermore, the wall-following functionality enabled the robot to dynamically adjust its movement and follow a path while maintaining a consistent distance from the side barrier.

Discussion:
What can you learn from the results obtained?
The results demonstrated how effective PID control can be in achieving stable and adaptive movement in robotics. By tuning the Kp, Ki, and Kd parameters, we observed changes in system responsiveness, overshoot, and stability. The proportional term corrected the current error, the integral addressed accumulated errors over time, and the derivative helped dampen oscillations. This showed how PID parameters influence dynamic control.
What happens if only the proportional term is used?
The system may respond quickly to errors but could oscillate around the setpoint without reaching a steady state.
What effect does the integral term have?
It helps eliminate steady-state errors by accumulating past errors but may cause overshoot if too high.
What effect does the derivative term have?
It dampens the response and reduces overshoot by responding to the rate of error change, improving system stability.
How does tuning affect the performance?
Proper tuning improves accuracy, speed, and stability. Poorly tuned parameters lead to instability or slow reaction.

Conclusion: 
This lab introduced the practical application of a PID controller using Arduino and sensor feedback for robot navigation. Key lessons included understanding the function and importance of each PID component, the importance of real-time sensor data in control systems, and how tuning directly affects system behavior. Implementing wall-following behavior emphasized how precise control logic and motor adjustments can achieve complex robotic tasks. Overall, this lab reinforced foundational control theory with hands-on experience, preparing us for more advanced automation and robotics systems.
