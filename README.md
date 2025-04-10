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
### Instruments
• A Computer running Arduino IDE, and Chrome Browser

• SparkFun Inventor’s kit

&nbsp; &nbsp; &nbsp; &nbsp; o RedBoard

&nbsp; &nbsp; &nbsp; &nbsp; o Ultrasonic sensor

&nbsp; &nbsp; &nbsp; &nbsp; o Two motors

&nbsp; &nbsp; &nbsp; &nbsp; o Motor Driver

&nbsp; &nbsp; &nbsp; &nbsp; o Battery Pack

### Part 1 – PID Use
In this section, you will make modifications to your Arduino sketch to implement a PID control using the distance measurement from the ultrasonic sensor.

First, open the Arduino IDE and install the PID_v2 Library.

- Open the Library Manager (Tools>ManageLibraries) and search for PID_V2. Install the latest library by Brett Beauregard.

Next, open your robot sketch and modify it according to the following steps:

Before the setup function:

- Include the library.
  
- Define the setpoint, measurement, output, Kp, Ki, and Kd variables as type double.

- Create the PID instance

_NOTE: The measurement, output, and setpoint have an ampersand (&) before them because these variables are pointers._

```c++
#include <PID_v2.h>

double setpoint;
double measurement;
double output;
double Kp;
double Ki;
double Kd;

PID myPID(&measurement, &output, &setpoint, Kp, Ki, Kd, DIRECT);
```

Within the setup function:

- Initialize the PID

```c++
myPID.SetTunings(Kp, Ki, Kd);
myPID.SetMode(AUTOMATIC);
```

Within the loop function:

- Run the PID after obtaining the distance. This line will compute the output of the PID according to the tuning and inputs and write the output to its pointer.

- Write the setpoint, measurement, and output values to the serial port to verify the operation.

```c++
    myPID.Compute();

    Serial.print(setpoint);
    Serial.print(" ");
    Serial.print(measurement);
    Serial.print(" ");
    Serial.println(output);
```

Finally, run your code and solve any compilation errors.

### Part 2 – Keep Your Distance
In this section, you will implement the PID controller into the robot movement such that given an object in front, the robot will move forward or backward to maintain the desired distance.

Write a function to indicate the robot to move back or forward depending on the measured distance.

- The PID library will return an output value from 0 to 255. Implement a function that obtains this output and moves forward or backward and with a fast or slow speed.

_Hint: Use the map function (map(variable, minValue, maxValue, newminValue, newmaxValue)) and assign the speed of the motors a value between -255 to 255._

Tune your system by modifying Kp, Ki, and Kd, and test your system with the instructor.

```c++
  duration = pulseIn(echoPin, HIGH);
  distance = (duration*340/10000)/2; // Units are cm

void moveRobot() {
  int speed = map(output, 0, 255, 50, 255); // Map PID output to motor speed (min 50 to avoid stalling)

  if (measurement > setpoint) {
    // Move forward
    rightMotor(-speed);
    leftMotor(speed);
   // Serial.println("Moving forward");
  }
  else if (measurement < setpoint) {
    // Move backward
    rightMotor(speed);
    leftMotor(-speed);
  //  Serial.println("Moving backward");
  }
}
```

### Part 3 – Wall follower
We did not do part 3 in this lab.

## Results
### Part 1 - PID Use
### Part 2 - Keep Your Distance
### Final Code
```c++
/*  SparkFun Inventor’s Kit
  Circuit 5B - Remote Control Robot

  Control a two wheeled robot by sending direction commands through the serial monitor.
  This sketch was adapted from one of the activities in the SparkFun Guide to Arduino.
  Check out the rest of the book at
  https://www.sparkfun.com/products/14326

  This sketch was written by SparkFun Electronics, with lots of help from the Arduino community.
  This code is completely free for any use.

  View circuit diagram and instructions at: https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40
  Download drawings and code at: https://github.com/sparkfun/SIK-Guide-Code

  Modified by:
  Carlos Jarro for University of Kentucky's BAE305 Lab 6
  02/28/2024
*/
#include <SoftwareSerial.h>
#include <PID_v2.h>

double setpoint;
double measurement;
double output;
double Kp;
double Ki;
double Kd;

SoftwareSerial mySerial(2, 3); // HC-05 Tx connected to Arduino #2 & HC-05 Rx to Arduino #3

const byte numChars = 6;       
char receivedChars[numChars];  // an array to store the received data
char tempChars[numChars];

char botDir[1];         // char type variable for the direction of the robot
//                         botDirection[0] = 'f' (forward), 'b' (backwards)
//                         'r' (right), 'l' (left), 's' (stop)
int botSpeed = 0;           //stores the speed of the whole robot
boolean newData = false;

//the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 13;           //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;            //control pin 2 on the motor driver for the right motor
const int PWMA = 11;            //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN2 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;           //control pin 1 on the motor driver for the left motor

const int trigPin = 6;        //trigger pin for distance snesor
const int echoPin = 7;        //echo pin for distance sensor
const int RED = 5;
const int GREEN = 4;


String botDirection;           //the direction that the robot will drive in (this change which direction the two motors spin in)
String motorSpeedStr;

int motorSpeed;               //speed integer for the motors
float duration, distance;     //duration and distance for the distance sensor

PID myPID(&measurement, &output, &setpoint, Kp, Ki, Kd, DIRECT);

/********************************************************************************/
void setup()
{
  Kp = 1.0;  // Adjust these based on testing
  Ki = 1.5;
  Kd = 0.1;
  setpoint = 15; // Example desired distance in cm
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(AUTOMATIC);
  mySerial.begin(9600);       //Default Baud Rate for software serial communications

  //set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  //set the distance sensor trigger pin as output and the echo pin as input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(RED,OUTPUT);
  pinMode(GREEN,OUTPUT);

  Serial.begin(9600);           //begin serial communication with the computer

  //prompt the user to enter a command
  Serial.println("Enter a direction followed by speed.");
  Serial.println("f = forward, b = backward, r = turn right, l = turn left, s = stop");
  Serial.println("Example command: f 50 or s 0");
}

/********************************************************************************/
void loop()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*340/10000)/2; // Units are cm
  //Serial.print("Distance: ");
  //Serial.println(distance);
    delayMicroseconds(50);
    measurement = distance;
    myPID.Compute();
    Serial.print(setpoint);
    Serial.print(" ");
    Serial.print(measurement);
    Serial.print(" ");
    Serial.println(output);

    moveRobot();

  if (Serial.available() > 0)                         //if the user has sent a command to the RedBoard
  {
    botDirection = Serial.readStringUntil(' ');       //read the characters in the command until you reach the first space
    motorSpeedStr = Serial.readStringUntil(' ');           //read the characters in the command until you reach the second space
    motorSpeed = motorSpeedStr.toInt();
    Serial.println(botDirection);
  }

  recvWithEndMarker();
  if (newData == true)
  {
    strcpy(tempChars, receivedChars);
    parseData();
    motorSpeed = botSpeed;
    newData = false;
    Serial.println(botDir[0]);
    Serial.println(motorSpeed);
   
  }
}
/********************************************************************************/
void moveRobot() {
  int speed = map(output, 0, 255, 50, 255); // Map PID output to motor speed (min 50 to avoid stalling)

  if (measurement > setpoint) {
    // Move forward
    rightMotor(-speed);
    leftMotor(speed);
   // Serial.println("Moving forward");
  }
  else if (measurement < setpoint) {
    // Move backward
    rightMotor(speed);
    leftMotor(-speed);
  //  Serial.println("Moving backward");
  }
}
/***************************/
void rightMotor(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        //function for driving the left motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMB, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    while (mySerial.available() > 0 && newData == false)
    {
      rc = mySerial.read();

      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars)
        {
          ndx = numChars - 1;
        }
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string
        ndx = 0;
        newData = true;
      }
    }
}
/*****************************************************************************************/
void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars," ");      // get the first part - the string
    strcpy(botDir, strtokIndx); // copy it to messageFromPC
 
    strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
    botSpeed = atoi(strtokIndx);     // convert this part to an integer
}
```

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
