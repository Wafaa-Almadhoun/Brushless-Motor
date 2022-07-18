# Brushless-Motor
# Stepper-motor-using-Arduino-UNO-R3-



## Table of contents
* [Introduction](#Introduction)
* [Technologies](#technologies)
* [Components required](#Components-required)
* [Connections](#Connections)
* [Block diagram & simulation ](#Block-diagram-&-simulation)



## Introduction
 A brushless DC electric motor , is a synchronous motor using a direct current (DC) electric power supply. It uses an electronic 
 controller to switch DC currents to the motor windings producing magnetic fields which effectively rotate in space and which 
 the permanent magnet rotor follows. The controller adjusts the phase and amplitude of the DC current pulses to control the 
 speed and torque of the motor ,Brushless motors find applications in such places as computer peripherals (disk drives, printers), 
 hand-held power tools, and vehicles ranging from model aircraft to automobiles. In modern washing machines, brushless DC motors have
 allowed replacement of rubber belts and gearboxes by a direct-drive design

  ,We will cover several topics : 👍 


 1. Brushless Motor A2212 Controlled by Arduino and ESC .
 2. Brushless DUAL SHAFT MOTOR - D5065 270KV Controlled by Arduino and Odrive .
  


## Technologies
Project is created with:
* Arduino IDE 1.8.19 [To Downloud](https://www.arduino.cc/en/software)

	
## Components required
### 1. Brushless Motor A2212 Controlled by Arduino and ESC 

  Brushless Motor A2212
  
  ESC 30A 
  
  Li-Po battery 3 cells 
  
  Arduino Board 
  
  10 k ohm Potentiometer 
  
  Breadboard and Jump Wires …
  
 ### 2. Brushless DUAL SHAFT MOTOR - D5065 270KV Controlled by Arduino and Odrive .
 
    
## Connections

### 1. Brushless Motor A2212 Controlled by Arduino and ESC 

     connect the three terminals of Brushless motor to the three terminals of the ESC
     
     Screw the Motor to a heavy wooden plank anything similar so that it remains stable at high RPM
     
     Connect the signal wire of ESC white color PWM pin 9 in Arduino, 
     
     Connect the GND & V+ wire of ESC to  GND & V+  in Arduino,
     
     Connect the GND & V+ wire of ESC to  GND & V+  in Arduino,
     
     Connect the battery VCC  and GND battery wire of ESC to Li-Po battery
     
     Connecting GND in Ardunio to the one side of potentiometer
     
     Connecting +5v in Ardunio to the other side of potentiometer
     
     Connecting pin A0 in Ardunio to the signal of potentiometer
     


     
 ### 2. Brushless DUAL SHAFT MOTOR - D5065 270KV Controlled by Arduino and Odrive .
 
    connecting 5V output on Arduino to the Vcc2 & Vcc1 pins
    
    Connect ground to ground.
    
    connect the input pins(IN1, IN2, IN3 and IN4) of the L293D IC to 
    
    four digital output pins(12, 11, 10 and 9) on Arduino
    0ne coil of stepper moter connecting to Out1 & Out2 and the anthor coil connecting to Out3 & Out4
    
 
 ### 3. BIG Stepper Motors NEMA 23 Bipolar with DM860A Microstep Driver 
 
     connecting pin6 in Ardunio to -DIR in DM860A 
     connecting pin7 in Ardunio to -PUL in DM860A
     connecting +5v output in Ardunio to +PUL & +DIR in DM860A
     connecting pin2 in Ardunio to one side of push button and also the 10 K ohm resistor connection up to the +5v on the arduino 
     connecting GND in Ardunio to the other side of push button
     connecting GND in Ardunio to the one side of potentiometer
     connecting +5v in Ardunio to the other side of potentiometer
     connecting pin A0 in Ardunio to the signal of potentiometer
     connecting the motor driver with 24v battery
     
     
## Block diagram & simulation
### 1. Brushless Motor A2212 Controlled by Arduino and ESC 

![Untitled Sketch 4_bb](https://user-images.githubusercontent.com/64277741/179435815-1bd9a5ea-6ab4-4b05-a3c1-88ec4007e8c9.png)


Figure (5): Block diagram and Connections of Brushless Motor A2212



#### The Code 
 #include <Servo.h>  //Using servo library to control ESC

Servo esc; //Creating a servo class with name as esc

void setup()

{

esc.attach(9); //Specify the esc signal pin,Here as D8

esc.writeMicroseconds(1000); //initialize the signal to 1000



}

void loop()

{

int val; //Creating a variable val

val= analogRead(A0); //Read input from analog pin a0 and store in val

val= map(val, 0, 1023,1000,2000); //mapping val to minimum and maximum 

esc.writeMicroseconds(val); //using val as the signal to esc

}

### 2. Bipolar Stepper with L293D Motor Driver IC .[see here ](https://github.com/Wafaa-Almadhoun/Stepper-motor-using-Arduino-UNO-R3-/blob/main/Bipolar%20Stepper%20with%20L293D%20Motor%20Driver%20IC.pdsprj)
![1](https://user-images.githubusercontent.com/64277741/179328636-268173e6-09b8-46fb-9431-1dfe2eae640f.PNG)
Figure (7): step one revolution in the other direction ("counterclockwise")

 ![2](https://user-images.githubusercontent.com/64277741/179328701-3dee3532-ada8-4ae9-abdd-f15dcee8762f.PNG)
Figure (8): step one revolution in one direction ("clockwise")

#### The code 

// Include the Arduino Stepper Library
#include <Stepper.h>

// Number of steps per output rotation NEMA 17

const int stepsPerRevolution = 200; 

// Create Instance of Stepper library

Stepper myStepper(stepsPerRevolution, 12, 11, 10, 9);


void setup()
{
  // set the speed at 20 rpm:
  
  myStepper.setSpeed(20);
  
}

void loop() 
{
  // step one revolution in one direction:
  
  myStepper.step(stepsPerRevolution);
  
  delay(1000);

  // step one revolution in the other direction:
  
  myStepper.step(-stepsPerRevolution);
  
  delay(1000);
}


### 3. BIG Stepper Motors NEMA 23 Bipolar with DM860A Microstep Driver  
![3BIG Stepper Motors NEMA 23 Bipolar with DM860A Microstep Driver](https://user-images.githubusercontent.com/64277741/179338072-d89222ff-f4ea-4005-a69e-4427b546f48d.png)

#### The Code 
// Defin pins
 
int reverseSwitch = 2;  // Push button for reverse
int driverPUL = 7;    // PUL- pin
int driverDIR = 6;    // DIR- pin
int spd = A0;     // Potentiometer
 
// Variables
 
int pd = 500;       // Pulse Delay period
boolean setdir = LOW; // Set Direction
 
// Interrupt Handler
 
void revmotor (){
 
  setdir = !setdir;
  
}
 
 
void setup() {
 
  pinMode (driverPUL, OUTPUT);
  pinMode (driverDIR, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(reverseSwitch), revmotor, FALLING);
  
}
 
void loop() {
  
    pd = map((analogRead(spd)),0,1023,2000,50);
    digitalWrite(driverDIR,setdir);
    digitalWrite(driverPUL,HIGH);
    delayMicroseconds(pd);
    digitalWrite(driverPUL,LOW);
    delayMicroseconds(pd);
 
}

