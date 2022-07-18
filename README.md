# Brushless-Motor-using-Arduino




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


 1. Brushless Motor A2212 Controlled by Arduino  Uno and ESC .
 2. Brushless DUAL SHAFT MOTOR - D5065 270KV Controlled by Arduino and Odrive .
  


## Technologies
Project is created with:
* Arduino IDE 1.8.19 [To Downloud](https://www.arduino.cc/en/software)

	
## Components required
### 1. Brushless Motor A2212 Controlled by Arduino Uno and  ESC 

 1- Brushless Motor A2212
  
 2-  ESC 30A 
  
 3- Li-Po battery 3 cells 
  
 4- Arduino  uno Board 
  
 5- 10 k ohm Potentiometer 
  
 6- Breadboard and Jump Wires …
  
 ### 2. Brushless DUAL SHAFT MOTOR - D5065 270KV Controlled by Arduino and Odrive .
 
  1- DUAL SHAFT MOTOR - D5065 270KV
  
  2-  HSA50R47J Resistor, Solder Lug, 0.47 ohm, HS Series, 50 W, ± 5%, Solder Lug, 1.25 kV
  
  3- Orive v3.2 controller 
  
  4- power supplay 24v 
  
  5- caples connection 
  
## Connections

### 1. Brushless Motor A2212 Controlled by Arduino Uno and ESC 

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
 
      connecting pin18 in Ardunio mega to GPIO 2  in Odrive  

      connecting pin19 in Ardunio mega to GPIO 1  in Odrive  

      connecting  GND in Ardunio mega to  GND  in Odrive  
       
      connecting pin A in motor to M0  pin A  in Odrive
    
      connecting pin B in motor to M0  pin B  in Odrive
     
      connecting pin c in motor to M0  pin c  in Odrive
      
      connecting AUX + -  in Odrive with the resistor  

      connecting DC + -  in Odrive with the power supplay
      

 
     
## Block diagram & simulation
### 1. Brushless Motor A2212 Controlled by Arduino  Uno and ESC 

![Untitled Sketch 4_bb](https://user-images.githubusercontent.com/64277741/179435815-1bd9a5ea-6ab4-4b05-a3c1-88ec4007e8c9.png)


Figure (1): Block diagram and Connections of Brushless Motor A2212



#### The Code 


select in Arduino IDE tools> board >  Arduino uno ,
 select in Arduino IDE tools> port > select the port ,
 then uploud the code to Arduino uno 
 
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

### 2. Brushless DUAL SHAFT MOTOR - D5065 270KV Controlled by Arduino and Odrive
![11](https://user-images.githubusercontent.com/64277741/179448376-8f427cc7-c8f6-40b6-ac82-e82866ad997f.png)

Figure (2): Block diagram and Connections of Brushless Motor D5065


#### The code 
you have to download ODriveArduino and HardwareSerial library 
 and select in Arduino IDE tools> board >  Arduino mega ,
 select in Arduino IDE tools> port > select the port ,
 then uploud the code to Arduino mega
 

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Below are some sample configurations.
// You can comment out the default Teensy one and uncomment the one you wish to use.
// You can of course use something different if you like
// Don't forget to also connect ODrive GND to Arduino GND.

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
HardwareSerial& odrive_serial = Serial1;

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
HardwareSerial& odrive_serial = Serial1;

// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this is implemented poorly and can lead to wrong data sent or read.
// pin 8: RX - connect to ODrive TX
// pin 9: TX - connect to ODrive RX
// SoftwareSerial odrive_serial(8, 9);


// ODrive object
ODriveArduino odrive(odrive_serial);

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 10.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

  Serial.println("Ready!");
  Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 's' to exectue test move");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();

    // Run calibration sequence
    if (c == '0' || c == '1') {
      int motornum = c-'0';
      int requested_state;

      requested_state = AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, true)) return;

      requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, true, 25.0f)) return;

      requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, false /*don't wait*/)) return;
    }

    // Sinusoidal test move
    if (c == 's') {
      Serial.println("Executing test move");
      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
        float pos_m0 = 2.0f * cos(ph);
        float pos_m1 = 2.0f * sin(ph);
        odrive.SetPosition(0, pos_m0);
        odrive.SetPosition(1, pos_m1);
        delay(5);
      }
    }

    // Read bus voltage
    if (c == 'b') {
      odrive_serial << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }

    // print motor positions in a 10s loop
    if (c == 'p') {
      static const unsigned long duration = 10000;
      unsigned long start = millis();
      while(millis() - start < duration) {
        for (int motor = 0; motor < 2; ++motor) {
          Serial << odrive.GetPosition(motor) << '\t';
        }
        Serial << '\n';
      }
    }
  }
}

	  
