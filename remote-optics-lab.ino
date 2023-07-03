#include "secrets.h"
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Arduino.h>
#include <WiFi.h>

/* -------------------------------------------------------- 
   -------- SENSOR HC-SR04 CONSTANTS AND VARIABLES --------
   -------------------------------------------------------- */

#define TEMPERATURE 20                  // temperature in celsius
#define HUMIDITY 50                     // humidity in %
#define SOUND_SPEED_0 331.4             // speed of sound at 0 degrees celsius

#define trigPin 32                      // trig input of the HC-SR04
#define echoPin 33                      // echo output of the HC-SR04

long duration;                          // to store the duration of the echo pulse
float speed;                             // to store the speed of sound
float distanceCm;                        // to store the distance measured in cm

/* --------------------------------------------------------
   --------- STEPPER MOTORS PIN CONSTANTS AND VARIABLES ---
   -------------------------------------------------------- */

#define stepA 23                        // pin step motor A
#define stepB 22                        // pin step motor B
#define dirA 21                         // pin direction motor A
#define dirB 19                         // pin direction motor B
#define endstopA 5                      // pin endstop A
#define endstopB 17                     // pin endstop B
#define enableMotors 18                 // pin enable motors
#define STEPS_PER_MM 80                 // steps per mm with 1/16 microstepping and gt2 belt of 20 teeth

AccelStepper motorA(1, stepA, dirA);
AccelStepper motorB(1, stepB, dirB);
MultiStepper motors;

long posiciones[2];                     // array to store the positions of the motors
long homing;                            // to store the position of the motors when homing

/* --------------------------------------------------------
   ------ STATE MACHINE CONSTANTS AND VARIABLES -----------
   -------------------------------------------------------- */
String command;                         // moveToHome, moveToPosition, stop
float endpointPosition;                 // from 10cm to 50cm
float currentPosition;                  // current position of the motors
String powerMotors;                     // on, off
String statusMotorA;                    // moving, stopped
String statusMotorB;                    // moving, stopped
String status_connection;               // connected, disconnected
String statusFan;                       // on, off

void setup() {
  Serial.begin(9600);                   // start the serial communication

  // sensor pins
  pinMode(trigPin, OUTPUT);             
  pinMode(echoPin, INPUT_PULLUP);              
  
  // motor pins                         
  pinMode(stepA, OUTPUT);              
  pinMode(stepB, OUTPUT);              
  pinMode(dirA, OUTPUT);               
  pinMode(dirB, OUTPUT);               
  pinMode(enableMotors, OUTPUT);        

  pinMode(endstopA, INPUT_PULLUP);          
  pinMode(endstopB, INPUT_PULLUP);             
  
  // motor setup
  motors.addStepper(motorA);
  motors.addStepper(motorB);

  
  Serial.println("Setting motorA speed and acceleration...");
  motorA.setMaxSpeed(200);
  motorA.setSpeed(200);
  motorA.setAcceleration(4);  

  Serial.println("Setting motorB speed and acceleration...");
  motorB.setMaxSpeed(200);
  motorB.setSpeed(200);
  motorB.setAcceleration(4);      
  
  command = "moveToHome";                     // initial state of the state machine

  delay(1000);          
 
  digitalWrite(enableMotors, HIGH); // disable motors

}


void loop() {
  
    Serial.println("Waiting for command...");
    while (Serial.available() == 0) {
        delay(100);
    }

    command = Serial.readStringUntil('\n');
    Serial.println("Command received: " + command);



  if (command == "moveToHome") {
        moveToHome();
    } else if (command == "moveToPosition") {
        int position = 10;  
        moveToPosition(position);
    } else if (command == "stop") {
        stopMotors();
    } else {
        Serial.println("Unknown command: " + command);
    }

  delay(1000); 
}




/* -------------------------------------------------------- 
  function to get the distance measured by the sensor in cm
  -------------------------------------------------------- */

float getDistance() {
  digitalWrite(trigPin, LOW);           // clear the trigPin
  delayMicroseconds(2);                 // wait for 2 micro seconds
  digitalWrite(trigPin, HIGH);          // set the trigPin on HIGH state for 10 micro seconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);           // clear the trigPin
  duration = pulseIn(echoPin, HIGH);    // Read the echoPin, returns the sound wave travel time in microseconds

  // speed of sound in m/s with temperature and humidity compensation
  speed = SOUND_SPEED_0 + (0.606 * TEMPERATURE) + (0.0124 * HUMIDITY); 

  // calculate the distance in cm and return it
  distanceCm = duration * (speed/10000)/2;
  Serial.println("Distance: " + String(distanceCm) + " cm");

  return distanceCm;
}

void moveMotorToHome(AccelStepper& motor, int endstop) {
    Serial.println("Moving motor...");
    int homing = -1;
      do
    {
      motor.moveTo(homing);
      homing--;
      motor.run();
    } while(digitalRead(endstop)==HIGH);

homing=1;
    do
    {
      motor.moveTo(homing);
      homing++;
      motor.run();
    } while(digitalRead(endstop)==LOW);
}

/* --------------------------------------------------------
   moves all the motors to their initial or "home" position. 
   --------------------------------------------------------*/ 
void moveToHome() {
    Serial.println("Starting homing...");
    digitalWrite(enableMotors, LOW);

    moveMotorToHome(motorA, endstopA);
    moveMotorToHome(motorB, endstopB);


    posiciones[0] = 0;
    posiciones[1] = 75;
    motorB.setCurrentPosition(posiciones[1]*STEPS_PER_MM);
    
    Serial.println("Homing finished!");
}


/* --------------------------------------------------------
   moves the motors to the position specified by the user.
   -------------------------------------------------------- */

void moveToPosition(int position)
{
    digitalWrite(enableMotors, LOW);
    delay(100);

    double actual = getDistance();
    while (abs(position - actual) > 1) {  // Asume una tolerancia de 1
        if (position > actual) {
            // Si la posición deseada es mayor que la actual, mueve los motores en la dirección positiva
            motorA.move(1);
            motorB.move(1);
        } else {
            // Si la posición deseada es menor que la actual, mueve los motores en la dirección negativa
            motorA.move(-1);
            motorB.move(-1);
        }

        // Ejecuta los movimientos de los motores
        motorA.run();
        motorB.run();

        // Actualiza la posición actual
        actual = getDistance();
    }

    digitalWrite(enableMotors, HIGH);
}

void moveToPositionWithPID()
{
  Serial.println("Moving motors to position with PID...");

}

void stopMotors()
{
  Serial.println("Stopping motors...");
}



void connectToWifi()
{
  Serial.println("Connecting to WiFi...");
  
  WiFi.begin(SSID, PASSWORD);


}