
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


// includes for NRF24L01+
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>

#include <Servo.h> 
 
Servo front_left, front_right, rear_left, rear_right;
RF24 radio(7,8);

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#define MOTOR_FRONT_LEFT_CCW 5
#define MOTOR_FRONT_RIGHT_CW 6
#define MOTOR_REAR_LEFT_CW 9
#define MOTOR_REAR_RIGHT_CCW 10

#define THRUST_STEADY 100

bool blinkState = false;
volatile bool printed = false;

byte control[4]; 

int level = 0; //10 to 175 are presumably good values


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    //setup pins
    front_left.attach(MOTOR_FRONT_LEFT_CCW);
    front_right.attach(MOTOR_FRONT_RIGHT_CW);
    rear_left.attach(MOTOR_REAR_LEFT_CW);
    rear_right.attach(MOTOR_REAR_RIGHT_CCW);
    
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
  
    Wire.begin();

    Serial.begin(57600);

    //setup radio
    radio.begin();
    radio.openReadingPipe(1,0xF0F0F0F0D2LL);
    radio.startListening();
    attachInterrupt(1, check_radio, FALLING);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {  
   
    //calculate motor levels
    if (Serial.available()) {
      level = Serial.parseInt();
      printed = false;
    }
    
    front_left.write(level);
    front_right.write(level);
    rear_left.write(level);
    rear_right.write(level);

    if (!printed) {
      Serial.println(level);
      Serial.print(control[0]);
      Serial.print("\t");
      Serial.print(control[1]);
      Serial.print("\t");
      Serial.print(control[2]);
      Serial.print("\t");
      Serial.println(control[3]);
      printed = true;  
    }
}

void check_radio(void) {
  radio.stopListening();
  // What happened?
  bool tx,fail,rx;
  radio.whatHappened(tx,fail,rx);
  
  // Did we receive a message?
  if (rx)
  {
      radio.read(&control,4);
      printed = false;
  }
  radio.startListening();
}
