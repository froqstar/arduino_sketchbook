#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>

/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
 */

int led = 13;

// the setup routine runs once when you press reset:
void setup() {
  
  RF24 radio(9,10);
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = digitalRead(A3);

  Serial.println(sensorValue);
  delay(1);        // delay in between reads for stability
}
