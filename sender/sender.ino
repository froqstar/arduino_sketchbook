#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>



RF24 radio(9,10);

int inputX1 = A2;
int inputX2 = A0;
int inputY1 = A3;
int inputY2 = A1;


void setup(){
  Serial.begin(57600);
  
  pinMode(7, INPUT);
  digitalWrite(7,HIGH);
  
  delay(2000);
  Serial.println("hi there!");
  
  radio.begin();
  radio.openWritingPipe(0xF0F0F0F0D2LL);
  
  radio.printDetails();
}


void loop(){
 byte data[4];
 
 int x1 = analogRead(inputX1); 
 byte out_x1 = map(x1, 0, 1023, 0, 90); 
 
 int x2 = analogRead(inputX2); 
 byte out_x2 = map(x2, 0, 1023, 0, 90);
 
 int y1 = analogRead(inputY1); 
 byte out_y1 = map(y1, 0, 1023, 0, 90);
 
 int y2 = analogRead(inputY2); 
 byte out_y2 = map(y2, 0, 1023, 0, 90);
 
 data[0] = out_x1;
 data[1] = out_x2;
 data[2] = out_y1;
 data[3] = out_y2;
 
 bool ok = radio.write(&data,4);
     
 Serial.print("x1 = ");      
 Serial.print(out_x1); 
 Serial.print("\t x2 = ");      
 Serial.print(out_x2);
 Serial.print("\t y1 = ");      
 Serial.print(out_y1);
 Serial.print("\t y2 = ");      
 Serial.println(out_y2);
 
 if (ok) {
   Serial.println("success");
 } else {
   Serial.println("failed");
 }
 delay(10);
}
