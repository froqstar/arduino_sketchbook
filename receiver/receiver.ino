#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>



RF24 radio(7,8);

byte message[4];
volatile bool printed = false;

void setup() {
  radio.begin();
  Serial.begin(57600);
  radio.openReadingPipe(1,0xF0F0F0F0D2LL);
  radio.startListening();
  
  attachInterrupt(1, check_radio, FALLING);
}
void loop(){
  if (!printed) {
    Serial.print(message[0]);
    Serial.print("\t");
    Serial.print(message[1]);
    Serial.print("\t");
    Serial.print(message[2]);
    Serial.print("\t");
    Serial.println(message[3]);
    printed = true;  
  }
}

void check_radio(void)
{
  radio.stopListening();
  // What happened?
  bool tx,fail,rx;
  radio.whatHappened(tx,fail,rx);

  // Did we receive a message?
  if ( rx )
  {
      radio.read(&message,4);
      printed = false;
  }
  radio.startListening();
}
