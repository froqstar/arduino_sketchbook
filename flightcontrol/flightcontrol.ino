
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


// includes for NRF24L01+
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>


#include <Servo.h> 


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#define MOTOR_FRONT_LEFT_CCW 5
#define MOTOR_FRONT_RIGHT_CW 6
#define MOTOR_REAR_LEFT_CW 9
#define MOTOR_REAR_RIGHT_CCW 10

#define THRUST_STEADY 100
#define THRUST_MIN 10
#define THRUST_MAX 160

MPU6050 mpu;
RF24 radio(7,8);
Servo front_left, front_right, rear_left, rear_right;

bool blinkState = false;
volatile bool printed = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float gyro[3];          // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float state[4];      // [yaw, pitch, roll, height]  current yaw/pitch/roll/height
byte control[4];        // [yaw, pitch, roll, height]  desired yaw/pitch/roll/height












// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    //setup servos
    front_left.attach(MOTOR_FRONT_LEFT_CCW);
    front_right.attach(MOTOR_FRONT_RIGHT_CW);
    rear_left.attach(MOTOR_REAR_LEFT_CW);
    rear_right.attach(MOTOR_REAR_RIGHT_CCW);
    
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
  
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    Serial.begin(57600);

    //setup MPU
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    //setup radio
    radio.begin();
    radio.openReadingPipe(1, 0xF0F0F0F0D2LL);
    radio.startListening();
    attachInterrupt(1, check_radio, FALLING);
    
    //arm ESCs
    front_left.write(THRUST_MIN);
    front_right.write(THRUST_MIN);
    rear_left.write(THRUST_MIN);
    rear_right.write(THRUST_MIN);
    delay(5000);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;


    //regulate here
    //TODO: convert angles to degrees!
    //TODO: calibrate gyro/substract offsets
    
    //calculate factors
    int factor_height = THRUST_STEADY + control[3];
    int factor_yaw = control[0];
    int factor_pitch = control[1] - state[1];
    int factor_roll = control[2] - state[2];
    
    //calculate motor levels
    int level_fl_ccw = factor_height - factor_yaw - factor_pitch + factor_roll;
    int level_fr_cw  = factor_height + factor_yaw - factor_pitch - factor_roll;
    int level_rl_cw  = factor_height + factor_yaw + factor_pitch + factor_roll;
    int level_rr_ccw = factor_height - factor_yaw + factor_pitch - factor_roll;
    
    //write values to the motors
    front_left.write(constrain(level_fl_ccw, THRUST_MIN, THRUST_MAX));
    front_right.write(constrain(level_fr_cw, THRUST_MIN, THRUST_MAX));
    rear_left.write(constrain(level_rl_cw, THRUST_MIN, THRUST_MAX));
    rear_right.write(constrain(level_rr_ccw, THRUST_MIN, THRUST_MAX));

    // get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(gyro, &q, &gravity);
        if (printed) {
          Serial.print("ypr\t");
          Serial.print(gyro[0] * 180/M_PI);
          Serial.print("\t");
          Serial.print(gyro[1] * 180/M_PI);
          Serial.print("\t");
          Serial.println(gyro[2] * 180/M_PI);
        }

        //update current state
        state[0] = gyro[0] * 180/M_PI;
        state[1] = gyro[1] * 180/M_PI;
        state[2] = gyro[2] * 180/M_PI;

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    
    if (!printed) {
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
