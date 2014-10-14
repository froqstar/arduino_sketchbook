// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>


#define RAD_TO_DEG 57.2958279088


const int MPU=0x68;  // I2C address of the MPU-6050

//int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

//angles
float angle_x;
float angle_y;
float angle_z;

//calibration values
float    base_acc_x;
float    base_acc_y;
float    base_acc_z;
float    base_tmp;
float    base_gyro_x;
float    base_gyro_y;
float    base_gyro_z;

//IMU values
typedef struct {
  long AX;
  long AY;
  long AZ;
  long TMP;
  long GX;
  long GY;  
  long GZ;
} imu_readings;

//timestamp of last IMU reading
unsigned long last_update;


int read_imu_raw(uint8_t *ptr) {
  imu_readings* data = (imu_readings *) ptr;
  
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  data->AX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  data->AY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  data->AZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  data->TMP=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  data->GX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  data->GY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  data->GZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  /*
  Serial.print("rawX = "); Serial.print(data->AX);
  Serial.print(" | rawY = "); Serial.print(data->AY);
  Serial.print(" | rawZ = "); Serial.print(data->AZ);
  Serial.print(" | raw Tmp = "); Serial.print(data->TMP); 
  Serial.print(" | rawX = "); Serial.print(data->GX);
  Serial.print(" | rawY = "); Serial.print(data->GY);
  Serial.print(" | rawZ = "); Serial.println(data->GZ);
  */
}


void calibrate_imu() {
  imu_readings raw;
  imu_readings sum = {0,0,0,0,0,0,0};
  read_imu_raw((uint8_t *) &raw);
  
  int readings = 20;
  
  for (int i=0; i<readings; i++) {
    read_imu_raw((uint8_t *) &raw);
    //Serial.print("sumX = "); Serial.println(sum.AX);
    sum.AX += raw.AX;
    sum.AY += raw.AY;
    sum.AZ += raw.AZ;
    sum.TMP += raw.TMP;
    sum.GX += raw.GX;
    sum.GY += raw.GY;
    sum.GZ += raw.GZ;
    delay(100);
  }
  base_acc_x = sum.AX / readings;
  base_acc_y = sum.AY / readings;
  base_acc_z = sum.AZ / readings;
  base_tmp = sum.TMP / readings;
  base_gyro_x = sum.GX / readings;
  base_gyro_y = sum.GY / readings;
  base_gyro_z = sum.GZ / readings;
}


int read_imu(uint8_t *ptr) {
  read_imu_raw(ptr);
  imu_readings *data = (imu_readings *) ptr;
  
  data->AX = data->AX - base_acc_x;
  //data->AX = map(data->AX, -90, 90, -32768, 32768);
  data->AY = data->AY - base_acc_y;
  //data->AY = map(data->AY, -90, 90, -32768, 32768);
  data->AZ = data->AZ - base_acc_z;
  //data->AZ = map(data->AZ, -90, 90, -32768, 32768);
  data->TMP = (data->TMP - base_tmp)/340.00+36.53;
  data->GX = (data->GX - base_gyro_x)/131;
  data->GY = (data->GY - base_gyro_y)/131;
  data->GZ = (data->GZ - base_gyro_z)/131;
}

int update_angles() {
  unsigned long now = millis();
  imu_readings raw;
  read_imu((uint8_t *) &raw);
  
  float accel_angle_y = atan(-1*raw.AX / sqrt(pow(raw.AY,2) + pow(raw.AZ,2))) * RAD_TO_DEG;
  float accel_angle_x = atan(raw.AY / sqrt(pow(raw.AX,2) + pow(raw.AZ,2))) * RAD_TO_DEG;
  float accel_angle_z = 0;

  float dt = (now - last_update) / 1000.0;
  float gyro_angle_x = raw.GX * dt + angle_x;
  float gyro_angle_y = raw.GY * dt + angle_y;
  float gyro_angle_z = raw.GZ * dt + angle_z;

  //complementary filter
  float alpha = 0.98;
  angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

  last_update = millis();
}


void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(57600);
  calibrate_imu();
  
  delay(1000);
  
  Serial.print("CalX = "); Serial.print(base_acc_x);
  Serial.print(" | CalY = "); Serial.print(base_acc_y);
  Serial.print(" | CalZ = "); Serial.print(base_acc_z);
  Serial.print(" | Tmp = "); Serial.print(base_tmp);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(base_gyro_x);
  Serial.print(" | GyY = "); Serial.print(base_gyro_y);
  Serial.print(" | GyZ = "); Serial.println(base_gyro_z);
  
  delay(1000);
}

void loop(){
  update_angles();
  
  Serial.print("Angle X = "); Serial.print(angle_x);
  Serial.print(" | Angle Y = "); Serial.print(angle_y);
  Serial.print(" | Angle Z = "); Serial.println(angle_z);
  
  //delay(1000);
}
