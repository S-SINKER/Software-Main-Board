#include "lib/mpu9250.h"
HardWire HWire(1, I2C_FAST_MODE); // I2c1
MPU9250 IMU;
float acc[3];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  HWire.begin();
  IMU.initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
  IMU.read_mag();
  Serial.print(IMU.magnetometer_data[0]);
  Serial.print(" ");
  Serial.print(IMU.magnetometer_data[1]);
  Serial.print(" ");
  Serial.print(IMU.magnetometer_data[2]);
  Serial.print("\n");
  delay(1000);
}
