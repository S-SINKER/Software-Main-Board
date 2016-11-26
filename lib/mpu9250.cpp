/*
Written by YongJae Kim (dydwo92@snu.ac.kr)
Adapted for Maple Mini by rogerclarkmelbourne (https://github.com/rogerclarkmelbourne/Arduino_STM32)
Library is based on the emlid/Navio(https://github.com/emlid/Navio/tree/master/C%2B%2B/Navio)
*/

#include "mpu9250.h"

#define G_SI 9.80665
#define PI 3.14159

MPU9250::MPU9250(){

}
/*--------------------------------------------------------------------
              BASIC READ & WRITE FUNCTIONS FOR MPU9250
--------------------------------------------------------------------*/
uint8_t MPU9250::WriteReg(uint8_t WriteAddr, uint8_t WriteData){
  HWire.beginTransmission(MPU9250_ADDR);
  HWire.write(WriteAddr);
  HWire.write(WriteData);
  return HWire.endTransmission();
}
/*-------------------------------------------------------------------*/
uint8_t MPU9250::ReadReg(uint8_t ReadAddr){
  HWire.beginTransmission(MPU9250_ADDR);
  HWire.write(ReadAddr);
  HWire.endTransmission();
  HWire.requestFrom(MPU9250_ADDR, 1);
  return HWire.read();
}
/*-------------------------------------------------------------------*/
void MPU9250::ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes){
  unsigned int i;
  HWire.beginTransmission(MPU9250_ADDR);
  HWire.write(ReadAddr);
  HWire.endTransmission();
  HWire.requestFrom(MPU9250_ADDR, Bytes);

  for(i=0; i<Bytes; i++)
    ReadBuf[i] = HWire.read();
}

/*--------------------------------------------------------------------
                          TEST CONNECTION
usage : return 1 if mpu9250 is connected, else return 0
--------------------------------------------------------------------*/
bool MPU9250::testConnection(void){
  unsigned int response;
  response = ReadReg(MPUREG_WHOAMI);

  if(response == 0x71)
    return true;
  else
    return false;
}

/*--------------------------------------------------------------------
                          INITIALIZATION
--------------------------------------------------------------------*/
#define MPU_InitRegNum 16
void MPU9250::initialize(int sample_rate_div, int low_pass_filter){
  uint8_t i = 0;
  uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
      //{0x80, MPUREG_PWR_MGMT_1},     // Reset Device - Disabled because it seems to corrupt initialisation of AK8963
      {0x01, MPUREG_PWR_MGMT_1},     // Clock Source
      {0x00, MPUREG_PWR_MGMT_2},     // Enable Acc & Gyro
      {low_pass_filter, MPUREG_CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
      {0x18, MPUREG_GYRO_CONFIG},    // +-2000dps
      {0x08, MPUREG_ACCEL_CONFIG},   // +-4G
      {0x09, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
      {0x30, MPUREG_INT_PIN_CFG},    //
      //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
      //{0x20, MPUREG_USER_CTRL},      // Enable AUX
      {0x20, MPUREG_USER_CTRL},       // I2C Master mode
      {0x0D, MPUREG_I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz

      {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
      //{0x09, MPUREG_I2C_SLV4_CTRL},
      //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay

      {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
      {0x01, MPUREG_I2C_SLV0_DO}, // Reset AK8963
      {0x81, MPUREG_I2C_SLV0_CTRL},  //Enable I2C and set 1 byte

      {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
      {0x12, MPUREG_I2C_SLV0_DO}, // Register value to continuous measurement in 16bit
      {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
  };

  for(i=0;i<MPU_InitRegNum; i++){
    WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
    delay(2); //I2C must slow down the write speed, otherwise it won't work
  }

    set_acc_scale(BITS_FS_16G);
    set_gyro_scale(BITS_FS_2000DPS);
    calib_mag();
}
/*-------------------------------------------------------------------*/
unsigned int MPU9250::set_acc_scale(uint8_t scale){
    WriteReg(MPUREG_ACCEL_CONFIG, scale);
    switch (scale){
        case BITS_FS_2G:
            acc_divider=16384;
        break;
        case BITS_FS_4G:
            acc_divider=8192;
        break;
        case BITS_FS_8G:
            acc_divider=4096;
        break;
        case BITS_FS_16G:
            acc_divider=2048;
        break;
      }

    scale = ReadReg(MPUREG_ACCEL_CONFIG);
    switch (scale){
        case BITS_FS_2G:
            return 2;
        break;
        case BITS_FS_4G:
            return 4;
        break;
        case BITS_FS_8G:
            return 8;
        break;
        case BITS_FS_16G:
            return 16;
        break;
      }

      return 0;
}
/*-------------------------------------------------------------------*/
unsigned int MPU9250::set_gyro_scale(uint8_t scale){
  WriteReg(MPUREG_GYRO_CONFIG, scale);
  switch (scale){
      case BITS_FS_250DPS:
          gyro_divider=131;
      break;
      case BITS_FS_500DPS:
          gyro_divider=65.5;
      break;
      case BITS_FS_1000DPS:
          gyro_divider=32.8;
      break;
      case BITS_FS_2000DPS:
          gyro_divider=16.4;
      break;
    }

    scale = ReadReg(MPUREG_GYRO_CONFIG);
    switch (scale){
        case BITS_FS_250DPS:
            return 250;
        break;
        case BITS_FS_500DPS:
            return 500;
        break;
        case BITS_FS_1000DPS:
            return 1000;
        break;
        case BITS_FS_2000DPS:
            return 2000;
        break;
      }

      return 0;
}
/*-------------------------------------------------------------------*/
void MPU9250::calib_mag(void){
  uint8_t response[3];
  float data;
  int i;

  WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
  WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX); //I2C slave 0 register address from where to begin data transfer
  WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83); //Read 3 bytes from the magnetometer

  ReadRegs(MPUREG_EXT_SENS_DATA_00,response,3);

  for(i=0; i<3; i++) {
      data=response[i];
      magnetometer_ASA[i]=((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
    }
}
/*-------------------------------------------------------------------*/
void MPU9250::read_acc(void){
  uint8_t response[6];
  int16_t bit_data;
  float data;
  int i;
  ReadRegs(MPUREG_ACCEL_XOUT_H,response,6);
  for(i=0; i<3; i++){
    bit_data = ((int16_t)response[i*2] << 8) | response[i*2+1];
    data = (float)bit_data;
    accelerometer_data[i] = G_SI * data / acc_divider;
  }
}
/*-------------------------------------------------------------------*/
void MPU9250::read_gyro(void){
  uint8_t response[6];
  int16_t bit_data;
  float data;
  int i;
  ReadRegs(MPUREG_GYRO_XOUT_H,response,6);
  for(i=0; i<3; i++){
    bit_data = ((int16_t)response[i*2] << 8) | response[i*2+1];
    data = (float)bit_data;
    gyroscope_data[i] = (PI / 180) * data / gyro_divider;
  }
}
/*-------------------------------------------------------------------*/
void MPU9250::read_mag(void){
  uint8_t response[7];
  int16_t bit_data;
  float data;
  int i;

  WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
  WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
  WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 6 bytes from the magnetometer

  ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
  //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

  for(i=0; i<3; i++){
    bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
    data=(float)bit_data;
    magnetometer_data[i]=data*magnetometer_ASA[i];
  }
}
/*-------------------------------------------------------------------*/
void MPU9250::read_temp(void){
  uint8_t response[2];
  int16_t bit_data;
  float data;
  ReadRegs(MPUREG_TEMP_OUT_H,response,2);

  bit_data=((int16_t)response[0]<<8)|response[1];
  data=(float)bit_data;
  temperature=(data/340)+36.53;
}
/*-------------------------------------------------------------------*/
void MPU9250::read_all(void){
  uint8_t response[21];
  int16_t bit_data;
  float data;
  int i;

  //Send I2C command at first
  WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
  WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
  WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 7 bytes from the magnetometer
  //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

  ReadRegs(MPUREG_ACCEL_XOUT_H,response,21);
  //Get accelerometer value
  for(i=0; i<3; i++) {
      bit_data = ((int16_t)response[i*2] << 8)|response[i*2+1];
      data = (float)bit_data;
      accelerometer_data[i] = G_SI * data / acc_divider;
  }
  //Get temperature
  bit_data = ((int16_t)response[i*2] << 8) | response[i*2+1];
  data = (float)bit_data;
  temperature = ((data - 21) / 333.87) + 21;
  //Get gyroscope value
  for(i=4; i<7; i++) {
      bit_data = ((int16_t)response[i*2] << 8) | response[i*2+1];
      data = (float)bit_data;
      gyroscope_data[i-4] = (PI / 180) * data / gyro_divider;
  }
  //Get Magnetometer value
  for(i=7; i<10; i++) {
      bit_data = ((int16_t)response[i*2+1] << 8) | response[i*2];
      data = (float)bit_data;
      magnetometer_data[i-7] = data * magnetometer_ASA[i-7];
    }
}
