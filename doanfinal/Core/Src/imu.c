#include "imu.h"
#include "flash.h"

#include "MPU6050.h"
#include "stdio.h"

#define true 1
#define false 0	

#ifndef NULL
#define NULL 0
#endif

#define OK 6U

MPU6050 mpu;
I2Cdev i2c;
IMU imu;
#define page_64_startAdress 0x08020000
#define page_65_startAdress 0x08000000

extern UART_HandleTypeDef huart1;
void imu_init(void)
{
  mpu.reset();
  HAL_Delay(30);
  mpu.initialize();
  HAL_Delay(30);
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);
  mpu.setStandbyXAccelEnabled(false);
  mpu.setStandbyYAccelEnabled(false);
  mpu.setStandbyXGyroEnabled(false);
  mpu.setStandbyYGyroEnabled(false);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  imu.read_offset=(int)Flash_Read(page_64_startAdress);
}
void imu_calib(void)
{ 
  mpu.setZGyroOffset(0);
  imu.read_offset = 0;
  while(1)
  {
    HAL_Delay(30);
    imu.popular_gz_value += imu.gz;
    imu.sample_count++;
    if(imu.sample_count > imu.sample_count_max)
    {
      imu.offset_value = imu.popular_gz_value/imu.sample_count_max;
      imu.sample_count = 0;
      imu.popular_gz_value =0;
      imu.offset_value = (int16_t)(imu.offset_value);
      Flash_Write(page_64_startAdress,(uint16_t)imu.offset_value); 
      imu.read_offset =   (int)Flash_Read(page_64_startAdress);     
      break;
    }
  }
}
void imu_reset(void)
{
  imu.integral=0;
}

void imu_return(void)
{
  imu.gz=mpu.getRotationZ() - imu.read_offset;
  if(abs(imu.gz) > 1) imu.integral += (imu.gz/gyro_ratio)*dt;
}    
