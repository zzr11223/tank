#ifndef __MPU6500_H__
#define __MPU6500_H__

#include "main.h"

/* MPU6500 is accessed through SPI2; the I2C AD0 address is not used. */

typedef struct
{
  float accel_x, accel_y, accel_z; /* g */
  float gyro_x, gyro_y, gyro_z;    /* degree/s */
} MPU6500_Data_t;

extern MPU6500_Data_t mpu6500_data;
extern float mpu6500_yaw;   /* relative yaw, degree */
extern float mpu6500_pitch; /* degree */
extern float mpu6500_roll;  /* degree */

uint8_t MPU6500_Init(void);
void MPU6500_Calibrate(uint16_t samples);
uint8_t MPU6500_ReadData(MPU6500_Data_t *data);
uint8_t MPU6500_Update(uint32_t now_ms);
void MPU6500_SetStationary(uint8_t stationary);
uint8_t MPU6500_IsReady(void);
uint8_t MPU6500_GetReadFailures(void);
uint8_t MPU6500_GetInitFailure(void);
uint8_t MPU6500_GetLastWhoAmI(void);
/* Legacy function name retained; it now returns the latest SPI status. */
uint32_t MPU6500_GetLastI2cError(void);

#endif /* __MPU6500_H__ */
