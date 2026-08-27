#ifndef __IMU_UART_H__
#define __IMU_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* Serial protocol used by the new nine-axis IMU. */
#define IMU_UART_FRAME_HEAD1       0x7EU
#define IMU_UART_FRAME_HEAD2       0x23U
#define IMU_UART_FUNC_VERSION      0x01U
#define IMU_UART_FUNC_RAW_ACCEL    0x04U
#define IMU_UART_FUNC_RAW_GYRO     0x0AU
#define IMU_UART_FUNC_RAW_MAG      0x10U
#define IMU_UART_FUNC_QUATERNION   0x16U
#define IMU_UART_FUNC_EULER        0x26U
#define IMU_UART_FUNC_BAROMETER    0x32U
#define IMU_UART_FUNC_REQUEST      0x80U
#define IMU_UART_FUNC_RETURN_STATE 0x81U

typedef struct
{
  float accel[3];       /* g */
  float gyro[3];        /* rad/s */
  float mag[3];         /* uT */
  float quaternion[4];  /* w, x, y, z */
  float euler[3];       /* roll, pitch, yaw; degree */
  float barometer[4];   /* height, temperature, pressure, pressure delta */
  uint8_t version[3];
} IMU_UART_Data_t;

/* Compatibility-facing attitude values consumed by the vehicle control. */
extern float imu_yaw;
extern float imu_pitch;
extern float imu_roll;

HAL_StatusTypeDef IMU_UART_Init(void);
HAL_StatusTypeDef IMU_UART_RequestVersion(void);
void IMU_UART_Process(uint32_t now_ms);
void IMU_UART_HandleRxComplete(UART_HandleTypeDef *huart);
void IMU_UART_HandleError(UART_HandleTypeDef *huart);

uint8_t IMU_UART_IsReady(void);
uint32_t IMU_UART_GetLastEulerTick(void);
void IMU_UART_GetAll(IMU_UART_Data_t *out);

HAL_StatusTypeDef IMU_UART_SendCommand(uint8_t function,
                                       const uint8_t *params,
                                       uint8_t param_len);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_UART_H__ */
