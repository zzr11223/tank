#ifndef __IMU_UART_H__
#define __IMU_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* Serial protocol used by the new nine-axis IMU. */
#define IMU_UART_FRAME_HEAD1       0x7EU
#define IMU_UART_FRAME_HEAD2       0x23U
#define IMU_UART_FUNC_EULER        0x26U

/* Euler angles used only by USART3 attitude telemetry, not vehicle control. */
extern float imu_yaw;
extern float imu_pitch;
extern float imu_roll;

HAL_StatusTypeDef IMU_UART_Init(void);
void IMU_UART_Process(void);
void IMU_UART_HandleRxComplete(UART_HandleTypeDef *huart);
void IMU_UART_HandleError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_UART_H__ */
