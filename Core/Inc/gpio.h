/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* 四路数字红外避障/测距模块，OUT 低电平表示达到模块设定距离阈值。
 * PC4/PC5/PC8/PC9 分别使用 EXTI4/5/8/9，不与 RC、编码器或 SPI2 冲突。 */
#define IR1_GPIO_Port         GPIOC
#define IR1_Pin               GPIO_PIN_4
#define IR2_GPIO_Port         GPIOC
#define IR2_Pin               GPIO_PIN_5
#define IR3_GPIO_Port         GPIOC
#define IR3_Pin               GPIO_PIN_8
#define IR4_GPIO_Port         GPIOC
#define IR4_Pin               GPIO_PIN_9

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

