/* I2C2 interface for MPU6500: PB10=SCL, PB11=SDA. */
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern I2C_HandleTypeDef hi2c2;
void MX_I2C2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
