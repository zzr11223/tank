/* SPI2 interface for MPU6500: PB13=SCK, PB14=MISO, PB15=MOSI, PB12=CS. */
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define MPU6500_CS_GPIO_Port GPIOB
#define MPU6500_CS_Pin       GPIO_PIN_12

void MX_SPI2_Init(void);
HAL_StatusTypeDef SPI2_ReadRegisters(uint8_t reg, uint8_t *data,
                                     uint16_t length);
HAL_StatusTypeDef SPI2_WriteRegister(uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
