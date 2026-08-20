#include "i2c.h"

I2C_HandleTypeDef hi2c2;

/*
 * Release an I2C slave that was interrupted while holding SDA low.  This is
 * done with the peripheral disabled, so it also works before the first
 * HAL_I2C_Init() and after a failed reinitialization.
 */
static void I2C2_BusRecovery(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t pulse;
  uint32_t delay;

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Both lines idle high; then provide nine clock edges for a stuck slave. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
  for (pulse = 0U; pulse < 9U; ++pulse)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    for (delay = 0U; delay < 250U; ++delay) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    for (delay = 0U; delay < 250U; ++delay) { __NOP(); }
  }

  /* Generate a STOP: SDA low, SCL high, SDA high. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
  for (delay = 0U; delay < 250U; ++delay) { __NOP(); }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  for (delay = 0U; delay < 250U; ++delay) { __NOP(); }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
}

void MX_I2C2_Init(void)
{
  /* HAL_I2C_DeInit() does not always clear a BUSY/START state left by an
   * aborted STM32F1 transaction.  Force-reset the peripheral before every
   * first init and re-init so WRONG_START cannot carry into the next try. */
  __HAL_RCC_I2C2_CLK_ENABLE();
  __HAL_RCC_I2C2_FORCE_RESET();
  __HAL_RCC_I2C2_RELEASE_RESET();

  hi2c2.Instance = I2C2;
  /* 50 kHz sacrifices throughput for extra edge/noise margin while the
   * motors and ESC wiring are active. */
  hi2c2.Init.ClockSpeed = 50000U;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0U;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0U;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  I2C2_BusRecovery();
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (i2cHandle->Instance == I2C2)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    /* I2C2: PB10=SCL, PB11=SDA. Both lines need external 3.3 V pull-ups. */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2cHandle)
{
  if (i2cHandle->Instance == I2C2)
  {
    __HAL_RCC_I2C2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
  }
}
