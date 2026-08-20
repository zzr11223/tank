#include "spi.h"

#define SPI2_TRANSACTION_TIMEOUT_MS  2U

static void SPI2_ClearRxFlags(void)
{
  volatile uint16_t dummy = 0U;

  if ((SPI2->SR & SPI_SR_RXNE) != 0U)
  {
    dummy = SPI2->DR;
  }
  if ((SPI2->SR & SPI_SR_OVR) != 0U)
  {
    dummy = SPI2->DR;
    dummy = SPI2->SR;
  }
  (void)dummy;
}

static HAL_StatusTypeDef SPI2_TransferByte(uint8_t tx, uint8_t *rx)
{
  uint32_t start;

  if (rx == 0) return HAL_ERROR;

  start = HAL_GetTick();
  while ((SPI2->SR & SPI_SR_TXE) == 0U)
  {
    if ((HAL_GetTick() - start) > SPI2_TRANSACTION_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }

  *(__IO uint8_t *)&SPI2->DR = tx;
  start = HAL_GetTick();
  while ((SPI2->SR & SPI_SR_RXNE) == 0U)
  {
    if ((HAL_GetTick() - start) > SPI2_TRANSACTION_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }

  *rx = *(__IO uint8_t *)&SPI2->DR;
  return HAL_OK;
}

static HAL_StatusTypeDef SPI2_EndTransaction(void)
{
  uint32_t start = HAL_GetTick();

  while ((SPI2->SR & SPI_SR_BSY) != 0U)
  {
    if ((HAL_GetTick() - start) > SPI2_TRANSACTION_TIMEOUT_MS)
    {
      HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
      return HAL_TIMEOUT;
    }
  }
  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
  return HAL_OK;
}

void MX_SPI2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_SPI2_FORCE_RESET();
  __HAL_RCC_SPI2_RELEASE_RESET();

  /* PB13=SCK and PB15=MOSI are push-pull outputs; PB14 is MISO input. */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MPU6500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MPU6500_CS_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);

  /* SPI2 clock is 36 MHz on this project; BR=64 gives approximately 562 kHz.
     MPU6500 uses SPI mode 0 (CPOL=0, CPHA=0), MSB first, 8-bit frames. */
  SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
              (SPI_CR1_BR_2 | SPI_CR1_BR_1);
  SPI2->CR2 = 0U;
  SPI2->CR1 |= SPI_CR1_SPE;
}

HAL_StatusTypeDef SPI2_ReadRegisters(uint8_t reg, uint8_t *data,
                                     uint16_t length)
{
  uint16_t i;
  uint8_t discard;
  HAL_StatusTypeDef status;

  if ((data == 0) || (length == 0U)) return HAL_ERROR;

  SPI2_ClearRxFlags();
  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_RESET);
  status = SPI2_TransferByte((uint8_t)(reg | 0x80U), &discard);
  if (status != HAL_OK)
  {
    (void)SPI2_EndTransaction();
    return status;
  }

  for (i = 0U; i < length; ++i)
  {
    status = SPI2_TransferByte(0xFFU, &data[i]);
    if (status != HAL_OK)
    {
      (void)SPI2_EndTransaction();
      return status;
    }
  }
  return SPI2_EndTransaction();
}

HAL_StatusTypeDef SPI2_WriteRegister(uint8_t reg, uint8_t value)
{
  uint8_t discard;
  HAL_StatusTypeDef status;

  SPI2_ClearRxFlags();
  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_RESET);
  status = SPI2_TransferByte((uint8_t)(reg & 0x7FU), &discard);
  if (status == HAL_OK)
  {
    status = SPI2_TransferByte(value, &discard);
  }
  if (SPI2_EndTransaction() != HAL_OK) status = HAL_TIMEOUT;
  return status;
}
