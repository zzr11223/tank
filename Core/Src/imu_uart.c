#include "imu_uart.h"

#include <string.h>

#include "usart.h"

#define IMU_UART_RX_BUFFER_SIZE       256U
#define IMU_UART_FRAME_BODY_MAX        64U
#define IMU_UART_RAD_TO_DEG      57.2957795f

#if ((IMU_UART_RX_BUFFER_SIZE & (IMU_UART_RX_BUFFER_SIZE - 1U)) != 0U)
#error "IMU_UART_RX_BUFFER_SIZE must be a power of two"
#endif

enum
{
  IMU_RX_WAIT_HEAD1 = 0,
  IMU_RX_WAIT_HEAD2,
  IMU_RX_WAIT_LENGTH,
  IMU_RX_WAIT_FUNCTION,
  IMU_RX_COLLECT_BODY
};

float imu_yaw = 0.0f;
float imu_pitch = 0.0f;
float imu_roll = 0.0f;
uint32_t imu_attitude_last_tick = 0U;
uint8_t imu_attitude_valid = 0U;

static uint8_t imu_uart_rx_byte;
static volatile uint8_t imu_rx_buffer[IMU_UART_RX_BUFFER_SIZE];
static volatile uint16_t imu_rx_head;
static volatile uint16_t imu_rx_tail;

static uint8_t parser_state;
static uint8_t parser_frame_length;
static uint8_t parser_function;
static uint8_t parser_body[IMU_UART_FRAME_BODY_MAX];
static uint16_t parser_body_index;

static uint16_t IMU_UART_NextIndex(uint16_t index)
{
  return (uint16_t)((index + 1U) & (IMU_UART_RX_BUFFER_SIZE - 1U));
}

static void IMU_UART_PushFromIsr(uint8_t byte)
{
  uint16_t next = IMU_UART_NextIndex(imu_rx_head);

  if (next == imu_rx_tail)
  {
    /* Match the public reference driver's policy: keep the newest byte and
       discard the oldest one when the ring buffer is full.  This is safer
       for a continuous auto-report stream because a stale partial frame is
       less useful than the beginning of the next frame. */
    imu_rx_tail = IMU_UART_NextIndex(imu_rx_tail);
  }

  imu_rx_buffer[imu_rx_head] = byte;
  imu_rx_head = next;
}

static uint8_t IMU_UART_Pop(uint8_t *byte)
{
  if (imu_rx_tail == imu_rx_head)
  {
    return 0U;
  }

  *byte = imu_rx_buffer[imu_rx_tail];
  imu_rx_tail = IMU_UART_NextIndex(imu_rx_tail);
  return 1U;
}

static float IMU_UART_ReadFloatLE(const uint8_t *bytes)
{
  float value;
  memcpy(&value, bytes, sizeof(value));
  return value;
}

static uint8_t IMU_UART_FloatIsFinite(float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return ((bits & 0x7F800000UL) != 0x7F800000UL) ? 1U : 0U;
}

static void IMU_UART_ParseFrame(uint8_t function,
                                const uint8_t *payload,
                                uint16_t payload_length)
{
  if ((function == IMU_UART_FUNC_EULER) && (payload_length >= 12U))
  {
    float roll = IMU_UART_ReadFloatLE(&payload[0]);
    float pitch = IMU_UART_ReadFloatLE(&payload[4]);
    float yaw = IMU_UART_ReadFloatLE(&payload[8]);

    if (IMU_UART_FloatIsFinite(roll) &&
        IMU_UART_FloatIsFinite(pitch) &&
        IMU_UART_FloatIsFinite(yaw))
    {
      /* Function 0x26 contains little-endian Roll, Pitch and Yaw floats
         in radians. Keep the sensor values uncorrected for diagnostics. */
      imu_roll = roll * IMU_UART_RAD_TO_DEG;
      imu_pitch = pitch * IMU_UART_RAD_TO_DEG;
      imu_yaw = yaw * IMU_UART_RAD_TO_DEG;
      imu_attitude_last_tick = HAL_GetTick();
      imu_attitude_valid = 1U;
    }
  }
}

HAL_StatusTypeDef IMU_UART_Init(void)
{
  imu_yaw = 0.0f;
  imu_pitch = 0.0f;
  imu_roll = 0.0f;
  imu_attitude_last_tick = 0U;
  imu_attitude_valid = 0U;
  imu_rx_head = 0U;
  imu_rx_tail = 0U;
  parser_state = IMU_RX_WAIT_HEAD1;
  parser_frame_length = 0U;
  parser_function = 0U;
  parser_body_index = 0U;
  return HAL_UART_Receive_IT(&huart2, &imu_uart_rx_byte, 1U);
}

void IMU_UART_HandleRxComplete(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != USART2))
  {
    return;
  }

  IMU_UART_PushFromIsr(imu_uart_rx_byte);
  (void)HAL_UART_Receive_IT(&huart2, &imu_uart_rx_byte, 1U);
}

void IMU_UART_HandleError(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != USART2))
  {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(huart);
  (void)HAL_UART_Receive_IT(&huart2, &imu_uart_rx_byte, 1U);
}

void IMU_UART_Process(void)
{
  uint8_t byte;

  while (IMU_UART_Pop(&byte))
  {
    switch (parser_state)
    {
      case IMU_RX_WAIT_HEAD1:
        if (byte == IMU_UART_FRAME_HEAD1)
        {
          parser_state = IMU_RX_WAIT_HEAD2;
        }
        break;

      case IMU_RX_WAIT_HEAD2:
        if (byte == IMU_UART_FRAME_HEAD2)
        {
          parser_state = IMU_RX_WAIT_LENGTH;
        }
        else
        {
          parser_state = (byte == IMU_UART_FRAME_HEAD1) ?
                         IMU_RX_WAIT_HEAD2 : IMU_RX_WAIT_HEAD1;
        }
        break;

      case IMU_RX_WAIT_LENGTH:
        parser_frame_length = byte;
        if ((parser_frame_length < 5U) ||
            ((uint16_t)(parser_frame_length - 4U) > IMU_UART_FRAME_BODY_MAX))
        {
          parser_state = IMU_RX_WAIT_HEAD1;
        }
        else
        {
          parser_state = IMU_RX_WAIT_FUNCTION;
        }
        break;

      case IMU_RX_WAIT_FUNCTION:
        parser_function = byte;
        parser_body_index = 0U;
        parser_state = IMU_RX_COLLECT_BODY;
        break;

      case IMU_RX_COLLECT_BODY:
      {
        uint16_t body_length = (uint16_t)(parser_frame_length - 4U);
        parser_body[parser_body_index++] = byte;

        if (parser_body_index >= body_length)
        {
          uint8_t checksum = (uint8_t)(IMU_UART_FRAME_HEAD1 +
                                       IMU_UART_FRAME_HEAD2 +
                                       parser_frame_length +
                                       parser_function);
          uint16_t i;

          for (i = 0U; i < (body_length - 1U); ++i)
          {
            checksum = (uint8_t)(checksum + parser_body[i]);
          }

          if (checksum == parser_body[body_length - 1U])
          {
            IMU_UART_ParseFrame(parser_function,
                                parser_body,
                                body_length - 1U);
          }
          parser_state = IMU_RX_WAIT_HEAD1;
        }
        break;
      }

      default:
        parser_state = IMU_RX_WAIT_HEAD1;
        break;
    }
  }

}
