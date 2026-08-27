#include "imu_uart.h"

#include <string.h>

#include "usart.h"

#define IMU_UART_RX_BUFFER_SIZE       256U
#define IMU_UART_FRAME_BODY_MAX        64U
#define IMU_UART_EULER_TIMEOUT_MS     500U
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

static IMU_UART_Data_t imu_data;
static uint8_t imu_uart_rx_byte;
static volatile uint8_t imu_rx_buffer[IMU_UART_RX_BUFFER_SIZE];
static volatile uint16_t imu_rx_head;
static volatile uint16_t imu_rx_tail;

static uint8_t parser_state;
static uint8_t parser_frame_length;
static uint8_t parser_function;
static uint8_t parser_body[IMU_UART_FRAME_BODY_MAX];
static uint16_t parser_body_index;

static uint8_t imu_euler_valid;
static uint32_t imu_last_euler_tick;

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

static int16_t IMU_UART_ReadInt16LE(const uint8_t *bytes)
{
  uint16_t value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
  return (int16_t)value;
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
                                uint16_t payload_length,
                                uint32_t now_ms)
{
  switch (function)
  {
    case IMU_UART_FUNC_RAW_ACCEL:
      if (payload_length >= 18U)
      {
        const float accel_ratio = 16.0f / 32767.0f;
        const float gyro_ratio = (2000.0f / 32767.0f) / IMU_UART_RAD_TO_DEG;
        const float mag_ratio = 800.0f / 32767.0f;

        imu_data.accel[0] = (float)IMU_UART_ReadInt16LE(&payload[0]) * accel_ratio;
        imu_data.accel[1] = (float)IMU_UART_ReadInt16LE(&payload[2]) * accel_ratio;
        imu_data.accel[2] = (float)IMU_UART_ReadInt16LE(&payload[4]) * accel_ratio;
        imu_data.gyro[0] = (float)IMU_UART_ReadInt16LE(&payload[6]) * gyro_ratio;
        imu_data.gyro[1] = (float)IMU_UART_ReadInt16LE(&payload[8]) * gyro_ratio;
        imu_data.gyro[2] = (float)IMU_UART_ReadInt16LE(&payload[10]) * gyro_ratio;
        imu_data.mag[0] = (float)IMU_UART_ReadInt16LE(&payload[12]) * mag_ratio;
        imu_data.mag[1] = (float)IMU_UART_ReadInt16LE(&payload[14]) * mag_ratio;
        imu_data.mag[2] = (float)IMU_UART_ReadInt16LE(&payload[16]) * mag_ratio;
      }
      break;

    case IMU_UART_FUNC_RAW_GYRO:
      if (payload_length >= 6U)
      {
        const float gyro_ratio = (2000.0f / 32767.0f) / IMU_UART_RAD_TO_DEG;
        imu_data.gyro[0] = (float)IMU_UART_ReadInt16LE(&payload[0]) * gyro_ratio;
        imu_data.gyro[1] = (float)IMU_UART_ReadInt16LE(&payload[2]) * gyro_ratio;
        imu_data.gyro[2] = (float)IMU_UART_ReadInt16LE(&payload[4]) * gyro_ratio;
      }
      break;

    case IMU_UART_FUNC_RAW_MAG:
      if (payload_length >= 6U)
      {
        const float mag_ratio = 800.0f / 32767.0f;
        imu_data.mag[0] = (float)IMU_UART_ReadInt16LE(&payload[0]) * mag_ratio;
        imu_data.mag[1] = (float)IMU_UART_ReadInt16LE(&payload[2]) * mag_ratio;
        imu_data.mag[2] = (float)IMU_UART_ReadInt16LE(&payload[4]) * mag_ratio;
      }
      break;

    case IMU_UART_FUNC_QUATERNION:
      if (payload_length >= 16U)
      {
        uint8_t i;
        for (i = 0U; i < 4U; ++i)
        {
          imu_data.quaternion[i] = IMU_UART_ReadFloatLE(&payload[i * 4U]);
        }
      }
      break;

    case IMU_UART_FUNC_EULER:
      if (payload_length >= 12U)
      {
        float roll = IMU_UART_ReadFloatLE(&payload[0]);
        float pitch = IMU_UART_ReadFloatLE(&payload[4]);
        float yaw = IMU_UART_ReadFloatLE(&payload[8]);

        if (IMU_UART_FloatIsFinite(roll) &&
            IMU_UART_FloatIsFinite(pitch) &&
            IMU_UART_FloatIsFinite(yaw))
        {
          /* Reference driver: function 0x26 is three little-endian floats
             in radians, ordered Roll, Pitch, Yaw.  Do not change the IMU
             algorithm or apply a software Yaw offset here. */
          imu_roll = roll * IMU_UART_RAD_TO_DEG;
          imu_pitch = pitch * IMU_UART_RAD_TO_DEG;
          imu_yaw = yaw * IMU_UART_RAD_TO_DEG;
          imu_data.euler[0] = imu_roll;
          imu_data.euler[1] = imu_pitch;
          imu_data.euler[2] = imu_yaw;
          imu_last_euler_tick = now_ms;
          imu_euler_valid = 1U;
        }
      }
      break;

    case IMU_UART_FUNC_BAROMETER:
      if (payload_length >= 16U)
      {
        uint8_t i;
        for (i = 0U; i < 4U; ++i)
        {
          imu_data.barometer[i] = IMU_UART_ReadFloatLE(&payload[i * 4U]);
        }
      }
      break;

    case IMU_UART_FUNC_VERSION:
      if (payload_length >= 3U)
      {
        imu_data.version[0] = payload[0];
        imu_data.version[1] = payload[1];
        imu_data.version[2] = payload[2];
      }
      break;

    default:
      break;
  }

}

HAL_StatusTypeDef IMU_UART_Init(void)
{
  memset(&imu_data, 0, sizeof(imu_data));
  imu_yaw = 0.0f;
  imu_pitch = 0.0f;
  imu_roll = 0.0f;
  imu_rx_head = 0U;
  imu_rx_tail = 0U;
  parser_state = IMU_RX_WAIT_HEAD1;
  parser_frame_length = 0U;
  parser_function = 0U;
  parser_body_index = 0U;
  imu_euler_valid = 0U;
  imu_last_euler_tick = 0U;

  return HAL_UART_Receive_IT(&huart2, &imu_uart_rx_byte, 1U);
}

HAL_StatusTypeDef IMU_UART_RequestVersion(void)
{
  const uint8_t request[2] = {IMU_UART_FUNC_VERSION, 0x00U};

  /* Same startup request as the supplied STM32F103C8T6 reference. */
  return IMU_UART_SendCommand(IMU_UART_FUNC_REQUEST,
                              request,
                              (uint8_t)sizeof(request));
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

void IMU_UART_Process(uint32_t now_ms)
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
                                body_length - 1U,
                                now_ms);
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

uint8_t IMU_UART_IsReady(void)
{
  return (imu_euler_valid &&
          ((HAL_GetTick() - imu_last_euler_tick) <= IMU_UART_EULER_TIMEOUT_MS)) ? 1U : 0U;
}

uint32_t IMU_UART_GetLastEulerTick(void)
{
  return imu_last_euler_tick;
}

void IMU_UART_GetAll(IMU_UART_Data_t *out)
{
  if (out != NULL)
  {
    *out = imu_data;
  }
}

HAL_StatusTypeDef IMU_UART_SendCommand(uint8_t function,
                                       const uint8_t *params,
                                       uint8_t param_len)
{
  uint8_t frame[8] = {IMU_UART_FRAME_HEAD1, IMU_UART_FRAME_HEAD2,
                      0U, 0U, 0U, 0U, 0U, 0U};
  uint8_t frame_length;
  uint8_t checksum = 0U;
  uint8_t i;

  if ((param_len > 3U) || ((param_len > 0U) && (params == NULL)))
  {
    return HAL_ERROR;
  }

  frame_length = (uint8_t)(5U + param_len);
  frame[2] = frame_length;
  frame[3] = function;
  for (i = 0U; i < param_len; ++i)
  {
    frame[4U + i] = params[i];
  }
  for (i = 0U; i < (frame_length - 1U); ++i)
  {
    checksum = (uint8_t)(checksum + frame[i]);
  }
  frame[frame_length - 1U] = checksum;

  return HAL_UART_Transmit(&huart2, frame, frame_length, 20U);
}
