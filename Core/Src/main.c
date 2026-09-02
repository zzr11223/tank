/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "imu_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ----- 原有 6 通道 PWM 输入：引脚不变，改为 GPIO EXTI + TIM5 计时 ----- */
#define RC_INPUT_MEDIAN_SAMPLES      7U  /* 必须为奇数 */

typedef struct
{
  uint32_t samples[RC_INPUT_MEDIAN_SAMPLES];
  uint8_t next_index;
  uint8_t initialized;
} RcMedianFilter;

/* CH1: PA6 */
volatile uint32_t CH1_PulseWidth = 0;
volatile uint32_t CH1_LastRise   = 0;
volatile uint8_t  CH1_State      = 0;
volatile uint32_t CH1_LastTick   = 0;
static RcMedianFilter CH1_InputFilter = {0};

/* CH2: PA2 */
volatile uint32_t CH2_PulseWidth = 0;
volatile uint32_t CH2_LastRise   = 0;
volatile uint8_t  CH2_State      = 0;
volatile uint32_t CH2_LastTick   = 0;
static RcMedianFilter CH2_InputFilter = {0};

/* CH3: PA3 */
volatile uint32_t CH3_PulseWidth = 0;
volatile uint32_t CH3_LastRise   = 0;
volatile uint8_t  CH3_State      = 0;
volatile uint32_t CH3_LastTick   = 0;
static RcMedianFilter CH3_InputFilter = {0};

/* CH4: PA7 */
volatile uint32_t CH4_PulseWidth = 0;
volatile uint32_t CH4_LastRise   = 0;
volatile uint8_t  CH4_State      = 0;
volatile uint32_t CH4_LastTick   = 0;
static RcMedianFilter CH4_InputFilter = {0};

/* CH5: PB0 */
volatile uint32_t CH5_PulseWidth = 0;
volatile uint32_t CH5_LastRise   = 0;
volatile uint8_t  CH5_State      = 0;
volatile uint32_t CH5_LastTick   = 0;
static RcMedianFilter CH5_InputFilter = {0};

/* CH6: PB1 */
volatile uint32_t CH6_PulseWidth = 0;
volatile uint32_t CH6_LastRise   = 0;
volatile uint8_t  CH6_State      = 0;
volatile uint32_t CH6_LastTick   = 0;
static RcMedianFilter CH6_InputFilter = {0};

/* 四路数字红外：OUT 低电平=达到设定距离阈值，任一路触发即紧急停车。 */
volatile uint8_t IR1_Detected = 0U;
volatile uint8_t IR2_Detected = 0U;
volatile uint8_t IR3_Detected = 0U;
volatile uint8_t IR4_Detected = 0U;
volatile uint8_t EmergencyStopActive = 0U;
volatile uint8_t EmergencyStopLatched = 0U;
static volatile uint8_t TrackPwmReady = 0U;

/* 四路 AB 编码器累计计数，正负方向由定时器硬件决定。 */
volatile int32_t Encoder1_Count = 0;
volatile int32_t Encoder2_Count = 0;
volatile int32_t Encoder3_Count = 0;
volatile int32_t Encoder4_Count = 0;
static uint16_t Encoder1_LastCounter = 0;
static uint16_t Encoder2_LastCounter = 0;
static uint16_t Encoder3_LastCounter = 0;
static uint16_t Encoder4_LastCounter = 0;

/* 最终写入 TIM1 CCR 的左右履带 PWM 脉宽，单位 us。 */
volatile uint16_t RightTrackPwmUs = 1500;
volatile uint16_t LeftTrackPwmUs = 1500;

/* 四路编码器用于累计计数遥测及直行时左右履带速度同步闭环。 */
static int32_t SpeedPrevEncoder1 = 0;
static int32_t SpeedPrevEncoder2 = 0;
static int32_t SpeedPrevEncoder3 = 0;
static int32_t SpeedPrevEncoder4 = 0;
static uint32_t SpeedLastTick = 0;
static int32_t LeftSpeedControlDeltaFiltered = 0;
static int32_t RightSpeedControlDeltaFiltered = 0;
static int32_t LeftTrackControlSpeedCps = 0;
static int32_t RightTrackControlSpeedCps = 0;

char dbg_buf[64];
uint32_t LastDebugTick = 0;
char attitude_buf[64];
uint32_t LastAttitudeUartTick = 0;

/*
 * Tank drive mapping
 *   CH3 / PA3 : forward/reverse throttle command
 *   CH1 / PA6 : right stick horizontal, steering command
 *   PE9       : TIM1_CH1, left track ESC / motor controller
 *   PE11      : TIM1_CH2, right track ESC / motor controller
 *
 * The motor controllers must accept bidirectional RC PWM: 1500 us is stop,
 * 1000 us is full reverse, and 2000 us is full forward.
 */
#define RC_OUTPUT_MIN_US          1000
#define RC_OUTPUT_CENTER_US       1500
#define RC_OUTPUT_MAX_US          2000
#define RC_INPUT_VALID_MIN_US     1000
#define RC_INPUT_VALID_MAX_US     2100
#define RC_CH1_DEADBAND_US          85
#define RC_CH3_DEADBAND_US          40
/* 遥控器输入使用 7 点中值滤波：剔除最多连续三帧尖峰，真实阶跃
 * 连续四帧后通过；没有 IIR 拖尾。
 * 本机接收机实际输出不会低于 1000 us，低于 1000 的读数一律丢弃。 */
#define DEBUG_TELEMETRY_INTERVAL_MS  100
#define ATTITUDE_UART_INTERVAL_MS      20

/* IMU heading hold: PI differential correction with verified sign. */
#define IMU_ATTITUDE_TIMEOUT_MS                   100U
#define YAW_HOLD_MIN_THROTTLE_US                    60
#define YAW_HOLD_KP_US_PER_DEG                      6.0f
#define YAW_HOLD_KI_US_PER_DEG_SECOND               2.0f
#define YAW_HOLD_ERROR_DEADBAND_DEG                  0.3f
#define YAW_HOLD_INTEGRAL_LIMIT_US                  24.0f
#define YAW_HOLD_MAX_CORRECTION_US                   60
static uint8_t ImuAttitudeFresh = 0U;
static uint8_t YawHoldActive = 0U;
static float YawHoldTargetDeg = 0.0f;
static float YawHoldErrorDeg = 0.0f;
static float YawHoldIntegralUs = 0.0f;
static int32_t YawHoldCorrectionUs = 0;
static uint32_t YawHoldLastUpdateTick = 0U;

/* 编码器速度：20 ms 采样，闭环使用 1/2 低通抑制量化噪声。 */
#define TRACK_SPEED_SAMPLE_INTERVAL_MS  20
#define TRACK_SPEED_CONTROL_FILTER_DIV    2

/*
 * 左右履带速度同步 PI。输出是“右履带速度幅值增加、左履带速度幅值减少”
 * 的 PWM 修正量；倒车时在混控处自动反转符号。这里控制左右速度差而不是
 * 绝对车速，因此不依赖油门到编码器 CPS 的非线性标定，也不会改变平均油门。
 */
#define TRACK_SPEED_CONTROL_MIN_THROTTLE_US       60
#define TRACK_SPEED_CONTROL_MIN_AVERAGE_CPS      300
#define TRACK_SPEED_CONTROL_DEADBAND_CPS          50
#define TRACK_SPEED_CONTROL_KP_US_PER_CPS     0.020f
#define TRACK_SPEED_CONTROL_KI_US_PER_SAMPLE  0.0015f
#define TRACK_SPEED_CONTROL_INTEGRAL_LIMIT_CPS 25000.0f
#define TRACK_SPEED_CONTROL_MAX_CORRECTION_US     60
static uint8_t TrackSpeedControlActive = 0U;
static float TrackSpeedControlIntegral = 0.0f;
static int32_t TrackSpeedCorrectionUs = 0;
static uint32_t TrackSpeedControlLastSampleTick = 0U;

/*
 * Two 3.87 m loaded runs were used to normalize each encoder to the same
 * physical-distance scale.  The reference scale is the average of ENC3/4.
 * Values are stored as factors multiplied by 1000 to avoid floating point.
 */
#define TRACK_ENCODER_SCALE               1000
#define ENC1_DISTANCE_SCALE               794
#define ENC2_DISTANCE_SCALE               869
#define ENC3_DISTANCE_SCALE              1013
#define ENC4_DISTANCE_SCALE               988
/* 实际同速样本的左/右计数总体比例约为 1.193；补偿右侧后再进入 PI。 */
#define TRACK_SPEED_REFERENCE_Q10        1024
#define TRACK_RS_REFERENCE_Q10           1222

/* Set either value to 1 only if the corresponding physical direction is reversed. */
#define TANK_THROTTLE_REVERSED       0
#define TANK_STEERING_REVERSED       0
#define TANK_RIGHT_TRACK_REVERSED    0
#define TANK_LEFT_TRACK_REVERSED     0
#define TANK_STEERING_PRIORITY       1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int32_t ClampI32(int32_t value, int32_t minimum, int32_t maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

/* 姿态角转整数百分之一度（工程无 %f 打印支持），如 12.34 度 → 1234。 */
static int32_t AngleToCentiDegrees(float angle)
{
  if (angle >= 0.0f)
  {
    return (int32_t)(angle * 100.0f + 0.5f);
  }
  return -(int32_t)((-angle) * 100.0f + 0.5f);
}

/* 串口显示用的 Yaw 范围：0.00 <= Yaw < 360.00 度。 */
static int32_t AngleToUnsignedCentiDegrees(float angle)
{
  int32_t centi_degrees;

  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  centi_degrees = (int32_t)(angle * 100.0f + 0.5f);
  return (centi_degrees >= 36000) ? 0 : centi_degrees;
}

/* Convert an RC pulse to a signed command around 0. Invalid/lost input stops. */
static int32_t RcPulseToCommand(uint32_t pulse_us, int32_t deadband_us)
{
  int32_t command;

  if ((pulse_us < RC_INPUT_VALID_MIN_US) || (pulse_us > RC_INPUT_VALID_MAX_US))
  {
    return 0;
  }

  command = (int32_t)pulse_us - RC_OUTPUT_CENTER_US;
  if ((command >= -deadband_us) && (command <= deadband_us))
  {
    return 0;
  }

  return ClampI32(command,
                  RC_OUTPUT_MIN_US - RC_OUTPUT_CENTER_US,
                  RC_OUTPUT_MAX_US - RC_OUTPUT_CENTER_US);
}

static uint16_t CommandToRcPulse(int32_t command)
{
  command = ClampI32(command,
                     RC_OUTPUT_MIN_US - RC_OUTPUT_CENTER_US,
                     RC_OUTPUT_MAX_US - RC_OUTPUT_CENTER_US);
  return (uint16_t)(RC_OUTPUT_CENTER_US + command);
}

/*
 * 所有运行时履带 PWM 都经此函数提交。短临界区保证红外中断与主循环
 * 不会交叉写两路 CCR；任一安全锁有效时，无论调用者请求什么都强制回中。
 */
static void CommitTrackPwm(uint16_t left_pwm, uint16_t right_pwm)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (EmergencyStopActive || EmergencyStopLatched)
  {
    left_pwm = RC_OUTPUT_CENTER_US;
    right_pwm = RC_OUTPUT_CENTER_US;
  }
  LeftTrackPwmUs = left_pwm;
  RightTrackPwmUs = right_pwm;
  if (TrackPwmReady)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_pwm);
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/* 主循环轮询作为 EXTI 的冗余保护，也能识别上电时已经为低电平的障碍。 */
static void RefreshInfraredInputs(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  IR1_Detected = (HAL_GPIO_ReadPin(IR1_GPIO_Port, IR1_Pin) == GPIO_PIN_RESET);
  IR2_Detected = (HAL_GPIO_ReadPin(IR2_GPIO_Port, IR2_Pin) == GPIO_PIN_RESET);
  IR3_Detected = (HAL_GPIO_ReadPin(IR3_GPIO_Port, IR3_Pin) == GPIO_PIN_RESET);
  IR4_Detected = (HAL_GPIO_ReadPin(IR4_GPIO_Port, IR4_Pin) == GPIO_PIN_RESET);
  EmergencyStopActive = (IR1_Detected != 0U) || (IR2_Detected != 0U) ||
                        (IR3_Detected != 0U) || (IR4_Detected != 0U);
  if (EmergencyStopActive)
  {
    EmergencyStopLatched = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
  if (EmergencyStopActive || EmergencyStopLatched)
  {
    CommitTrackPwm(RC_OUTPUT_CENTER_US, RC_OUTPUT_CENTER_US);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    IMU_UART_HandleRxComplete(huart);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    IMU_UART_HandleError(huart);
  }
}

static uint32_t MedianU32(const uint32_t samples[RC_INPUT_MEDIAN_SAMPLES])
{
  uint32_t sorted[RC_INPUT_MEDIAN_SAMPLES];
  uint32_t value;
  uint8_t i;
  uint8_t j;

  for (i = 0U; i < RC_INPUT_MEDIAN_SAMPLES; i++)
  {
    sorted[i] = samples[i];
  }

  for (i = 1U; i < RC_INPUT_MEDIAN_SAMPLES; i++)
  {
    value = sorted[i];
    j = i;
    while ((j > 0U) && (sorted[j - 1U] > value))
    {
      sorted[j] = sorted[j - 1U];
      j--;
    }
    sorted[j] = value;
  }

  return sorted[RC_INPUT_MEDIAN_SAMPLES / 2U];
}

static uint32_t FilterRcPulseMedian(RcMedianFilter *filter,
                                    uint32_t sample,
                                    uint8_t reset)
{
  uint8_t i;

  if ((filter->initialized == 0U) || (reset != 0U))
  {
    for (i = 0U; i < RC_INPUT_MEDIAN_SAMPLES; i++)
    {
      filter->samples[i] = sample;
    }
    filter->next_index = 0U;
    filter->initialized = 1U;
    return sample;
  }

  filter->samples[filter->next_index] = sample;
  filter->next_index++;
  if (filter->next_index >= RC_INPUT_MEDIAN_SAMPLES)
  {
    filter->next_index = 0U;
  }

  return MedianU32(filter->samples);
}

static void RcCaptureEdge(volatile uint32_t *pulse_width,
                           RcMedianFilter *input_filter,
                           volatile uint32_t *last_rise,
                           volatile uint8_t *state,
                           volatile uint32_t *last_tick,
                          GPIO_TypeDef *port,
                          uint16_t pin)
{
  uint32_t now = __HAL_TIM_GET_COUNTER(&htim5);
  uint32_t width;

  if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
  {
    *last_rise = now;
    *state = 1;
  }
  else if (*state != 0)
  {
    /* uint32_t subtraction naturally handles TIM5 wraparound. */
    width = now - *last_rise;
    if ((width >= RC_INPUT_VALID_MIN_US) &&
        (width <= RC_INPUT_VALID_MAX_US))
    {
      /* 任何合法帧都视为信号在线并刷新失联计时；失败保护只对
       * 真实丢帧生效。 */
      *last_tick = HAL_GetTick();

      /* 上电首帧或失败保护恢复后立即采纳；其余帧使用 7 点中值，
       * 最多连续三个异常脉宽不会进入控制量。 */
      *pulse_width = FilterRcPulseMedian(input_filter,
                                         width,
                                         (*pulse_width == 0U) ? 1U : 0U);
    }
    /* 低于 1000 us（本机实际不可能出现）或超过 2100 us 的读数直接丢弃。 */
    *state = 0;
  }
}

static void UpdateOneEncoder(TIM_HandleTypeDef *htim,
                             volatile int32_t *position,
                             uint16_t *last_counter)
{
  uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
  int16_t delta = (int16_t)(current - *last_counter);

  *position += delta;
  *last_counter = current;
}

static void UpdateEncoderCounters(void)
{
  UpdateOneEncoder(&htim2, &Encoder1_Count, &Encoder1_LastCounter);
  UpdateOneEncoder(&htim3, &Encoder2_Count, &Encoder2_LastCounter);
  UpdateOneEncoder(&htim4, &Encoder3_Count, &Encoder3_LastCounter);
  UpdateOneEncoder(&htim8, &Encoder4_Count, &Encoder4_LastCounter);
}

static int32_t AbsI32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static float NormalizeYawDegrees(float yaw)
{
  while (yaw < 0.0f) yaw += 360.0f;
  while (yaw >= 360.0f) yaw -= 360.0f;
  return yaw;
}

static float WrapYawErrorDegrees(float error)
{
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  return error;
}

static void ResetYawHold(void)
{
  YawHoldActive = 0U;
  YawHoldTargetDeg = 0.0f;
  YawHoldErrorDeg = 0.0f;
  YawHoldIntegralUs = 0.0f;
  YawHoldCorrectionUs = 0;
  YawHoldLastUpdateTick = 0U;
}

/*
 * Capture the current heading when straight driving starts, then calculate a
 * PI differential correction. Physical evidence confirmed that a right
 * deviation decreases Yaw, so positive error must speed up the right track
 * and slow down the left track to steer back left. The integral term learns
 * the static left/right PWM bias required to hold the requested heading.
 */
static void UpdateYawHold(uint32_t now)
{
  int32_t throttle = RcPulseToCommand(CH3_PulseWidth, RC_CH3_DEADBAND_US);
  int32_t steering = RcPulseToCommand(CH1_PulseWidth, RC_CH1_DEADBAND_US);
  uint32_t elapsed_ms;
  float proportional_us;
  float integral_delta_us;
  float raw_correction;

  ImuAttitudeFresh =
      ((imu_attitude_valid != 0U) &&
       ((now - imu_attitude_last_tick) <= IMU_ATTITUDE_TIMEOUT_MS)) ? 1U : 0U;

  if ((ImuAttitudeFresh == 0U) ||
      (EmergencyStopActive != 0U) ||
      (EmergencyStopLatched != 0U) ||
      (AbsI32(throttle) < YAW_HOLD_MIN_THROTTLE_US) ||
      (steering != 0))
  {
    ResetYawHold();
    return;
  }

  if (YawHoldActive == 0U)
  {
    YawHoldTargetDeg = NormalizeYawDegrees(imu_yaw);
    YawHoldIntegralUs = 0.0f;
    YawHoldLastUpdateTick = now;
    YawHoldActive = 1U;
    return;
  }

  elapsed_ms = now - YawHoldLastUpdateTick;
  YawHoldLastUpdateTick = now;

  YawHoldErrorDeg =
      WrapYawErrorDegrees(YawHoldTargetDeg -
                          NormalizeYawDegrees(imu_yaw));

  if ((YawHoldErrorDeg <= YAW_HOLD_ERROR_DEADBAND_DEG) &&
      (YawHoldErrorDeg >= -YAW_HOLD_ERROR_DEADBAND_DEG))
  {
    YawHoldErrorDeg = 0.0f;
  }

  proportional_us = YawHoldErrorDeg * YAW_HOLD_KP_US_PER_DEG;
  raw_correction = proportional_us + YawHoldIntegralUs;

  /*
   * Time-based integral removes the steady heading error left by P control.
   * Conditional integration prevents windup while the total output is
   * saturated, but still permits an opposite error to unwind the integral.
   */
  if (elapsed_ms > 0U)
  {
    integral_delta_us =
        YawHoldErrorDeg * YAW_HOLD_KI_US_PER_DEG_SECOND *
        ((float)elapsed_ms / 1000.0f);

    if (((raw_correction < (float)YAW_HOLD_MAX_CORRECTION_US) &&
         (raw_correction > -(float)YAW_HOLD_MAX_CORRECTION_US)) ||
        ((raw_correction >= (float)YAW_HOLD_MAX_CORRECTION_US) &&
         (integral_delta_us < 0.0f)) ||
        ((raw_correction <= -(float)YAW_HOLD_MAX_CORRECTION_US) &&
         (integral_delta_us > 0.0f)))
    {
      YawHoldIntegralUs += integral_delta_us;
      if (YawHoldIntegralUs > YAW_HOLD_INTEGRAL_LIMIT_US)
      {
        YawHoldIntegralUs = YAW_HOLD_INTEGRAL_LIMIT_US;
      }
      else if (YawHoldIntegralUs < -YAW_HOLD_INTEGRAL_LIMIT_US)
      {
        YawHoldIntegralUs = -YAW_HOLD_INTEGRAL_LIMIT_US;
      }
    }
  }

  raw_correction = proportional_us + YawHoldIntegralUs;
  if (raw_correction > (float)YAW_HOLD_MAX_CORRECTION_US)
  {
    raw_correction = (float)YAW_HOLD_MAX_CORRECTION_US;
  }
  else if (raw_correction < -(float)YAW_HOLD_MAX_CORRECTION_US)
  {
    raw_correction = -(float)YAW_HOLD_MAX_CORRECTION_US;
  }

  YawHoldCorrectionUs =
      (raw_correction >= 0.0f) ?
      (int32_t)(raw_correction + 0.5f) :
      (int32_t)(raw_correction - 0.5f);
}

static int32_t CalibrateEncoderDelta(int32_t delta, int32_t scale)
{
  return (AbsI32(delta) * scale) / TRACK_ENCODER_SCALE;
}

/*
 * Physical straight-run evidence: ENC1/ENC2 are the left-track encoders;
 * ENC3/ENC4 are the right-track encoders. Forward raw counts have opposite
 * signs between sides, so speed magnitude intentionally uses AbsI32().
 * 使用每个采样周期的脉冲增量而不是累计位置，避免累计值不同造成误判。
 */
static void UpdateTrackSpeedMeasurement(uint32_t now)
{
  uint32_t elapsed;
  int32_t delta1;
  int32_t delta2;
  int32_t delta3;
  int32_t delta4;
  int32_t left_delta;
  int32_t right_delta;
  if ((now - SpeedLastTick) < TRACK_SPEED_SAMPLE_INTERVAL_MS)
  {
    return;
  }

  elapsed = now - SpeedLastTick;
  SpeedLastTick = now;

  delta1 = Encoder1_Count - SpeedPrevEncoder1;
  delta2 = Encoder2_Count - SpeedPrevEncoder2;
  delta3 = Encoder3_Count - SpeedPrevEncoder3;
  delta4 = Encoder4_Count - SpeedPrevEncoder4;
  SpeedPrevEncoder1 = Encoder1_Count;
  SpeedPrevEncoder2 = Encoder2_Count;
  SpeedPrevEncoder3 = Encoder3_Count;
  SpeedPrevEncoder4 = Encoder4_Count;

  /*
   * Normalize each encoder before averaging.  Equal raw counts are not equal
   * physical distance here: the two 3.87 m calibration runs measured about
   * 22007.9/20111.4/17250.1/17690.6 counts per metre for ENC1..ENC4.
   */
  left_delta = (CalibrateEncoderDelta(delta1, ENC1_DISTANCE_SCALE) +
                CalibrateEncoderDelta(delta2, ENC2_DISTANCE_SCALE)) / 2;
  right_delta = (CalibrateEncoderDelta(delta3, ENC3_DISTANCE_SCALE) +
                 CalibrateEncoderDelta(delta4, ENC4_DISTANCE_SCALE)) / 2;

  /* 以实测实际同速时的左右计数比例修正右侧速度，再进入同步 PI。 */
  right_delta = (right_delta * TRACK_RS_REFERENCE_Q10) /
                TRACK_SPEED_REFERENCE_Q10;

  /*
   * 先把各周期增量归一化到固定采样周期基准，再做低通。
   * 直接滤波原始增量时，主循环偶发延迟（阻塞式串口发送等）会让 elapsed
   * 变成 60~100 ms，delta 随之翻倍而低通滞后，速度=滤波增量/elapsed
   * 会整段假跳变（实测 PWM 恒定而速度读数 8800→3638→9450 乱跳）。
   * 归一化后速度与滤波误差都不再随周期长度变化。
   */
  left_delta = (left_delta * (int32_t)TRACK_SPEED_SAMPLE_INTERVAL_MS) / (int32_t)elapsed;
  right_delta = (right_delta * (int32_t)TRACK_SPEED_SAMPLE_INTERVAL_MS) / (int32_t)elapsed;

  /* 闭环测速使用 1/2 低通，响应约 40 ms。 */
  LeftSpeedControlDeltaFiltered +=
      (left_delta - LeftSpeedControlDeltaFiltered) / TRACK_SPEED_CONTROL_FILTER_DIV;
  RightSpeedControlDeltaFiltered +=
      (right_delta - RightSpeedControlDeltaFiltered) / TRACK_SPEED_CONTROL_FILTER_DIV;
  LeftTrackControlSpeedCps =
      (LeftSpeedControlDeltaFiltered * 1000) / (int32_t)TRACK_SPEED_SAMPLE_INTERVAL_MS;
  RightTrackControlSpeedCps =
      (RightSpeedControlDeltaFiltered * 1000) / (int32_t)TRACK_SPEED_SAMPLE_INTERVAL_MS;
}

static void ResetTrackSpeedControl(void)
{
  TrackSpeedControlActive = 0U;
  TrackSpeedControlIntegral = 0.0f;
  TrackSpeedCorrectionUs = 0;
  TrackSpeedControlLastSampleTick = SpeedLastTick;
}

static uint8_t RcPulseIsValid(uint32_t pulse_us)
{
  return ((pulse_us >= RC_INPUT_VALID_MIN_US) &&
          (pulse_us <= RC_INPUT_VALID_MAX_US)) ? 1U : 0U;
}

static uint8_t RcControlsAreCentered(void)
{
  return ((RcPulseIsValid(CH1_PulseWidth) != 0U) &&
          (RcPulseIsValid(CH3_PulseWidth) != 0U) &&
          (RcPulseToCommand(CH1_PulseWidth, RC_CH1_DEADBAND_US) == 0) &&
          (RcPulseToCommand(CH3_PulseWidth, RC_CH3_DEADBAND_US) == 0)) ? 1U : 0U;
}

static void UpdateInfraredInterlock(void)
{
  if (EmergencyStopActive != 0U)
  {
    EmergencyStopLatched = 1U;
  }
  else if ((EmergencyStopLatched != 0U) &&
           (RcControlsAreCentered() != 0U))
  {
    /* 障碍解除后必须先把油门和转向都回中一次；随后重新推杆才运动。 */
    EmergencyStopLatched = 0U;
  }
}

/*
 * 直行时闭合左右速度差 PI 环。
 * error > 0 表示左履带更快，因此输出正修正：右侧加速、左侧减速。
 * 本阶段只根据编码器速度工作，不读取 IMU 状态或航向角。
 */
static int32_t UpdateTrackSpeedControl(int32_t throttle, int32_t steering)
{
  int32_t throttle_magnitude;
  int32_t average_speed;
  int32_t speed_error;
  float candidate_integral;
  float raw_correction;

  throttle_magnitude = AbsI32(throttle);
  if ((throttle_magnitude < TRACK_SPEED_CONTROL_MIN_THROTTLE_US) ||
      (steering != 0))
  {
    ResetTrackSpeedControl();
    return 0;
  }

  if (!TrackSpeedControlActive)
  {
    TrackSpeedControlActive = 1U;
    TrackSpeedControlIntegral = 0.0f;
    TrackSpeedCorrectionUs = 0;
    TrackSpeedControlLastSampleTick = SpeedLastTick;
    return 0;
  }

  /* 每得到一组新的 20 ms 编码器速度样本才运行一次 PI。 */
  if (TrackSpeedControlLastSampleTick == SpeedLastTick)
  {
    return TrackSpeedCorrectionUs;
  }
  TrackSpeedControlLastSampleTick = SpeedLastTick;

  average_speed = (LeftTrackControlSpeedCps + RightTrackControlSpeedCps) / 2;
  if (average_speed < TRACK_SPEED_CONTROL_MIN_AVERAGE_CPS)
  {
    TrackSpeedControlIntegral = 0.0f;
    TrackSpeedCorrectionUs = 0;
    return 0;
  }

  speed_error = LeftTrackControlSpeedCps - RightTrackControlSpeedCps;
  if ((speed_error > -TRACK_SPEED_CONTROL_DEADBAND_CPS) &&
      (speed_error < TRACK_SPEED_CONTROL_DEADBAND_CPS))
  {
    speed_error = 0;
  }

  candidate_integral = TrackSpeedControlIntegral + (float)speed_error;
  if (candidate_integral > TRACK_SPEED_CONTROL_INTEGRAL_LIMIT_CPS)
  {
    candidate_integral = TRACK_SPEED_CONTROL_INTEGRAL_LIMIT_CPS;
  }
  else if (candidate_integral < -TRACK_SPEED_CONTROL_INTEGRAL_LIMIT_CPS)
  {
    candidate_integral = -TRACK_SPEED_CONTROL_INTEGRAL_LIMIT_CPS;
  }

  raw_correction = TRACK_SPEED_CONTROL_KP_US_PER_CPS * (float)speed_error +
                   TRACK_SPEED_CONTROL_KI_US_PER_SAMPLE * candidate_integral;
  if (raw_correction > (float)TRACK_SPEED_CONTROL_MAX_CORRECTION_US)
  {
    TrackSpeedCorrectionUs = TRACK_SPEED_CONTROL_MAX_CORRECTION_US;
    /* 已到正限幅时只允许反向误差释放积分。 */
    if (speed_error < 0)
    {
      TrackSpeedControlIntegral = candidate_integral;
    }
  }
  else if (raw_correction < -(float)TRACK_SPEED_CONTROL_MAX_CORRECTION_US)
  {
    TrackSpeedCorrectionUs = -TRACK_SPEED_CONTROL_MAX_CORRECTION_US;
    /* 已到负限幅时只允许反向误差释放积分。 */
    if (speed_error > 0)
    {
      TrackSpeedControlIntegral = candidate_integral;
    }
  }
  else
  {
    TrackSpeedControlIntegral = candidate_integral;
    TrackSpeedCorrectionUs = (raw_correction >= 0.0f) ?
                             (int32_t)(raw_correction + 0.5f) :
                             (int32_t)(raw_correction - 0.5f);
  }

  return TrackSpeedCorrectionUs;
}

/*
 * Tank control. With steering priority enabled, any right-stick horizontal
 * command drives the tracks in opposite directions, so it turns in place.
 */
static void UpdateTankDrive(uint32_t throttle_pulse_us, uint32_t steering_pulse_us)
{
  int32_t throttle = RcPulseToCommand(throttle_pulse_us, RC_CH3_DEADBAND_US);
  int32_t steering = RcPulseToCommand(steering_pulse_us, RC_CH1_DEADBAND_US);
  int32_t right_command;
  int32_t left_command;
  int32_t speed_correction;
  int32_t drive_direction;
  uint16_t right_pwm;
  uint16_t left_pwm;

#if TANK_THROTTLE_REVERSED
  throttle = -throttle;
#endif
#if TANK_STEERING_REVERSED
  steering = -steering;
#endif

  /* Positive steering: left track forward, right track reverse. */
#if TANK_STEERING_PRIORITY
  if (steering != 0)
  {
    right_command = -steering;
    left_command  =  steering;
  }
  else
  {
    right_command = throttle;
    left_command  = throttle;
  }
#else
  right_command = throttle - steering;
  left_command  = throttle + steering;
#endif

  if (YawHoldActive != 0U)
  {
    /*
     * The encoder PI drives left/right speed error toward zero and would
     * oppose an intentional heading differential. Keep one controller active
     * at a time. Heading correction sign is independent of travel direction
     * because it controls signed yaw rate, not track speed magnitude.
     */
    ResetTrackSpeedControl();
    right_command += YawHoldCorrectionUs;
    left_command  -= YawHoldCorrectionUs;
  }
  else
  {
    /*
     * 速度同步修正的是履带速度幅值：前进时正修正增加右 PWM，倒车时正修正
     * 减小右 PWM（让右侧反向幅值更大）。左右等量反向叠加，不改变平均油门。
     */
    speed_correction = UpdateTrackSpeedControl(throttle, steering);
    drive_direction = (throttle >= 0) ? 1 : -1;
    right_command += drive_direction * speed_correction;
    left_command  -= drive_direction * speed_correction;
  }

#if TANK_RIGHT_TRACK_REVERSED
  right_command = -right_command;
#endif
#if TANK_LEFT_TRACK_REVERSED
  left_command = -left_command;
#endif

  /* 线路实测：PE9 / TIM1_CH1 驱动左履带，PE11 / TIM1_CH2 驱动右履带。 */
  right_pwm = CommandToRcPulse(right_command);
  left_pwm = CommandToRcPulse(left_command);
  CommitTrackPwm(left_pwm, right_pwm);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  /* USART2 (remapped to PD6 RX) IMU is optional: a missing sensor must not stop vehicle control. */
  (void)IMU_UART_Init();

  /* 启动 TIM1 CH1/CH2 PWM 输出 */
  /* Start both ESC/motor-controller outputs at their neutral pulse width. */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, RC_OUTPUT_CENTER_US);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, RC_OUTPUT_CENTER_US);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  TrackPwmReady = 1U;
  RefreshInfraredInputs();
  CommitTrackPwm(RC_OUTPUT_CENTER_US, RC_OUTPUT_CENTER_US);

  /* TIM5 为原有六路 PWM 输入提供 1 MHz 时间基准。 */
  HAL_TIM_Base_Start(&htim5);

  /* Four identical hardware x4 encoder timers start from one known origin. */
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_SET_COUNTER(&htim8, 0U);
  Encoder1_Count = 0;
  Encoder2_Count = 0;
  Encoder3_Count = 0;
  Encoder4_Count = 0;
  Encoder1_LastCounter = 0;
  Encoder2_LastCounter = 0;
  Encoder3_LastCounter = 0;
  Encoder4_LastCounter = 0;

  /* 四路 AB 编码器全部使用相同的硬件正交 x4 计数配置。 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  SpeedPrevEncoder1 = 0;
  SpeedPrevEncoder2 = 0;
  SpeedPrevEncoder3 = 0;
  SpeedPrevEncoder4 = 0;
  SpeedLastTick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    RefreshInfraredInputs();
    IMU_UART_Process();

    UpdateEncoderCounters();
    UpdateTrackSpeedMeasurement(now);

    /* 仅保留红外急停锁存；障碍解除后先回中，再接受新的推杆命令。 */
    UpdateInfraredInterlock();
    now = HAL_GetTick();
    UpdateYawHold(now);

    /* 红外锁存优先级最高；解除前禁止遥控和闭环覆盖中位。 */
    if (EmergencyStopActive || EmergencyStopLatched)
    {
      ResetTrackSpeedControl();
      CommitTrackPwm(RC_OUTPUT_CENTER_US, RC_OUTPUT_CENTER_US);
    }
    else
    {
      /* CH3 controls forward/reverse; CH1 remains the steering input. */
      UpdateTankDrive(CH3_PulseWidth, CH1_PulseWidth);
    }

    /* USART1/PA9 encoder CSV (115200 8N1): ENC1,ENC2,ENC3,ENC4. */
    if ((now - LastDebugTick) >= DEBUG_TELEMETRY_INTERVAL_MS)
    {
      int n = sprintf(dbg_buf,
                      "%ld,%ld,%ld,%ld\r\n",
                      (long)Encoder1_Count,
                      (long)Encoder2_Count,
                      (long)Encoder3_Count,
                      (long)Encoder4_Count);
      HAL_UART_Transmit(&huart1, (uint8_t *)dbg_buf, n, 20);
      LastDebugTick = now;
    }

    /* USART3/PB10 attitude CSV: Yaw,Pitch,Roll. */
    if ((now - LastAttitudeUartTick) >= ATTITUDE_UART_INTERVAL_MS)
    {
      int32_t yaw_cd = AngleToUnsignedCentiDegrees(imu_yaw);
      int32_t pitch_cd = AngleToCentiDegrees(imu_pitch);
      int32_t roll_cd = AngleToCentiDegrees(imu_roll);
      int n = sprintf(attitude_buf,
                      "%ld.%02ld,%c%ld.%02ld,%c%ld.%02ld\r\n",
                      (long)(yaw_cd / 100),
                      (long)(yaw_cd % 100),
                      (pitch_cd < 0) ? '-' : '+',
                      (long)(AbsI32(pitch_cd) / 100),
                      (long)(AbsI32(pitch_cd) % 100),
                      (roll_cd < 0) ? '-' : '+',
                      (long)(AbsI32(roll_cd) / 100),
                      (long)(AbsI32(roll_cd) % 100));
      HAL_UART_Transmit(&huart3, (uint8_t *)attitude_buf, n, 10);
      LastAttitudeUartTick = now;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* PA0/PA1 are intentionally not configured. Pin 0/1 therefore means PB0/PB1. */
  switch (GPIO_Pin)
  {
    case GPIO_PIN_0:
      RcCaptureEdge(&CH5_PulseWidth, &CH5_InputFilter,
                    &CH5_LastRise, &CH5_State, &CH5_LastTick,
                    GPIOB, GPIO_PIN_0);
      break;
    case GPIO_PIN_1:
      RcCaptureEdge(&CH6_PulseWidth, &CH6_InputFilter,
                    &CH6_LastRise, &CH6_State, &CH6_LastTick,
                    GPIOB, GPIO_PIN_1);
      break;
    case GPIO_PIN_2:
      RcCaptureEdge(&CH2_PulseWidth, &CH2_InputFilter,
                    &CH2_LastRise, &CH2_State, &CH2_LastTick,
                    GPIOA, GPIO_PIN_2);
      break;
    case GPIO_PIN_3:
      RcCaptureEdge(&CH3_PulseWidth, &CH3_InputFilter,
                    &CH3_LastRise, &CH3_State, &CH3_LastTick,
                    GPIOA, GPIO_PIN_3);
      break;
    case GPIO_PIN_4:
      IR1_Detected = (HAL_GPIO_ReadPin(IR1_GPIO_Port, IR1_Pin) == GPIO_PIN_RESET);
      break;
    case GPIO_PIN_5:
      IR2_Detected = (HAL_GPIO_ReadPin(IR2_GPIO_Port, IR2_Pin) == GPIO_PIN_RESET);
      break;
    case GPIO_PIN_6:
      RcCaptureEdge(&CH1_PulseWidth, &CH1_InputFilter,
                    &CH1_LastRise, &CH1_State, &CH1_LastTick,
                    GPIOA, GPIO_PIN_6);
      break;
    case GPIO_PIN_7:
      RcCaptureEdge(&CH4_PulseWidth, &CH4_InputFilter,
                    &CH4_LastRise, &CH4_State, &CH4_LastTick,
                    GPIOA, GPIO_PIN_7);
      break;
    case GPIO_PIN_8:
      IR3_Detected = (HAL_GPIO_ReadPin(IR3_GPIO_Port, IR3_Pin) == GPIO_PIN_RESET);
      break;
    case GPIO_PIN_9:
      IR4_Detected = (HAL_GPIO_ReadPin(IR4_GPIO_Port, IR4_Pin) == GPIO_PIN_RESET);
      break;
    default:
      break;
  }

  EmergencyStopActive = (IR1_Detected != 0U) || (IR2_Detected != 0U) ||
                        (IR3_Detected != 0U) || (IR4_Detected != 0U);
  if (EmergencyStopActive)
  {
    EmergencyStopLatched = 1U;
    /* 中断内立即锁存并回中；CommitTrackPwm 防止主循环在竞态窗口覆盖急停。 */
    CommitTrackPwm(RC_OUTPUT_CENTER_US, RC_OUTPUT_CENTER_US);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
