#include "mpu6500.h"
#include "spi.h"

#define MPU6500_WHO_AM_I           0x75U
#define MPU6500_SMPLRT_DIV         0x19U
#define MPU6500_CONFIG             0x1AU
#define MPU6500_GYRO_CONFIG        0x1BU
#define MPU6500_ACCEL_CONFIG       0x1CU
#define MPU6500_USER_CTRL          0x6AU
#define MPU6500_ACCEL_XOUT_H       0x3BU
#define MPU6500_GYRO_ZOUT_H        0x47U
#define MPU6500_PWR_MGMT_1         0x6BU
#define MPU6500_PWR_MGMT_2         0x6CU

#define ACCEL_SENSITIVITY          8192.0f  /* +/-4 g */
#define GYRO_SENSITIVITY           65.5f    /* +/-500 degree/s */
#define RAD_TO_DEG                 57.2957795f
#define FILTER_ALPHA               0.98f
#define UPDATE_PERIOD_MS           10U
#define ATTITUDE_UPDATE_PERIOD_MS  20U
#define REINIT_DELAY_MS            1000U
#define MAX_RUNTIME_YAW_RATE_DPS   450.0f
#define MAX_RUNTIME_RATE_STEP_RAW  512
#define YAW_RATE_FILTER_ALPHA      0.35f
#define STATIONARY_BIAS_RATE_LIMIT_DPS  0.6f
#define STATIONARY_BIAS_SETTLE_SAMPLES   50U
#define STATIONARY_BIAS_ADAPT_ALPHA    0.01f
#define STATIONARY_YAW_RATE_DEADBAND_DPS 0.05f

#define MPU_INIT_FAIL_NONE         0U
#define MPU_INIT_FAIL_NO_DEVICE    1U
#define MPU_INIT_FAIL_RESET        2U
#define MPU_INIT_FAIL_CONFIG       3U

MPU6500_Data_t mpu6500_data;
float mpu6500_yaw;
float mpu6500_pitch;
float mpu6500_roll;

static uint8_t mpu6500_ready;
static uint8_t mpu6500_read_failures;
static uint8_t mpu6500_bias_valid;
static uint32_t mpu6500_last_update_ms;  /* 最近一次积分（成功读取或外推）的时刻 */
static uint32_t mpu6500_last_attempt_ms; /* 最近一次尝试读取的时刻（节流用） */
static uint32_t mpu6500_last_attitude_attempt_ms;
static uint32_t mpu6500_last_attitude_update_ms;
static uint32_t mpu6500_next_reinit_ms;
static uint8_t mpu6500_init_failure;
static uint8_t mpu6500_last_who_am_i;
static uint32_t mpu6500_last_spi_error;
static float gyro_bias_x;
static float gyro_bias_y;
static float gyro_bias_z;
static float runtime_yaw_rate_filtered;
static uint8_t runtime_yaw_rate_valid;
static float mpu6500_last_gyro_z;        /* 最近一次读到的偏航角速度，用于失败外推 */
static uint8_t mpu6500_stationary;
static uint16_t stationary_bias_samples;

/* Avoids a math-library dependency while retaining adequate angle precision. */
static float FastAtan2(float y, float x)
{
  float abs_y = (y < 0.0f) ? -y : y;
  float ratio;
  float angle;

  if ((x == 0.0f) && (abs_y == 0.0f)) return 0.0f;
  if (x >= 0.0f)
  {
    ratio = (x - abs_y) / (x + abs_y);
    angle = 0.785398164f * (1.0f - ratio);
  }
  else
  {
    ratio = (x + abs_y) / (abs_y - x);
    angle = 2.35619449f - 0.785398164f * ratio;
  }
  return (y < 0.0f) ? -angle : angle;
}

static float FastSqrt(float value)
{
  float estimate;
  uint8_t i;
  if (value <= 0.0f) return 0.0f;
  estimate = (value > 1.0f) ? value : 1.0f;
  for (i = 0U; i < 6U; ++i)
  {
    estimate = 0.5f * (estimate + value / estimate);
  }
  return estimate;
}

static HAL_StatusTypeDef ReadRegister(uint8_t reg, uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef status;
  /* SPI 事务失败由上层短间隔重试，错误状态保留用于诊断。 */
  status = SPI2_ReadRegisters(reg, data, length);
  mpu6500_last_spi_error = (status == HAL_OK) ? 0U : (uint32_t)status;
  return status;
}

static HAL_StatusTypeDef WriteRegister(uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef status = SPI2_WriteRegister(reg, value);
  mpu6500_last_spi_error = (status == HAL_OK) ? 0U : (uint32_t)status;
  return status;
}

static HAL_StatusTypeDef WriteRegisterRetry(uint8_t reg, uint8_t value)
{
  uint8_t attempt;
  for (attempt = 0U; attempt < 3U; ++attempt)
  {
    if (WriteRegister(reg, value) == HAL_OK) return HAL_OK;
    HAL_Delay(2U);
  }
  return HAL_ERROR;
}

static HAL_StatusTypeDef ReadRegisterRetry(uint8_t reg, uint8_t *data,
                                            uint16_t length)
{
  uint8_t attempt;
  for (attempt = 0U; attempt < 2U; ++attempt)
  {
    if (ReadRegister(reg, data, length) == HAL_OK) return HAL_OK;
    HAL_Delay(1U);
  }
  return HAL_ERROR;
}

/*
 * During motion only yaw is needed by the straight-line controller.  Reading
 * the complete 14-byte accel/gyro frame makes the SPI transaction longer
 * and increases the chance that motor EMI corrupts a byte.  Read the gyro-Z
 * register pair only and reject an implausible frame instead of integrating
 * it into yaw.  The full frame is still used during startup calibration.
 */
static HAL_StatusTypeDef ReadRuntimeYawRate(float *gyro_z)
{
  uint8_t buffer_a[2];
  uint8_t buffer_b[2];
  int16_t raw_a;
  int16_t raw_b;
  int32_t raw_delta;
  int32_t raw_average;
  float raw_rate;
  float rate;

  if (gyro_z == 0) return HAL_ERROR;
  if ((ReadRegisterRetry(MPU6500_GYRO_ZOUT_H, buffer_a, 2U) != HAL_OK) ||
      (ReadRegisterRetry(MPU6500_GYRO_ZOUT_H, buffer_b, 2U) != HAL_OK))
  {
    return HAL_ERROR;
  }

  raw_a = (int16_t)(((uint16_t)buffer_a[0] << 8) | buffer_a[1]);
  raw_b = (int16_t)(((uint16_t)buffer_b[0] << 8) | buffer_b[1]);
  raw_delta = (int32_t)raw_b - (int32_t)raw_a;
  if (raw_delta < 0) raw_delta = -raw_delta;
  if (raw_delta > MAX_RUNTIME_RATE_STEP_RAW) return HAL_ERROR;

  raw_average = ((int32_t)raw_a + (int32_t)raw_b) / 2;
  raw_rate = (float)raw_average / GYRO_SENSITIVITY;
  rate = raw_rate - gyro_bias_z;

  /*
   * 仅在主控同时确认 PWM 回中和履带停止后在线追踪温漂。先连续观察
   * 0.5 s 的低角速度，再缓慢修正 bias；超过阈值立即认为车体在转动，
   * 不把真实转动吸收到零偏里。
   */
  if (mpu6500_stationary &&
      (rate < STATIONARY_BIAS_RATE_LIMIT_DPS) &&
      (rate > -STATIONARY_BIAS_RATE_LIMIT_DPS))
  {
    if (stationary_bias_samples < STATIONARY_BIAS_SETTLE_SAMPLES)
    {
      ++stationary_bias_samples;
    }
    else
    {
      gyro_bias_z += STATIONARY_BIAS_ADAPT_ALPHA * rate;
      rate = raw_rate - gyro_bias_z;
    }
  }
  else
  {
    stationary_bias_samples = 0U;
  }
  /* A tracked vehicle cannot turn at hundreds of degrees per second here;
     this also rejects common single-byte SPI corruption patterns. */
  if ((rate > MAX_RUNTIME_YAW_RATE_DPS) ||
      (rate < -MAX_RUNTIME_YAW_RATE_DPS))
  {
    return HAL_ERROR;
  }
  if (!runtime_yaw_rate_valid)
  {
    runtime_yaw_rate_filtered = rate;
    runtime_yaw_rate_valid = 1U;
  }
  else
  {
    runtime_yaw_rate_filtered += YAW_RATE_FILTER_ALPHA *
                                  (rate - runtime_yaw_rate_filtered);
  }
  if (mpu6500_stationary &&
      (stationary_bias_samples >= STATIONARY_BIAS_SETTLE_SAMPLES) &&
      (runtime_yaw_rate_filtered < STATIONARY_YAW_RATE_DEADBAND_DPS) &&
      (runtime_yaw_rate_filtered > -STATIONARY_YAW_RATE_DEADBAND_DPS))
  {
    runtime_yaw_rate_filtered = 0.0f;
  }
  *gyro_z = runtime_yaw_rate_filtered;
  return HAL_OK;
}

static uint8_t IsSupportedWhoAmI(uint8_t who_am_i)
{
  /* MPU6500=0x70; MPU9250/9255=0x71/0x73. MPU6050=0x68 is I2C-only. */
  return (who_am_i == 0x70U) || (who_am_i == 0x71U) ||
         (who_am_i == 0x73U);
}

static void SetInitialAttitude(const MPU6500_Data_t *data)
{
  float horizontal = FastSqrt(data->accel_y * data->accel_y + data->accel_z * data->accel_z);
  mpu6500_roll = FastAtan2(data->accel_y, data->accel_z) * RAD_TO_DEG;
  mpu6500_pitch = FastAtan2(-data->accel_x, horizontal) * RAD_TO_DEG;
  mpu6500_yaw = 0.0f;
}

static void ScheduleReinit(uint32_t now_ms)
{
  mpu6500_ready = 0U;
  mpu6500_read_failures = 0U;
  mpu6500_next_reinit_ms = now_ms + REINIT_DELAY_MS;
}

uint8_t MPU6500_Init(void)
{
  uint8_t who_am_i;

  mpu6500_ready = 0U;
  mpu6500_init_failure = MPU_INIT_FAIL_NONE;
  mpu6500_last_who_am_i = 0U;
  runtime_yaw_rate_filtered = 0.0f;
  runtime_yaw_rate_valid = 0U;
  stationary_bias_samples = 0U;
  HAL_Delay(20U);
  who_am_i = 0U;
  if (ReadRegisterRetry(MPU6500_WHO_AM_I, &who_am_i, 1U) != HAL_OK)
  {
    mpu6500_init_failure = MPU_INIT_FAIL_NO_DEVICE;
    ScheduleReinit(HAL_GetTick());
    return 0U;
  }
  mpu6500_last_who_am_i = who_am_i;
  if (!IsSupportedWhoAmI(who_am_i))
  {
    mpu6500_init_failure = MPU_INIT_FAIL_NO_DEVICE;
    ScheduleReinit(HAL_GetTick());
    return 0U;
  }

  if (WriteRegisterRetry(MPU6500_PWR_MGMT_1, 0x80U) != HAL_OK)
  {
    mpu6500_init_failure = MPU_INIT_FAIL_RESET;
    ScheduleReinit(HAL_GetTick());
    return 0U;
  }
  HAL_Delay(100U);

  /* PLL clock, DLPF 20 Hz, gyro +/-500 dps, accel +/-4 g, 100 Hz output. */
  if ((WriteRegisterRetry(MPU6500_USER_CTRL, 0x10U) != HAL_OK) ||
      (WriteRegisterRetry(MPU6500_PWR_MGMT_1, 0x01U) != HAL_OK) ||
      (WriteRegisterRetry(MPU6500_PWR_MGMT_2, 0x00U) != HAL_OK) ||
      (WriteRegisterRetry(MPU6500_CONFIG, 0x04U) != HAL_OK) ||
      (WriteRegisterRetry(MPU6500_SMPLRT_DIV, 0x09U) != HAL_OK) ||
      (WriteRegisterRetry(MPU6500_GYRO_CONFIG, 0x08U) != HAL_OK) ||
      (WriteRegisterRetry(MPU6500_ACCEL_CONFIG, 0x08U) != HAL_OK))
  {
    mpu6500_init_failure = MPU_INIT_FAIL_CONFIG;
    ScheduleReinit(HAL_GetTick());
    return 0U;
  }

  /* Keep a completed static calibration when reconnecting during vehicle motion. */
  if (!mpu6500_bias_valid)
  {
    gyro_bias_x = gyro_bias_y = gyro_bias_z = 0.0f;
  }
  mpu6500_read_failures = 0U;
  mpu6500_last_update_ms = 0U;
  mpu6500_last_attempt_ms = 0U;
  mpu6500_last_attitude_attempt_ms = 0U;
  mpu6500_last_attitude_update_ms = 0U;
  mpu6500_next_reinit_ms = 0U;
  mpu6500_init_failure = MPU_INIT_FAIL_NONE;
  mpu6500_ready = 1U;
  return 1U;
}

uint8_t MPU6500_ReadData(MPU6500_Data_t *data)
{
  uint8_t buffer[14];
  int16_t raw;
  if ((!mpu6500_ready) || (data == 0)) return 0U;
  if (ReadRegisterRetry(MPU6500_ACCEL_XOUT_H, buffer, 14U) != HAL_OK) return 0U;

  raw = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
  data->accel_x = (float)raw / ACCEL_SENSITIVITY;
  raw = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
  data->accel_y = (float)raw / ACCEL_SENSITIVITY;
  raw = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);
  data->accel_z = (float)raw / ACCEL_SENSITIVITY;
  raw = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);
  data->gyro_x = (float)raw / GYRO_SENSITIVITY - gyro_bias_x;
  raw = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
  data->gyro_y = (float)raw / GYRO_SENSITIVITY - gyro_bias_y;
  raw = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);
  data->gyro_z = (float)raw / GYRO_SENSITIVITY - gyro_bias_z;
  return 1U;
}

void MPU6500_Calibrate(uint16_t samples)
{
  MPU6500_Data_t sample;
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  uint16_t accepted = 0U, attempts = 0U;
  if ((!mpu6500_ready) || (samples == 0U)) return;

  gyro_bias_x = gyro_bias_y = gyro_bias_z = 0.0f;
  mpu6500_bias_valid = 0U;
  while ((accepted < samples) && (attempts < (uint16_t)(samples * 3U)))
  {
    ++attempts;
    if (MPU6500_ReadData(&sample))
    {
      sum_x += sample.gyro_x;
      sum_y += sample.gyro_y;
      sum_z += sample.gyro_z;
      ++accepted;
    }
    HAL_Delay(UPDATE_PERIOD_MS);
  }
  if (accepted == samples)
  {
    gyro_bias_x = sum_x / (float)samples;
    gyro_bias_y = sum_y / (float)samples;
    gyro_bias_z = sum_z / (float)samples;
    mpu6500_bias_valid = 1U;
    runtime_yaw_rate_filtered = 0.0f;
    runtime_yaw_rate_valid = 0U;
    mpu6500_last_gyro_z = 0.0f;
    stationary_bias_samples = 0U;
    mpu6500_last_attitude_attempt_ms = 0U;
    mpu6500_last_attitude_update_ms = 0U;
  }
  if (MPU6500_ReadData(&mpu6500_data)) SetInitialAttitude(&mpu6500_data);
}

void MPU6500_SetStationary(uint8_t stationary)
{
  mpu6500_stationary = stationary ? 1U : 0U;
  if (!mpu6500_stationary)
  {
    stationary_bias_samples = 0U;
  }
}

uint8_t MPU6500_Update(uint32_t now_ms)
{
  float dt, attitude_dt, horizontal, pitch_accel, roll_accel;
  MPU6500_Data_t attitude_sample;
  uint8_t read_ok;
  if (!mpu6500_ready)
  {
    /* Retry after a short pause. Reset the SPI peripheral before retrying. */
    if ((int32_t)(now_ms - mpu6500_next_reinit_ms) >= 0)
    {
      MX_SPI2_Init();
      (void)MPU6500_Init();
    }
    return 0U;
  }
  if ((now_ms - mpu6500_last_attempt_ms) < UPDATE_PERIOD_MS) return 0U;
  mpu6500_last_attempt_ms = now_ms;

  /* After startup bias calibration, use the short gyro-Z transaction during
     motion.  If calibration was not completed, retain the full-frame path so
     the sensor can still recover and be diagnosed. */
  if (mpu6500_bias_valid)
  {
    read_ok = (ReadRuntimeYawRate(&mpu6500_data.gyro_z) == HAL_OK) ? 1U : 0U;
  }
  else
  {
    read_ok = MPU6500_ReadData(&mpu6500_data);
  }

  if (!read_ok)
  {
    ++mpu6500_read_failures;
    /* Three failures trigger a delayed automatic bus/device reinitialization. */
    if (mpu6500_read_failures >= 3U) ScheduleReinit(now_ms);
    /* 读失败期间用上一次角速度外推 yaw（上限 0.25 s），不让转角丢失；
     * 成功读取后 dt 从上次外推点起算，不会重复积分。 */
    if (mpu6500_last_update_ms != 0U)
    {
      float gap = (float)(now_ms - mpu6500_last_update_ms) / 1000.0f;
      if (gap > 0.25f) gap = 0.25f;
      mpu6500_yaw += mpu6500_last_gyro_z * gap;
      mpu6500_last_update_ms = now_ms;
    }
    return 0U;
  }
  mpu6500_read_failures = 0U;

  /* dt 从上次积分点（成功读取或外推）起算：行驶中振动/EMI 导致 SPI
   * 读失败时，失败期间损失的时间会在下一次成功读取时一并补上。
   * 旧实现把时间戳放在读取之前，失败一次就永久丢掉一段积分，
   * 表现为车转了几十度而 yaw 纹丝不动。 */
  dt = (mpu6500_last_update_ms == 0U) ? 0.01f :
       (float)(now_ms - mpu6500_last_update_ms) / 1000.0f;
  if (dt > 0.25f) dt = 0.25f;
  mpu6500_last_update_ms = now_ms;
  mpu6500_last_gyro_z = mpu6500_data.gyro_z;

  /* Yaw 保持 10 ms 双读短帧抗干扰；每 20 ms 额外读一次完整姿态帧，
     用加速度计 + gyro X/Y 互补滤波持续更新专用串口所需 Pitch/Roll。 */
  if (mpu6500_bias_valid)
  {
    mpu6500_yaw += mpu6500_data.gyro_z * dt;
    if (mpu6500_yaw > 180.0f) mpu6500_yaw -= 360.0f;
    if (mpu6500_yaw < -180.0f) mpu6500_yaw += 360.0f;

    if ((now_ms - mpu6500_last_attitude_attempt_ms) >= ATTITUDE_UPDATE_PERIOD_MS)
    {
      mpu6500_last_attitude_attempt_ms = now_ms;
      if (MPU6500_ReadData(&attitude_sample))
      {
        attitude_dt = (mpu6500_last_attitude_update_ms == 0U) ? 0.02f :
                      (float)(now_ms - mpu6500_last_attitude_update_ms) / 1000.0f;
        if (attitude_dt > 0.10f) attitude_dt = 0.10f;
        mpu6500_last_attitude_update_ms = now_ms;

        horizontal = FastSqrt(attitude_sample.accel_y * attitude_sample.accel_y +
                              attitude_sample.accel_z * attitude_sample.accel_z);
        pitch_accel = FastAtan2(-attitude_sample.accel_x, horizontal) * RAD_TO_DEG;
        roll_accel = FastAtan2(attitude_sample.accel_y,
                               attitude_sample.accel_z) * RAD_TO_DEG;
        mpu6500_pitch = FILTER_ALPHA *
                        (mpu6500_pitch + attitude_sample.gyro_y * attitude_dt) +
                        (1.0f - FILTER_ALPHA) * pitch_accel;
        mpu6500_roll = FILTER_ALPHA *
                       (mpu6500_roll + attitude_sample.gyro_x * attitude_dt) +
                       (1.0f - FILTER_ALPHA) * roll_accel;

        mpu6500_data.accel_x = attitude_sample.accel_x;
        mpu6500_data.accel_y = attitude_sample.accel_y;
        mpu6500_data.accel_z = attitude_sample.accel_z;
        mpu6500_data.gyro_x = attitude_sample.gyro_x;
        mpu6500_data.gyro_y = attitude_sample.gyro_y;
      }
    }
    return 1U;
  }

  horizontal = FastSqrt(mpu6500_data.accel_y * mpu6500_data.accel_y +
                        mpu6500_data.accel_z * mpu6500_data.accel_z);
  pitch_accel = FastAtan2(-mpu6500_data.accel_x, horizontal) * RAD_TO_DEG;
  roll_accel = FastAtan2(mpu6500_data.accel_y, mpu6500_data.accel_z) * RAD_TO_DEG;
  mpu6500_pitch = FILTER_ALPHA * (mpu6500_pitch + mpu6500_data.gyro_y * dt) +
                  (1.0f - FILTER_ALPHA) * pitch_accel;
  mpu6500_roll = FILTER_ALPHA * (mpu6500_roll + mpu6500_data.gyro_x * dt) +
                 (1.0f - FILTER_ALPHA) * roll_accel;
  mpu6500_yaw += mpu6500_data.gyro_z * dt;
  if (mpu6500_yaw > 180.0f) mpu6500_yaw -= 360.0f;
  if (mpu6500_yaw < -180.0f) mpu6500_yaw += 360.0f;
  return 1U;
}

uint8_t MPU6500_IsReady(void)
{
  return mpu6500_ready;
}

/* 诊断：当前连续读取失败次数（0=正常，>=3 即将触发总线重新初始化）。 */
uint8_t MPU6500_GetReadFailures(void)
{
  return mpu6500_read_failures;
}

uint8_t MPU6500_GetInitFailure(void)
{
  return mpu6500_init_failure;
}

uint8_t MPU6500_GetLastWhoAmI(void)
{
  return mpu6500_last_who_am_i;
}

uint32_t MPU6500_GetLastI2cError(void)
{
  /* Keep the old API name for compatibility; the value now reports SPI
     transfer status (0=OK, HAL status code on the latest failure). */
  return mpu6500_last_spi_error;
}
