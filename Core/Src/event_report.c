/**
  * @file    event_report.c
  * @brief   事件采集、4G联网、MQTT上报与失败落盘完整链路
  */

#include "event_report.h"
#include "main.h"
#include "lsm6ds.h"
#include "at_ml307c.h"
#include "event_store.h"
#include "adc.h"
#include "iwdg.h"
#include "uart_driver.h"
#include "sys_config.h"
#include "power_manager.h"
#include "rtc_utils.h"
#include "service_handler.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ========================== 私有宏 ========================== */

#define IMU_CAPTURE_TIME_MS    3000U /* 3秒采样窗口，期间可并行开机4G模组 */
#define IMU_SAMPLE_PERIOD_MS   10U
#define TILT_CONFIRM_TIME_MS   500U
#define TILT_CONFIRM_SAMPLES  (TILT_CONFIRM_TIME_MS / IMU_SAMPLE_PERIOD_MS)
#define IMU_SOURCE_SETTLE_MS   2U
#define NETWORK_BUDGET_MS      240000U /* 等待蜂窝/MQTTX通讯的总预算4分钟。 */
#define RTC_GNSS_FIX_TIMEOUT_MS 180000U /* RTC心跳必须最多等待3分钟定位。 */
#define WAKE_GNSS_FIX_TIMEOUT_MS 180000U /* 唤醒后GPS定位最长3分钟。 */
#define MANUAL_GNSS_FIX_TIMEOUT_MS 5000U /* 开机/维护手动上报保持原5秒定位预算。 */
#define GPS_SAMPLE_TIMEOUT_MS   3000U /* 持续跟踪时每次定位检测的最长等待。 */
#define GPS_TRACK_INTERVAL_MS   3000U /* GPS持续跟踪检测与上报周期。 */
#define GPS_STILL_DISTANCE_M    10.0f /* 位置变化不超过10米视为静止。 */
#define GPS_STILL_SAMPLE_COUNT  3U /* 连续三次静止后关闭4G。 */
#define EVENT_COOLDOWN_SEC      30U
#define REPORT_STAGE_IDLE        0U
#define REPORT_STAGE_CAPTURE     1U
#define REPORT_STAGE_MODEM_READY 2U
#define REPORT_STAGE_IMEI        3U
#define REPORT_STAGE_NETWORK     4U
#define REPORT_STAGE_RTC         5U
#define REPORT_STAGE_MQTT        6U
#define REPORT_STAGE_LOCATION    7U
#define REPORT_STAGE_PUBLISH     8U
#define REPORT_STAGE_QUEUE       9U
#define REPORT_STAGE_DOWNLINK   10U
#define REPORT_STAGE_SHUTDOWN   11U
#define REPORT_STAGE_COMPLETE   12U
#define FLASH_SAFE_VOLTAGE_MV  3450U
#define CRITICAL_VOLTAGE_MV    3350U

/* ========================== 全局变量 ========================== */

uint32_t g_volatile_event_seq = 1;         /**< 不写Flash事件使用的临时event_id序列。 */

uint8_t  g_last_report_ok = 0U;             /**< 最近完整上报是否成功。 */
uint8_t  g_last_report_stage = REPORT_STAGE_IDLE; /**< 最近上报结束/失败阶段。 */
uint8_t  g_last_report_fail = EVENT_FAIL_NONE;    /**< 最近上报失败原因。 */
uint8_t  g_last_report_csq = 99U;           /**< 最近网络信号CSQ，99表示未知。 */
uint8_t  g_last_report_attached = 0U;       /**< 最近一次网络附着结果。 */
uint32_t g_last_report_duration_ms = 0U;   /**< 最近完整链路耗时，ms。 */

uint32_t g_last_imu_event_time = 0U;       /**< 最近确认IMU事件的Unix时间。 */
uint8_t  g_last_imu_class = 0U;             /**< 最近IMU事件的软件分类标志。 */
uint8_t  g_last_imu_wake = EVENT_WAKE_UNKNOWN; /**< 最近IMU硬件唤醒类型。 */

uint16_t g_imu_false_wake_count = 0U;         /**< 3秒复核后不满足任何事件条件的次数。 */

/* ========================== 私有函数 ========================== */

static float GPS_Distance_Meters(const ML307C_GPS_Data_t *from,
                                 const ML307C_GPS_Data_t *to)
{
  const float deg_to_rad = 0.01745329252f;
  const float earth_radius_m = 6371000.0f;
  float lat1 = from->latitude * deg_to_rad;
  float lat2 = to->latitude * deg_to_rad;
  float dlat = lat2 - lat1;
  float dlon = (to->longitude - from->longitude) * deg_to_rad;
  float a = sinf(dlat * 0.5f) * sinf(dlat * 0.5f) +
            cosf(lat1) * cosf(lat2) *
            sinf(dlon * 0.5f) * sinf(dlon * 0.5f);
  if (a > 1.0f) a = 1.0f;
  return 2.0f * earth_radius_m * asinf(sqrtf(a));
}

/**
 * @brief 在已连接MQTT的前提下完成唤醒后的GPS更新和持续移动跟踪。
 * @note 初次定位最多等待3分钟；模组单次定位成功后自动关闭搜索。随后每3秒
 *       重发`AT+MGNSS=2`重新使能（不重置单次定位模式）查询位置并发送轻量
 *       GPS消息。连续三次与上一位置的距离不超过10米时返回，调用方随即关闭4G。
 */
static uint8_t Track_Wake_GPS(uint32_t event_id, uint32_t timestamp,
                              char *topic, EventRecord_t *event)
{
  ML307C_GPS_Data_t gps = {0};
  ML307C_GPS_Data_t previous = {0};
  uint8_t still_count = 0U;
  uint8_t has_previous = 0U;

  if (!ML307C_GPS_Start()) {
    gps.err_code = ML307C_LOC_ERR_START;
    event->fail_reason = EVENT_FAIL_GNSS;
    return ML307C_Send_GPS_Update(event_id, timestamp, &gps, topic);
  }
  if (!ML307C_GPS_Wait_Fix(&gps, WAKE_GNSS_FIX_TIMEOUT_MS)) {
    if (!ML307C_GPS_Stop() && gps.err_code == ML307C_LOC_ERR_UNKNOWN)
      gps.err_code = ML307C_LOC_ERR_STOP;
    else if (gps.err_code == ML307C_LOC_ERR_UNKNOWN)
      gps.err_code = ML307C_LOC_ERR_TIMEOUT;
    gps.is_fixed = 0;
    event->fail_reason = EVENT_FAIL_GNSS;
    return ML307C_Send_GPS_Update(event_id, timestamp, &gps, topic);
  }

  if (!ML307C_Send_GPS_Update(event_id, timestamp, &gps, topic)) return 0U;
  previous = gps;
  has_previous = 1U;

  while (1) {
    ML307C_GPS_Data_t current = {0};
    uint32_t sample_start = HAL_GetTick();
    uint8_t gps_requeried;

    /* 单次定位成功会关闭MGNSS，这里仅重发`AT+MGNSS=2`重新使能；只有
     * 重发成功但本次未定位时才需要主动Stop。 */
    gps_requeried = ML307C_GPS_Requery() ? 1U : 0U;
    if (gps_requeried &&
        ML307C_GPS_Wait_Fix(&current, GPS_SAMPLE_TIMEOUT_MS)) {
      float distance_m = has_previous ? GPS_Distance_Meters(&previous, &current) : 0.0f;
      if (!ML307C_Send_GPS_Update(event_id, timestamp, &current, topic)) return 0U;
      previous = current;
      has_previous = 1U;
      if (distance_m <= GPS_STILL_DISTANCE_M) {
        if (++still_count >= GPS_STILL_SAMPLE_COUNT) return 1U;
      } else {
        still_count = 0U;
      }
    } else {
      if (gps_requeried) (void)ML307C_GPS_Stop();
      still_count = 0U;
    }

    while ((HAL_GetTick() - sample_start) < GPS_TRACK_INTERVAL_MS) {
      HAL_IWDG_Refresh(&hiwdg);
      Service_Task(1U);
      HAL_Delay(10U);
    }
  }
}

/**
 * @brief 将浮点测量值饱和转换为有符号16位整数。 */
static int16_t Clamp_Int16(float value)
{
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return (int16_t)value;
}

/** @brief 从二进制维护协议payload读取小端16位无符号数。 */
uint16_t Read_LE16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/** @brief 从二进制维护协议payload读取小端32位无符号数。 */
uint32_t Read_LE32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/** @brief 向维护协议响应payload写入小端16位无符号数。 */
void Write_LE16(uint8_t *p, uint16_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
}

/** @brief 向维护协议响应payload写入小端32位无符号数。 */
void Write_LE32(uint8_t *p, uint32_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
  p[2] = (uint8_t)(value >> 16);
  p[3] = (uint8_t)(value >> 24);
}

/**
 * @brief 计算当前重力向量与配置安装零轴之间的夹角。
 * @param acc_mg 三轴加速度，单位mg。
 * @return 0~18000厘度；向量模长过小返回0。
 * @details 零度由g_cfg.mount_axis固定指定，不会以上一次姿态作为动态基准。
 */
static int16_t Tilt_From_Accel_Cdeg(const float acc_mg[3])
{
  float norm = sqrtf(acc_mg[0] * acc_mg[0] +
                     acc_mg[1] * acc_mg[1] +
                     acc_mg[2] * acc_mg[2]);
  float reference_component;
  if (norm < 100.0f) return 0;
  switch (g_cfg.mount_axis) {
  case MOUNT_AXIS_Z_NEG: reference_component = -acc_mg[2]; break;
  case MOUNT_AXIS_X_POS: reference_component =  acc_mg[0]; break;
  case MOUNT_AXIS_X_NEG: reference_component = -acc_mg[0]; break;
  case MOUNT_AXIS_Y_POS: reference_component =  acc_mg[1]; break;
  case MOUNT_AXIS_Y_NEG: reference_component = -acc_mg[1]; break;
  case MOUNT_AXIS_Z_POS:
  default:               reference_component =  acc_mg[2]; break;
  }
  reference_component /= norm;
  if (reference_component > 1.0f) reference_component = 1.0f;
  if (reference_component < -1.0f) reference_component = -1.0f;
  return Clamp_Int16(acosf(reference_component) * 5729.578f);
}

/**
 * @brief 从加速度计计算pitch/roll（传感器体坐标系欧拉角）。
 * @param acc_mg 三轴加速度，单位mg。
 * @param pitch_cdeg 输出pitch，绕Y轴旋转，单位0.01°。
 * @param roll_cdeg  输出roll，绕X轴旋转，单位0.01°。
 * @details pitch=atan2(-ax,√(ay²+az²))，roll=atan2(ay,az)，静止时即重力相对体轴的角度。
 */
static void Accel_To_Pitch_Roll_Cdeg(const float acc_mg[3],
                                     int16_t *pitch_cdeg, int16_t *roll_cdeg)
{
  float norm_xy = sqrtf(acc_mg[1] * acc_mg[1] + acc_mg[2] * acc_mg[2]);
  float pitch_rad = atan2f(-acc_mg[0], norm_xy);
  float roll_rad  = atan2f(acc_mg[1], acc_mg[2]);
  *pitch_cdeg = Clamp_Int16(pitch_rad * 5729.578f);
  *roll_cdeg  = Clamp_Int16(roll_rad * 5729.578f);
}

/* ========================== 公共函数 ========================== */

/**
 * @brief 连续采集3秒IMU，同时可并行执行ML307C开机脉冲和MATREADY监听。
 * @param event 输出事件采样统计，调用前应清零并填好基础元数据。
 * @param start_modem 1并行启动/探测4G；0只采样，不给模组上电。
 * @return start_modem为1时表示AT握手成功；只采样时固定返回0。
 * @details 每10 ms采样一次，倾角连续超阈值500 ms才设置EVENT_FLAG_TILTED。
 */
uint8_t Capture_Event_And_Start_Modem(EventRecord_t *event,
                                     uint8_t start_modem)
{
  uint32_t start;
  uint32_t next_sample;
  uint8_t pulse_active = 0U;
  uint8_t matready_seen = 0U;
  uint8_t first_sample = 1U;
  uint16_t tilt_over_samples = 0U;
  int16_t pitch_start_cdeg = 0, pitch_final_cdeg = 0; /**< pitch开始/最终角，本地追踪。 */
  int16_t roll_start_cdeg = 0, roll_final_cdeg = 0;   /**< roll开始/最终角，本地追踪。 */
  float yaw_accum_cdeg = 0.0f;                        /**< yaw净旋转角累积，厘度。 */

  if (event == NULL) return 0U;
  (void)LSM6DS_Set_Active_Mode();

  if (start_modem) {
    if (!ML307C_Is_Powered()) {
      ML307C_Begin_PowerOn();
      pulse_active = 1U;
    } else {
      ML307C_Clear_Buffer();
    }
  }

  start = HAL_GetTick();
  next_sample = start;
  while ((HAL_GetTick() - start) < IMU_CAPTURE_TIME_MS) {
    uint32_t now = HAL_GetTick();
    if (pulse_active && (now - start) >= 2300U) {
      ML307C_End_PowerOn_Pulse();
      pulse_active = 0U;
    }
    if (start_modem && ML307C_Poll_MATREADY()) matready_seen = 1U;

    if ((int32_t)(now - next_sample) >= 0) {
      float acc[3] = {0};
      float gyro[3] = {0};
      if (LSM6DS_Read_Storage(acc, gyro)) {
        int16_t tilt = Tilt_From_Accel_Cdeg(acc);
        int16_t pitch_cdeg, roll_cdeg;
        float norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        Accel_To_Pitch_Roll_Cdeg(acc, &pitch_cdeg, &roll_cdeg);
        if (first_sample) {
          pitch_start_cdeg = pitch_cdeg;
          roll_start_cdeg = roll_cdeg;
          first_sample = 0U;
        }
        pitch_final_cdeg = pitch_cdeg;
        roll_final_cdeg = roll_cdeg;
        /* tilt仍用于事件分类：连续超阈值500 ms确认倾倒，但不随事件存储。 */
        if (tilt >= (int16_t)(g_cfg.tilt_deg * 100U)) {
          if (tilt_over_samples < TILT_CONFIRM_SAMPLES) tilt_over_samples++;
          if (tilt_over_samples >= TILT_CONFIRM_SAMPLES)
            event->flags |= EVENT_FLAG_TILTED;
        } else {
          tilt_over_samples = 0U;
        }
        if (norm > event->acc_norm_peak_mg) event->acc_norm_peak_mg = (uint16_t)norm;
        /* yaw: 角速度投影到重力轴(单位矢量acc/|acc|)后积分，得到绕竖直轴的净旋转角。 */
        if (norm > 100.0f) {
          float yaw_rate_dps = (gyro[0] * acc[0] + gyro[1] * acc[1] +
                                gyro[2] * acc[2]) / norm;
          yaw_accum_cdeg += yaw_rate_dps * (IMU_SAMPLE_PERIOD_MS / 1000.0f) * 100.0f;
        }
        for (uint8_t axis = 0U; axis < 3U; axis++) {
          float abs_acc = fabsf(acc[axis]);
          float abs_gyro = fabsf(gyro[axis]);
          event->acc_final_mg[axis] = Clamp_Int16(acc[axis]);
          event->gyro_final_dps[axis] = Clamp_Int16(gyro[axis]);
          if (abs_acc > event->acc_peak_mg[axis])
            event->acc_peak_mg[axis] = Clamp_Int16(abs_acc);
          if (abs_gyro > event->gyro_peak_dps[axis])
            event->gyro_peak_dps[axis] = Clamp_Int16(abs_gyro);
        }
        if (event->sample_count < 0xFFFFU) event->sample_count++;
      }
      next_sample += IMU_SAMPLE_PERIOD_MS;
    }
    Service_Task(1U);
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(1);
  }
  if (pulse_active) ML307C_End_PowerOn_Pulse();

  /* 三轴变换角[pitch, roll, yaw]：pitch/roll为最终-开始，yaw为陀螺积分净变化。 */
  event->tilt_change_cdeg[0] = Clamp_Int16((float)((int32_t)pitch_final_cdeg -
                                                   pitch_start_cdeg));
  event->tilt_change_cdeg[1] = Clamp_Int16((float)((int32_t)roll_final_cdeg -
                                                   roll_start_cdeg));
  event->tilt_change_cdeg[2] = Clamp_Int16(yaw_accum_cdeg);

  /* Active capture masks WU/6D on INT1.  Re-arm immediately after the sample
   * window instead of leaving the wake path disabled throughout network,
   * GNSS, MQTT and modem shutdown, which can take tens of seconds.  Any new
   * IMU edge is latched by EXTI and consumed by Enter_Stop1_Mode afterwards. */
  g_imu_ok = LSM6DS_Set_Sleep_Mode();

  if (!start_modem) return 0U;

  /* MATREADY is preferred, but an AT probe is the compatibility fallback. */
  start = HAL_GetTick();
  do {
    if (ML307C_Send_CMD("AT", "OK", matready_seen ? 1000U : 500U) == 1) return 1U;
    HAL_IWDG_Refresh(&hiwdg);
    Delay_With_Service(100U);
  } while ((HAL_GetTick() - start) < 5000U);
  return 0U;
}

/** @brief Start the modem only after an IMU event has passed confirmation. */
static uint8_t Start_Modem_After_Event_Validation(void)
{
  uint32_t start;

  if (!ML307C_Is_Powered()) Turn_On_ML307C();
  else ML307C_Clear_Buffer();

  start = HAL_GetTick();
  do {
    if (ML307C_Send_CMD("AT", "OK", 500U) == 1) return 1U;
    HAL_IWDG_Refresh(&hiwdg);
    Delay_With_Service(100U);
  } while ((HAL_GetTick() - start) < 5000U);
  return 0U;
}

/**
 * @brief 执行一次从采样、低压判断、4G联网到QoS 1发布/失败落盘的完整链路。
 * @param wu_flag 硬件WAKE-UP源标志。
 * @param d6d_flag 硬件6D源标志。
 * @param rtc_flag RTC周期心跳标志。
 * @param manual 1表示维护口手动触发，不把本次事件写入Flash。
 * @param source_fallback 1表示INT1已触发但源寄存器分类丢失。
 * @return 1本次事件或队列事件至少成功发布，0未发布成功。
 * @note 无论成功或失败都会安全关闭ML307C并重新布防IMU，随后主循环可回Stop1。
 */
uint8_t Run_Event_Report(uint8_t wu_flag, uint8_t d6d_flag,
                        uint8_t rtc_flag, uint8_t manual,
                        uint8_t source_fallback)
{
  EventRecord_t event = {0};
  EventRecord_t queued;
  RTC_TimeTypeDef rtc_time = {0};
  RTC_DateTypeDef rtc_date = {0};
  ML307C_GPS_Data_t gps = {0};
  ML307C_LBS_Data_t lbs = {0};
  ML307C_Network_Status_t network = {0};
  float voltage = ADC_Get_Battery_Voltage_Avg();
  uint16_t voltage_mv = (voltage > 0.0f) ? (uint16_t)(voltage * 1000.0f + 0.5f) : 0U;
  uint8_t allow_modem = Volt_Fuse_Check(voltage);
  uint8_t store_current = (!manual && !rtc_flag && (wu_flag || d6d_flag));
  uint8_t defer_modem = store_current;
  uint8_t current_stored = 0U;
  uint8_t modem_ready;
  uint8_t mqtt_connected = 0U;
  uint8_t sent = 0U;
  uint8_t wake_report = (uint8_t)(store_current || (wu_flag || d6d_flag));
  char topic[40];
  uint32_t report_start = HAL_GetTick();

  g_last_report_ok = 0U;
  g_last_report_stage = REPORT_STAGE_CAPTURE;
  g_last_report_fail = EVENT_FAIL_NONE;
  g_last_report_csq = 99U;
  g_last_report_attached = 0U;
  g_last_report_duration_ms = 0U;

  if (voltage_mv < CRITICAL_VOLTAGE_MV) allow_modem = 0U;

  event.voltage_mv = voltage_mv;
  event.reset_reason = g_reset_reason;
  event.timestamp = RTC_Get_Context(&rtc_time, &rtc_date);
  if (rtc_date.Month >= 1U && rtc_date.Month <= 12U && rtc_date.Date >= 1U)
    event.flags |= EVENT_FLAG_TIME_VALID;
  if (wu_flag && d6d_flag) event.wake_reason = EVENT_WAKE_IMU_BOTH;
  else if (d6d_flag) event.wake_reason = EVENT_WAKE_IMU_6D;
  else if (wu_flag) event.wake_reason = EVENT_WAKE_IMU_WU;
  else if (rtc_flag) event.wake_reason = EVENT_WAKE_RTC;
  else event.wake_reason = EVENT_WAKE_MANUAL;

  /* RTC/manual reports may overlap modem boot with capture.  IMU reports are
   * confirmed first so a 6D false wake or cooldown duplicate never flashes
   * the modem without publishing anything. */
  modem_ready = Capture_Event_And_Start_Modem(
      &event, (uint8_t)(allow_modem && !defer_modem));
  if (event.sample_count == 0U) event.fail_reason = EVENT_FAIL_INTERNAL;
  /* If PB1 proved an IMU event but the latched source bits disappeared before
   * they were read, classify it from the confirmed physical result. */
  if (source_fallback) {
    if (event.flags & EVENT_FLAG_TILTED) {
      d6d_flag = 1U;
      wu_flag = (event.acc_norm_peak_mg >=
                 (uint16_t)(1000U + g_cfg.wu_mg)) ? 1U : 0U;
    } else {
      wu_flag = 1U;
      d6d_flag = 0U;
    }
    if (wu_flag && d6d_flag) event.wake_reason = EVENT_WAKE_IMU_BOTH;
    else if (d6d_flag)       event.wake_reason = EVENT_WAKE_IMU_6D;
    else                     event.wake_reason = EVENT_WAKE_IMU_WU;
  }
  /* WAKE_UP is the configured acceleration alarm. A 6D interrupt is a
   * low-power wake hint and must pass the fixed installation-angle check. */
  if (wu_flag) event.flags |= EVENT_FLAG_IMPACT;
  if (event.acc_norm_peak_mg >= (uint16_t)(1000U + g_cfg.wu_mg))
    event.flags |= EVENT_FLAG_IMPACT;
  if (store_current && d6d_flag &&
      !(event.flags & EVENT_FLAG_TILTED) &&
      (g_last_imu_class & EVENT_FLAG_TILTED)) {
    event.flags |= EVENT_FLAG_RECOVERED;
  }
  if (store_current && !wu_flag && d6d_flag &&
      !(event.flags & (EVENT_FLAG_TILTED | EVENT_FLAG_RECOVERED))) {
    if (g_imu_false_wake_count < 0xFFFFU) g_imu_false_wake_count++;
    goto report_cleanup;
  }
  event.severity = (event.flags & EVENT_FLAG_TILTED) ? 3U :
                   ((event.flags & EVENT_FLAG_IMPACT) ? 2U : 1U);

  if (store_current && (event.flags & EVENT_FLAG_TIME_VALID)) {
    uint8_t event_class = event.flags & (EVENT_FLAG_TILTED | EVENT_FLAG_IMPACT);
    if ((g_last_imu_class & EVENT_FLAG_TILTED) &&
        !(event_class & EVENT_FLAG_TILTED)) {
      event.flags |= EVENT_FLAG_RECOVERED;
    }
    if (g_last_imu_event_time != 0U &&
        event.timestamp >= g_last_imu_event_time &&
        (event.timestamp - g_last_imu_event_time) < EVENT_COOLDOWN_SEC &&
        event_class == g_last_imu_class &&
        event.wake_reason == g_last_imu_wake) {
      g_last_imu_event_time = event.timestamp;
      goto report_cleanup;
    }
    g_last_imu_event_time = event.timestamp;
    g_last_imu_class = event_class;
    g_last_imu_wake = event.wake_reason;
  }

  if (!allow_modem) {
    g_last_report_stage = REPORT_STAGE_MODEM_READY;
    event.fail_reason = EVENT_FAIL_LOW_VOLTAGE;
    if (store_current && voltage_mv >= FLASH_SAFE_VOLTAGE_MV)
      (void)EventStore_Enqueue(&event);
    goto report_cleanup;
  }

  if (defer_modem) {
    g_last_report_stage = REPORT_STAGE_MODEM_READY;
    modem_ready = Start_Modem_After_Event_Validation();
  }

  if (!modem_ready) {
    g_last_report_stage = REPORT_STAGE_MODEM_READY;
    event.fail_reason = EVENT_FAIL_MODEM_READY;
    if (store_current) (void)EventStore_Enqueue(&event);
    goto report_cleanup;
  }

  if (store_current) current_stored = EventStore_Enqueue(&event);
  if (event.event_id == 0U) {
    event.event_id = event.timestamp ? event.timestamp : g_volatile_event_seq++;
  }

  g_last_report_stage = REPORT_STAGE_IMEI;
  if (!ML307C_Has_IMEI() && !ML307C_Get_IMEI()) {
    event.fail_reason = EVENT_FAIL_INTERNAL;
    goto report_cleanup;
  }
  g_last_report_stage = REPORT_STAGE_NETWORK;
  if (!ML307C_Wait_Network(NETWORK_BUDGET_MS, &network)) {
    event.fail_reason = EVENT_FAIL_NETWORK;
    goto report_cleanup;
  }
  g_last_report_csq = (network.csq >= 0 && network.csq <= 255) ?
                       (uint8_t)network.csq : 99U;
  g_last_report_attached = network.is_attached ? 1U : 0U;

  g_last_report_stage = REPORT_STAGE_RTC;
  (void)ML307C_Sync_RTC();
  event.timestamp = RTC_Get_Context(&rtc_time, &rtc_date);
  g_last_report_stage = REPORT_STAGE_MQTT;
  if (ML307C_MQTT_Connect("101.34.217.153", 1883,
                          "solar_imu", "solar_imu") != 1) {
    event.fail_reason = EVENT_FAIL_MQTT_CONNECT;
    goto report_cleanup;
  }
  mqtt_connected = 1U;

  snprintf(topic, sizeof(topic), "device/%s/data", ML307C_Get_IMEI_Str());

  /* Flash队列是唯一的未送达标志。连接成功后先按FIFO发送所有旧记录，
   * 只有收到每条QoS 1 PUBACK后才删除对应快照。新IMU事件已先落盘，
   * 因而它位于旧记录之后并自然成为本轮最后一条。 */
  g_last_report_stage = REPORT_STAGE_QUEUE;
  while (EventStore_Count() > 0U) {
    uint8_t is_current_event;
    if (!EventStore_Get(0U, &queued)) {
      event.fail_reason = EVENT_FAIL_INTERNAL;
      sent = 0U;
      goto report_cleanup;
    }
    is_current_event = (uint8_t)(current_stored &&
                                 queued.event_id == event.event_id);
    if (!ML307C_Send_EventReport(&queued, NULL, NULL,
                                 2000 + rtc_date.Year, rtc_date.Month, rtc_date.Date,
                                 rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds,
                                 topic, (uint8_t)(is_current_event ? 0U : 1U))) {
      event.fail_reason = EVENT_FAIL_MQTT_PUBACK;
      sent = 0U;
      goto report_cleanup;
    }
    if (!EventStore_Remove(queued.event_id)) {
      event.fail_reason = EVENT_FAIL_INTERNAL;
      sent = 0U;
      goto report_cleanup;
    }
    sent = 1U;
  }

  g_last_report_stage = REPORT_STAGE_LOCATION;
  if (wake_report) {
    /* 唤醒链路：事件已即时发布，此处接入持续跟踪——初次定位最多3分钟，
     * 随后每3秒重发`AT+MGNSS=2`查询，连续三次移动量小才返回并关闭4G。 */
    (void)Track_Wake_GPS(event.event_id, event.timestamp, topic, &event);
  } else {
    if (ML307C_GPS_Start()) {
      if (!ML307C_GPS_Wait_Fix(&gps, rtc_flag ? RTC_GNSS_FIX_TIMEOUT_MS :
                                                   MANUAL_GNSS_FIX_TIMEOUT_MS)) {
        /* `AT+MGNSS=2`只会在成功后自动关闭；超时必须主动停止搜星。 */
        if (!ML307C_GPS_Stop() && gps.err_code == ML307C_LOC_ERR_UNKNOWN)
          gps.err_code = ML307C_LOC_ERR_STOP;
        else if (gps.err_code == ML307C_LOC_ERR_UNKNOWN)
          gps.err_code = ML307C_LOC_ERR_TIMEOUT;
        gps.is_fixed = 0;
        event.fail_reason = EVENT_FAIL_GNSS;
      }
    } else {
      gps.err_code = ML307C_LOC_ERR_START;
      event.fail_reason = EVENT_FAIL_GNSS;
    }
  }
#if 0 /* LBS兜底：GPS失败时改用基站定位；当前GPS-only，需要时把0改为1打开测试 */
  if (!gps.is_fixed) (void)ML307C_Get_LBS_Info(&lbs);
#endif

  /* 心跳和人工请求不会写入Flash；旧队列清空后才发送这条临时的新消息。 */
  if (!current_stored) {
    g_last_report_stage = REPORT_STAGE_PUBLISH;
    sent = ML307C_Send_EventReport(&event, &gps, &lbs,
                                   2000 + rtc_date.Year, rtc_date.Month, rtc_date.Date,
                                   rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds,
                                   topic, 0U);
    if (!sent) {
      event.fail_reason = EVENT_FAIL_MQTT_PUBACK;
      goto report_cleanup;
    }
  }

  g_last_report_stage = REPORT_STAGE_DOWNLINK;
  if (sent && rtc_flag) (void)Check_MQTT_Settings();
  else if (sent && !wake_report) (void)Check_MQTT_Downlink();
  /* 网络失败不创建额外RTC重试；未确认事件保留在Flash双缓存，下次心跳/事件时补发。 */

report_cleanup:
  g_last_report_fail = event.fail_reason;
  g_last_report_stage = sent ? REPORT_STAGE_COMPLETE : g_last_report_stage;
  g_last_report_ok = sent ? 1U : 0U;
  if (mqtt_connected) (void)ML307C_Send_CMD("AT+MQTTDISC=0", "OK", 2000);
  if (!sent && g_last_report_stage == REPORT_STAGE_DOWNLINK)
    g_last_report_stage = REPORT_STAGE_PUBLISH;
  Turn_Off_ML307C();
  g_last_report_duration_ms = HAL_GetTick() - report_start;
  (void)LSM6DS_Set_Sleep_Mode();
  return sent;
}
