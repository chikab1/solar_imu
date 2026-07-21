/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 智能路牌 低功耗固件 (DMA串口)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "lsm6ds.h"
#include "at_ml307c.h"
#include "uart_driver.h"
#include "event_store.h"
#include "service_protocol.h"
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CFG_MAGIC             0x55AA55ACU
#define CFG_DEFAULT_WU_MG     500
#define CFG_DEFAULT_TILT_DEG  30
#define CFG_DEFAULT_SLEEP_SEC 3600
#define CFG_DEFAULT_V_LOW_MV  3550
#define CFG_DEFAULT_MOUNT_AXIS MOUNT_AXIS_Z_POS
#define SERIAL_IDLE_TIMEOUT_MS 60000U
#define FLASH_SAFE_VOLTAGE_MV  3450U
#define CRITICAL_VOLTAGE_MV    3350U
#define IMU_CAPTURE_TIME_MS    3000U
#define IMU_SAMPLE_PERIOD_MS   10U
#define TILT_CONFIRM_TIME_MS   500U
#define TILT_CONFIRM_SAMPLES  (TILT_CONFIRM_TIME_MS / IMU_SAMPLE_PERIOD_MS)
#define IMU_SOURCE_SETTLE_MS   2U
#define NETWORK_BUDGET_MS      20000U
#define IWDG_STOP_GUARD_SEC    20U
#define RTC_LSI_DIV16_TICKS_PER_SEC 2000U
#define RTC_MAX_CHUNK_SEC       20U
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

SysConfig_t g_cfg;

/* 命令队列 (中断设标志, 主循环执行) */
typedef enum { CMD_NONE = 0, CMD_ON, CMD_OFF, CMD_SLEEP, CMD_VBAT, CMD_MQTT, CMD_TEST } PendingCmd_t;
static volatile PendingCmd_t g_pending_cmd = CMD_NONE;

/* 低电量熔断状态机 (回线迟滞保护)
 * 硬熔断阈值: g_cfg.v_low_mv - 禁止启动4G模组 (防止200~300mA射频脉冲砸出Brownout)
 * 回线恢复阈值: 熔断阈值+200mV - 太阳能充回安全水位后才允许重新开机
 * g_low_volt_fuse == 0: 正常 (允许开机)
 * g_low_volt_fuse == 1: 熔断 (禁止开机, 等待充电恢复) */
static uint8_t g_low_volt_fuse = 0;

/* 联网失败计数器 (硬自愈自循环机制)
 * 连续 3 次联网超时, 第 4 次开机直接执行物理硬复位模组
 * 任意一次联网成功即清零 */
static uint8_t g_net_fail_count = 0;
#define NET_FAIL_HARD_RESET_THRESHOLD  3

/* Stop1 wake source and serial maintenance window. */
static volatile uint8_t g_uart2_wakeup_flag = 0;
static volatile uint8_t g_uart2_activity_flag = 0;
static volatile uint8_t g_uart2_rearm_needed = 0;
static volatile uint8_t g_uart1_rearm_needed = 0;
static volatile uint8_t g_rtc_wakeup_flag = 0;
static volatile uint8_t g_imu_exti_wakeup_flag = 0U;
static volatile uint16_t g_imu_exti_wake_count = 0U;
static uint8_t g_imu_source_fallback_count = 0U;
static uint16_t g_imu_false_wake_count = 0U;
static uint16_t g_imu_wu_source_count = 0U;
static uint16_t g_imu_6d_source_count = 0U;
static uint16_t g_imu_both_source_count = 0U;
static volatile uint16_t g_rtc_hw_wake_count = 0U;
static volatile uint16_t g_rtc_callback_count = 0U;
static uint8_t g_rtc_arm_status = HAL_OK;
static uint8_t g_rtc_deactivate_status = HAL_OK;
/* MX_RTC_Init() arms the wakeup timer once during boot. */
static uint8_t g_rtc_timer_active = 1U;
static uint16_t g_rtc_arm_count = 0U;
static uint16_t g_rtc_last_interval = 0U;
static uint32_t g_rtc_last_cr = 0U;
static uint32_t g_rtc_last_sr = 0U;
static uint32_t g_rtc_last_requested_sleep = 0U;
static uint16_t g_rtc_consumed_count = 0U;
static uint16_t g_rtc_ready_count = 0U;
static uint8_t  g_serial_session_active = 0;
static uint32_t g_last_uart2_activity = 0;
static uint8_t g_service_busy = 0U;
static uint8_t g_service_task_running = 0U;

/* Wake information consumed by the automatic full-report command. */
static uint8_t g_report_wu = 0;
static uint8_t g_report_6d = 0;
static uint8_t g_report_rtc = 0;
static uint8_t g_report_source_fallback = 0U;
static uint8_t g_iwdg_runs_in_stop = 0;
static uint32_t g_guard_sleep_accum_sec = 0;
static uint8_t g_retry_stage = 0;
static uint32_t g_retry_delay_sec = 0;
static uint32_t g_volatile_event_seq = 1;
static uint32_t g_last_server_cmd_id = 0;
static uint8_t g_imu_ok = 0;
static uint8_t g_reset_reason = 0;
static uint32_t g_last_imu_event_time = 0U;
static uint8_t g_last_imu_class = 0U;
static uint8_t g_last_imu_wake = EVENT_WAKE_UNKNOWN;
static uint8_t g_last_report_ok = 0U;
static uint8_t g_last_report_stage = REPORT_STAGE_IDLE;
static uint8_t g_last_report_fail = EVENT_FAIL_NONE;
static uint8_t g_last_report_csq = 99U;
static uint8_t g_last_report_attached = 0U;
static uint32_t g_last_report_duration_ms = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Enter_Stop1_Mode(uint8_t *out_wu, uint8_t *out_6d,
                             uint8_t *out_rtc, uint8_t *out_uart,
                             uint8_t *out_source_fallback);
static void Config_Load(void);
static void Config_Save(void);
static void Handle_Service_Frame(const ServiceFrame_t *frame);
static void Service_Task(uint8_t busy_only);
static void Service_UART1_Recover(void);
static void Service_UART2_Recover(void);
static void Delay_With_Service(uint32_t delay_ms);
static uint8_t Run_Event_Report(uint8_t wu_flag, uint8_t d6d_flag,
                                uint8_t rtc_flag, uint8_t manual,
                                uint8_t source_fallback);
static uint8_t Capture_Event_And_Start_Modem(EventRecord_t *event,
                                             uint8_t start_modem);
static uint32_t RTC_Get_Context(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  从 RTC 备份寄存器加载系统配置
  * @note   使用 TAMP BKP0R~BKP3R (4x32bit=16字节) 存储参数.
  *         若 magic 不匹配 (首次上电/备份域丢失), 写入出厂默认值.
  *         备份寄存器在 Stop1/关机复位下均不丢失, 零磨损无限次擦写.
  */
static void Config_Load(void)
{
    uint32_t b0 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
    if (b0 != CFG_MAGIC) {
        g_cfg.magic     = CFG_MAGIC;
        g_cfg.wu_mg     = CFG_DEFAULT_WU_MG;
        g_cfg.tilt_deg  = CFG_DEFAULT_TILT_DEG;
        g_cfg.sleep_sec = CFG_DEFAULT_SLEEP_SEC;
        g_cfg.v_low_mv  = CFG_DEFAULT_V_LOW_MV;
        g_cfg.mount_axis = CFG_DEFAULT_MOUNT_AXIS;
        Config_Save();
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, 0U);
        g_last_server_cmd_id = 0U;
        return;
    }
    g_cfg.magic     = b0;
    uint32_t b1 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
    g_cfg.wu_mg     = (uint16_t)(b1 & 0xFFFF);
    g_cfg.tilt_deg  = (uint16_t)(b1 >> 16);
    g_cfg.sleep_sec = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
    uint32_t b3 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
    g_cfg.v_low_mv  = (uint16_t)(b3 & 0xFFFF);
    g_cfg.mount_axis = (uint8_t)((b3 >> 16) & 0xFFU);
    if (g_cfg.v_low_mv < 3500U || g_cfg.v_low_mv > 4000U) {
        g_cfg.v_low_mv = CFG_DEFAULT_V_LOW_MV;
        Config_Save();
    }
    if (g_cfg.mount_axis > MOUNT_AXIS_Y_NEG) {
        g_cfg.mount_axis = CFG_DEFAULT_MOUNT_AXIS;
        Config_Save();
    }
    g_last_server_cmd_id = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);
}

/**
  * @brief  将系统配置写入 RTC 备份寄存器
  */
static void Config_Save(void)
{
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, g_cfg.magic);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1,
                        ((uint32_t)g_cfg.tilt_deg << 16) | g_cfg.wu_mg);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, g_cfg.sleep_sec);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3,
                        ((uint32_t)g_cfg.mount_axis << 16) |
                        (uint32_t)g_cfg.v_low_mv);
}

/**
  * @brief  从字符串中提取 "key":value 的整数值
  * @param  buf: 字符串缓冲区
  * @param  key: 要查找的键名 (如 "wu", "tilt", "sleep", "vlow")
  * @param  out_val: 输出找到的值
  * @retval 1: 找到并解析成功, 0: 未找到
  */
static uint8_t Parse_Json_Int(const char *buf, const char *key, int *out_val)
{
    char pattern[16];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(buf, pattern);
    if (p == NULL) return 0;
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t' || *p == ':') p++;
    if (*p < '0' || *p > '9') return 0;
    *out_val = atoi(p);
    return 1;
}

/**
  * @brief  检查 MQTT 下行指令窗口 (1500ms)
  * @note   发布数据后调用. 订阅 device/IMEI/cmd 主题,
  *         等待 1500ms 捕获 +MQTTURC: "publish" URC.
  *         若收到 JSON 参数, 更新 g_cfg 并保存到备份寄存器.
  * @retval 1: 收到并应用了新配置, 0: 无下行指令
  */
static uint8_t Check_MQTT_Downlink(void)
{
    char sub_cmd[64];
    snprintf(sub_cmd, sizeof(sub_cmd),
             "AT+MQTTSUB=0,\"device/%s/cmd\",1", ML307C_Get_IMEI_Str());
    if (ML307C_Send_CMD(sub_cmd, "+MQTTURC: \"suback\"", 5000) != 1)
        return 0;

    const char *urc = ML307C_Wait_URC("+MQTTURC: \"publish\"", 1500);
    if (urc == NULL) return 0;

    int val;
    int cmd_id;
    int version;
    int expires;
    uint8_t updated = 0;
    RTC_TimeTypeDef now_time = {0};
    RTC_DateTypeDef now_date = {0};
    uint32_t now = RTC_Get_Context(&now_time, &now_date);

    if (!Parse_Json_Int(urc, "cmd_id", &cmd_id) || cmd_id <= 0 ||
        !Parse_Json_Int(urc, "ver", &version) || version != 1 ||
        !Parse_Json_Int(urc, "exp", &expires) || (uint32_t)expires < now ||
        (uint32_t)cmd_id == g_last_server_cmd_id) {
        return 0;
    }

    if (Parse_Json_Int(urc, "wu", &val) && val >= 0 && val <= 2000) {
        g_cfg.wu_mg = (uint16_t)val; updated = 1;
    }
    if (Parse_Json_Int(urc, "tilt", &val) && val >= 10 && val <= 90) {
        g_cfg.tilt_deg = (uint16_t)val; updated = 1;
    }
    if (Parse_Json_Int(urc, "sleep", &val) && val >= 10 && val <= 65535) {
        g_cfg.sleep_sec = (uint32_t)val; updated = 1;
    }
    if (Parse_Json_Int(urc, "vlow", &val) && val >= 3500 && val <= 4000) {
        g_cfg.v_low_mv = (uint16_t)val; updated = 1;
    }
    if (Parse_Json_Int(urc, "mount", &val) &&
        val >= MOUNT_AXIS_Z_POS && val <= MOUNT_AXIS_Y_NEG) {
        g_cfg.mount_axis = (uint8_t)val; updated = 1;
    }

    if (updated) {
        char ack_topic[48];
        char ack_payload[48];
        g_last_server_cmd_id = (uint32_t)cmd_id;
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, g_last_server_cmd_id);
        Config_Save();
        (void)LSM6DS_Config_Gatekeeper(g_cfg.wu_mg,
                                       (uint8_t)g_cfg.tilt_deg);
        (void)LSM6DS_Set_Sleep_Mode();
        snprintf(ack_topic, sizeof(ack_topic), "device/%s/ack",
                 ML307C_Get_IMEI_Str());
        snprintf(ack_payload, sizeof(ack_payload),
                 "{\"cmd_id\":%d,\"ok\":1}", cmd_id);
        (void)ML307C_MQTT_Publish(ack_topic, ack_payload);
        return 1;
    }
    return 0;
}

/**
  * @brief  电压过采样均值滤波 (16次ADC取均值, 抗瞬间毛刺)
  * @retval 均值电压 (V), 如 3.27f
  */
static float ADC_Get_Battery_Voltage_Avg(void)
{
    float sum = 0.0f;
    for (int i = 0; i < 16; i++)
        sum += ADC_Get_Battery_Voltage();
    return sum / 16.0f;
}

/**
  * @brief  低电量熔断检查 (三级硬熔断与回线迟滞)
  * @param  v_avg: 过采样均值电压 (V)
  * @retval 1: 允许启动4G模组, 0: 电压过低禁止启动
  * @note   熔断后即使电压回升到3.3~3.5V之间也不解除,
  *         必须充回3.5V以上才解除封印 (迟滞区间=0.2V)
  */
static uint8_t Volt_Fuse_Check(float v_avg)
{
    float cutoff = g_cfg.v_low_mv / 1000.0f;
    float hyst   = (g_cfg.v_low_mv + 200) / 1000.0f;
    if (g_low_volt_fuse == 0) {
        if (v_avg < cutoff) {
            g_low_volt_fuse = 1;
            return 0;
        }
        return 1;
    } else {
        if (v_avg >= hyst) {
            g_low_volt_fuse = 0;
            return 1;
        }
        return 0;
    }
}

#if 0 /* Legacy text command parser retained only as migration reference. */
/**
  * @brief  处理 PC 发来的指令 (整行匹配)
  */
static void Process_PC_Command(const uint8_t *data, uint16_t len)
{
  /* 去掉尾部 \r\n */
  while (len > 0 && (data[len-1] == '\r' || data[len-1] == '\n')) len--;
  if (len == 0) return;

  if (len == 3 && data[0] == '!' && data[1] == 'O' && data[2] == 'N') {
    g_pending_cmd = CMD_ON;
  }
  else if (len == 4 && data[0] == '!' && data[1] == 'O' && data[2] == 'F' && data[3] == 'F') {
    g_pending_cmd = CMD_OFF;
  }
  else if (len == 6 && data[0] == '!' && data[1] == 'S' && data[2] == 'L'
                    && data[3] == 'E' && data[4] == 'E' && data[5] == 'P') {
    g_pending_cmd = CMD_SLEEP;
  }
  else if (len == 5 && data[0] == '!' && data[1] == 'V' && data[2] == 'B'
                    && data[3] == 'A' && data[4] == 'T') {
    g_pending_cmd = CMD_VBAT;
  }
  else if (len == 5 && data[0] == '!' && data[1] == 'M' && data[2] == 'Q'
                    && data[3] == 'T' && data[4] == 'T') {
    g_pending_cmd = CMD_MQTT;
  }
  else if (len == 5 && data[0] == '!' && data[1] == 'T' && data[2] == 'E'
                    && data[3] == 'S' && data[4] == 'T') {
    g_pending_cmd = CMD_TEST;
  }
  else {
    /* 非指令 → 透传给 4G 模组 (补回被剥离的 \r\n) */
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 100);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 100);
  }
}
#endif

static int16_t Clamp_Int16(float value)
{
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return (int16_t)value;
}

static uint16_t Read_LE16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t Read_LE32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void Write_LE16(uint8_t *p, uint16_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
}

static void Write_LE32(uint8_t *p, uint32_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
  p[2] = (uint8_t)(value >> 16);
  p[3] = (uint8_t)(value >> 24);
}

static uint32_t RTC_Get_Context(RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
  static const uint8_t month_days[12] =
      {31,28,31,30,31,30,31,31,30,31,30,31};
  uint32_t days = 0U;
  uint16_t full_year;

  HAL_RTC_GetTime(&hrtc, time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, date, RTC_FORMAT_BIN);
  full_year = (uint16_t)(2000U + date->Year);

  for (uint16_t year = 2000U; year < full_year; year++) {
    days += ((year % 4U) == 0U &&
             ((year % 100U) != 0U || (year % 400U) == 0U)) ? 366U : 365U;
  }
  for (uint8_t month = 1U; month < date->Month && month <= 12U; month++) {
    days += month_days[month - 1U];
    if (month == 2U && (full_year % 4U) == 0U &&
        ((full_year % 100U) != 0U || (full_year % 400U) == 0U)) {
      days++;
    }
  }
  if (date->Date > 0U) days += (uint32_t)date->Date - 1U;
  return 946684800UL + days * 86400UL + (uint32_t)time->Hours * 3600UL +
         (uint32_t)time->Minutes * 60UL + time->Seconds;
}

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

static void Schedule_Network_Retry(uint8_t low_voltage)
{
  if (low_voltage) {
    g_retry_stage = 2U;
    g_retry_delay_sec = 21600U;
    return;
  }
  if (g_retry_stage == 0U) g_retry_delay_sec = 300U;
  else if (g_retry_stage == 1U) g_retry_delay_sec = 900U;
  else g_retry_delay_sec = 3600U;
  if (g_retry_stage < 2U) g_retry_stage++;
}

static void Clear_Network_Retry(void)
{
  g_retry_stage = 0U;
  g_retry_delay_sec = 0U;
}

static uint8_t Capture_Event_And_Start_Modem(EventRecord_t *event,
                                             uint8_t start_modem)
{
  uint32_t start;
  uint32_t next_sample;
  uint8_t pulse_active = 0U;
  uint8_t matready_seen = 0U;
  uint8_t first_sample = 1U;
  uint16_t tilt_over_samples = 0U;

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
        float norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        if (first_sample) {
          event->tilt_start_cdeg = tilt;
          event->tilt_peak_cdeg = tilt;
          first_sample = 0U;
        }
        event->tilt_final_cdeg = tilt;
        if (tilt > event->tilt_peak_cdeg) event->tilt_peak_cdeg = tilt;
        if (tilt >= (int16_t)(g_cfg.tilt_deg * 100U)) {
          if (tilt_over_samples < TILT_CONFIRM_SAMPLES) tilt_over_samples++;
          if (tilt_over_samples >= TILT_CONFIRM_SAMPLES)
            event->flags |= EVENT_FLAG_TILTED;
        } else {
          tilt_over_samples = 0U;
        }
        if (norm > event->acc_norm_peak_mg) event->acc_norm_peak_mg = (uint16_t)norm;
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

static void Handle_Service_Frame(const ServiceFrame_t *frame)
{
  uint8_t response[SERVICE_PROTOCOL_MAX_PAYLOAD] = {0};
  uint16_t response_len = 0U;
  uint8_t status = SERVICE_STATUS_OK;

  if (frame == NULL) return;
  switch (frame->command) {
  case SERVICE_CMD_GET_STATUS: {
    float voltage = ADC_Get_Battery_Voltage_Avg();
    uint16_t mv = (voltage > 0.0f) ? (uint16_t)(voltage * 1000.0f + 0.5f) : 0U;
    Write_LE16(&response[0], mv);
    response[2] = EventStore_Count();
    response[3] = ML307C_Is_Powered();
    response[4] = g_low_volt_fuse;
    response[5] = g_iwdg_runs_in_stop;
    Write_LE16(&response[6], g_cfg.v_low_mv);
    Write_LE32(&response[8], g_cfg.sleep_sec);
    response[12] = g_imu_ok;
    response[13] = g_reset_reason;
    Write_LE16(&response[14], ServiceProtocol_GetDropCount());
    Write_LE16(&response[16], ServiceProtocol_GetTimeoutCount());
    Write_LE16(&response[18], ServiceProtocol_GetUartErrorCount());
    response[20] = g_last_report_ok;
    response[21] = g_last_report_stage;
    response[22] = g_last_report_fail;
    response[23] = g_last_report_csq;
    response[24] = g_last_report_attached;
    Write_LE32(&response[25], g_last_report_duration_ms);
    response[29] = g_rtc_arm_status;
    Write_LE16(&response[30], g_rtc_arm_count);
    Write_LE16(&response[32], g_rtc_hw_wake_count);
    Write_LE16(&response[34], g_rtc_callback_count);
    Write_LE16(&response[36], g_rtc_last_interval);
    Write_LE32(&response[38], g_rtc_last_cr);
    Write_LE32(&response[42], g_rtc_last_sr);
    response[46] = g_rtc_deactivate_status;
    response[47] = g_rtc_timer_active;
    Write_LE32(&response[48], g_guard_sleep_accum_sec);
    Write_LE32(&response[52], g_rtc_last_requested_sleep);
    Write_LE16(&response[56], g_rtc_consumed_count);
    Write_LE16(&response[58], g_rtc_ready_count);
    Write_LE16(&response[60], g_imu_exti_wake_count);
    response[62] = g_imu_source_fallback_count;
    response_len = 63U;
    break;
  }
  case SERVICE_CMD_RUN_REPORT:
    if (g_pending_cmd != CMD_NONE || g_service_busy) {
      status = SERVICE_STATUS_BUSY;
    } else {
      g_report_wu = 0U;
      g_report_6d = 0U;
      g_report_rtc = 0U;
      g_pending_cmd = CMD_TEST;
    }
    break;
  case SERVICE_CMD_SET_CONFIG:
    if (frame->length != 10U) {
      status = SERVICE_STATUS_BAD_LENGTH;
    } else {
      uint16_t wu = Read_LE16(&frame->payload[0]);
      uint16_t tilt = Read_LE16(&frame->payload[2]);
      uint32_t sleep = Read_LE32(&frame->payload[4]);
      uint16_t vlow = Read_LE16(&frame->payload[8]);
      if (wu < 250U || wu > 2000U || tilt < 10U || tilt > 90U ||
          sleep < 10U || sleep > 65535U ||
          vlow < 3500U || vlow > 4000U) {
        status = SERVICE_STATUS_BAD_VALUE;
      } else {
        uint32_t old_sleep_sec = g_cfg.sleep_sec;
        g_cfg.wu_mg = wu;
        g_cfg.tilt_deg = tilt;
        g_cfg.sleep_sec = sleep;
        g_cfg.v_low_mv = vlow;
        if (sleep != old_sleep_sec) g_guard_sleep_accum_sec = 0U;
        Config_Save();
        (void)LSM6DS_Config_Gatekeeper(wu, (uint8_t)tilt);
        (void)LSM6DS_Set_Sleep_Mode();
      }
    }
    break;
  case SERVICE_CMD_READ_QUEUE: {
    uint8_t index = (frame->length > 0U) ? frame->payload[0] : 0U;
    EventRecord_t record;
    if (!EventStore_Get(index, &record)) {
      status = SERVICE_STATUS_BAD_VALUE;
    } else {
      response[0] = EventStore_Count();
      response[1] = index;
      memcpy(&response[2], &record, sizeof(record));
      response_len = (uint16_t)(2U + sizeof(record));
    }
    break;
  }
  case SERVICE_CMD_CLEAR_QUEUE: {
    float voltage = ADC_Get_Battery_Voltage_Avg();
    if (voltage * 1000.0f < FLASH_SAFE_VOLTAGE_MV || !EventStore_Clear())
      status = SERVICE_STATUS_FAILED;
    break;
  }
  case SERVICE_CMD_SLEEP:
    Turn_Off_ML307C();
    g_serial_session_active = 0U;
    /* Consume the RX-activity indication belonging to this validated SLEEP
     * frame.  Without this, a frame arriving between the main-loop activity
     * check and Service_Task() is mistaken for new traffic at the Stop1 race
     * guard, causing an immediate UART wake and a second READY response.  Any
     * byte received after this point sets the flag again and still cancels the
     * sleep transition as intended. */
    {
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      g_uart2_activity_flag = 0U;
      if (primask == 0U) __enable_irq();
    }
    break;
  case SERVICE_CMD_MODEM_ON: {
    float voltage = ADC_Get_Battery_Voltage_Avg();
    if (!Volt_Fuse_Check(voltage)) status = SERVICE_STATUS_FAILED;
    else Turn_On_ML307C();
    break;
  }
  case SERVICE_CMD_MODEM_OFF:
    Turn_Off_ML307C();
    break;
  case SERVICE_CMD_GET_IMU_DIAG:
    if (!LSM6DS_Get_Gatekeeper_Diag(response)) {
      status = SERVICE_STATUS_FAILED;
    } else {
      response[8] = g_cfg.mount_axis;
      Write_LE16(&response[9], g_cfg.tilt_deg);
      Write_LE16(&response[11], g_cfg.wu_mg);
      Write_LE16(&response[13], g_imu_false_wake_count);
      Write_LE16(&response[15], g_imu_wu_source_count);
      Write_LE16(&response[17], g_imu_6d_source_count);
      Write_LE16(&response[19], g_imu_both_source_count);
      response_len = 21U;
    }
    break;
  case SERVICE_CMD_SET_MOUNT:
    if (frame->length != 1U) {
      status = SERVICE_STATUS_BAD_LENGTH;
    } else if (frame->payload[0] > MOUNT_AXIS_Y_NEG) {
      status = SERVICE_STATUS_BAD_VALUE;
    } else {
      g_cfg.mount_axis = frame->payload[0];
      Config_Save();
    }
    break;
  default:
    status = SERVICE_STATUS_BAD_COMMAND;
    break;
  }

  ServiceProtocol_SendResponse(frame->command, frame->sequence,
                               status, response, response_len);
}

static void Service_UART2_Recover(void)
{
  if (!g_uart2_rearm_needed) return;
  HAL_UART_AbortReceive(&huart2);
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                  UART_CLEAR_NEF | UART_CLEAR_OREF |
                                  UART_CLEAR_IDLEF);
  if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, g_uart2_drv.dma_rx_buf,
                                 g_uart2_drv.dma_rx_buf_size) == HAL_OK) {
    g_uart2_rearm_needed = 0U;
  }
}

static void Service_UART1_Recover(void)
{
  if (!g_uart1_rearm_needed) return;
  if (UART1_RestartReceive()) g_uart1_rearm_needed = 0U;
}

static void Service_Task(uint8_t busy_only)
{
  ServiceFrame_t frame;
  uint8_t command, sequence, status;

  if (g_service_task_running) return;
  g_service_task_running = 1U;
  Service_UART1_Recover();
  Service_UART2_Recover();
  ServiceProtocol_Poll();

  while (ServiceProtocol_GetWakePing()) {
    g_serial_session_active = 1U;
    g_last_uart2_activity = HAL_GetTick();
    ServiceProtocol_SendWakeAck();
  }

  while (ServiceProtocol_GetFrame(&frame)) {
    g_serial_session_active = 1U;
    g_last_uart2_activity = HAL_GetTick();
    if (busy_only || g_service_busy) {
      ServiceProtocol_SendResponse(frame.command, frame.sequence,
                                   SERVICE_STATUS_BUSY, NULL, 0U);
    } else {
      Handle_Service_Frame(&frame);
    }
  }

  while (ServiceProtocol_GetError(&command, &sequence, &status)) {
    ServiceProtocol_SendResponse(command, sequence, status, NULL, 0U);
  }
  g_service_task_running = 0U;
}

void ML307C_Background_Poll(void)
{
  Service_Task(1U);
}

static void Delay_With_Service(uint32_t delay_ms)
{
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < delay_ms) {
    Service_Task(1U);
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(5U);
  }
}

static uint8_t Run_Event_Report(uint8_t wu_flag, uint8_t d6d_flag,
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
  uint8_t current_stored = 0U;
  uint8_t modem_ready;
  uint8_t mqtt_connected = 0U;
  uint8_t sent = 0U;
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

  modem_ready = Capture_Event_And_Start_Modem(&event, allow_modem);
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
    Schedule_Network_Retry(1U);
    goto report_cleanup;
  }

  if (!modem_ready) {
    g_last_report_stage = REPORT_STAGE_MODEM_READY;
    event.fail_reason = EVENT_FAIL_MODEM_READY;
    if (store_current) (void)EventStore_Enqueue(&event);
    g_net_fail_count++;
    Schedule_Network_Retry(0U);
    goto report_cleanup;
  }

  if (store_current) current_stored = EventStore_Enqueue(&event);
  if (event.event_id == 0U) {
    event.event_id = event.timestamp ? event.timestamp : g_volatile_event_seq++;
  }

  if (g_net_fail_count >= NET_FAIL_HARD_RESET_THRESHOLD) {
    g_last_report_stage = REPORT_STAGE_MODEM_READY;
    ML307C_Hard_Reset();
    g_net_fail_count = 0U;
    if (ML307C_Send_CMD("AT", "OK", 8000) != 1) {
      event.fail_reason = EVENT_FAIL_MODEM_READY;
      Schedule_Network_Retry(0U);
      goto report_cleanup;
    }
  }

  g_last_report_stage = REPORT_STAGE_IMEI;
  if (!ML307C_Has_IMEI() && !ML307C_Get_IMEI()) {
    event.fail_reason = EVENT_FAIL_INTERNAL;
    g_net_fail_count++;
    Schedule_Network_Retry(0U);
    goto report_cleanup;
  }
  g_last_report_stage = REPORT_STAGE_NETWORK;
  if (!ML307C_Wait_Network(NETWORK_BUDGET_MS, &network)) {
    event.fail_reason = EVENT_FAIL_NETWORK;
    g_net_fail_count++;
    Schedule_Network_Retry(0U);
    goto report_cleanup;
  }
  g_net_fail_count = 0U;
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
    Schedule_Network_Retry(0U);
    goto report_cleanup;
  }
  mqtt_connected = 1U;

  g_last_report_stage = REPORT_STAGE_LOCATION;
  (void)ML307C_GPS_Start();
  Delay_With_Service(2000U);
  if (ML307C_Send_CMD("AT+CGNSINF", "+CGNSINF:", 3000) == 1)
    (void)ML307C_GPS_Parse(ML307C_Get_RxBuffer(), &gps);
  if (!gps.is_fixed) (void)ML307C_Get_LBS_Info(&lbs);

  g_last_report_stage = REPORT_STAGE_PUBLISH;
  snprintf(topic, sizeof(topic), "device/%s/data", ML307C_Get_IMEI_Str());
  sent = ML307C_Send_EventReport(&event, &gps, &lbs,
                                 2000 + rtc_date.Year, rtc_date.Month, rtc_date.Date,
                                 rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds,
                                 topic, 0U);
  if (!sent) event.fail_reason = EVENT_FAIL_MQTT_PUBACK;
  if (sent && current_stored) (void)EventStore_Remove(event.event_id);

  if (sent && EventStore_Count() > 0U && EventStore_Get(0U, &queued) &&
      queued.event_id != event.event_id) {
    g_last_report_stage = REPORT_STAGE_QUEUE;
    if (queued.retry_count < 0xFFU) queued.retry_count++;
    if (ML307C_Send_EventReport(&queued, &gps, &lbs,
                                2000 + rtc_date.Year, rtc_date.Month, rtc_date.Date,
                                rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds,
                                topic, 1U)) {
      (void)EventStore_Remove(queued.event_id);
    } else {
      event.fail_reason = EVENT_FAIL_MQTT_PUBACK;
      sent = 0U;
    }
  }

  g_last_report_stage = REPORT_STAGE_DOWNLINK;
  (void)Check_MQTT_Downlink();
  if (sent) {
    Clear_Network_Retry();
    if (EventStore_Count() > 0U) Schedule_Network_Retry(0U);
  } else {
    Schedule_Network_Retry(0U);
  }

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  __HAL_DBGMCU_FREEZE_IWDG();

  Config_Load();

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PWRRST))  g_reset_reason |= 0x01U;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))  g_reset_reason |= 0x02U;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  g_reset_reason |= 0x04U;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) g_reset_reason |= 0x08U;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) g_reset_reason |= 0x10U;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) g_reset_reason |= 0x20U;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* IMU + 门卫 */
  if (LSM6DS_Init(&hi2c2) == 1) {
    g_imu_ok = LSM6DS_Config_Gatekeeper(g_cfg.wu_mg,
                                         (uint8_t)g_cfg.tilt_deg);
  }

  HAL_GPIO_WritePin(LTE_PWRKEY_GPIO_Port, LTE_PWRKEY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LTE_RESET_GPIO_Port,  LTE_RESET_Pin,  GPIO_PIN_RESET);

  UART_Driver_Init();

  ServiceProtocol_Init();
  (void)EventStore_Init();
  (void)LSM6DS_Set_Sleep_Mode();
  g_iwdg_runs_in_stop = ((FLASH->OPTR & FLASH_OPTR_IWDG_STOP) != 0U) ? 1U : 0U;

  /* Announce every MCU boot once. Run_Event_Report() still applies the battery
   * fuse, so an undervoltage boot never powers the cellular modem. */
  g_report_wu = 0U;
  g_report_6d = 0U;
  g_report_rtc = 0U;
  g_pending_cmd = CMD_TEST;

#if 0 /* Legacy text diagnostics and boot-time modem test: disabled for production. */
  {
    Turn_On_ML307C();
    uint32_t to = HAL_GetTick();
    int ok = 0;
    while (HAL_GetTick() - to < 8000) {
      HAL_IWDG_Refresh(&hiwdg);
      if (ML307C_Send_CMD("AT", "OK", 500) == 1) { ok = 1; break; }
      HAL_Delay(200);
    }
    if (ok) {
      ML307C_Drain_Rx(500);
      if (ML307C_Get_IMEI() == 1) {
        char imei_msg[48];
        int n = snprintf(imei_msg, sizeof(imei_msg), "[SYS] IMEI: %s\r\n", ML307C_Get_IMEI_Str());
        HAL_UART_Transmit(&huart2, (uint8_t*)imei_msg, n, 100);
      } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] IMEI FAIL\r\n", 17, 100);
      }
    } else {
      HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] 4G PWR FAIL\r\n", 19, 100);
    }
    Turn_Off_ML307C();
  }

  {
    char cfg_msg[80];
    int cn = snprintf(cfg_msg, sizeof(cfg_msg),
                      "[SYS] wu=%dmg tilt=%d sleep=%ds vlow=%dmV\r\n",
                      g_cfg.wu_mg, g_cfg.tilt_deg,
                      (int)g_cfg.sleep_sec, g_cfg.v_low_mv);
    HAL_UART_Transmit(&huart2, (uint8_t*)cfg_msg, cn, 100);
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] Ready\r\n", 13, 100);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#if 0 /* Replaced by the bounded production workflow below. */

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
    }

    /* Any USART2 traffic opens (or extends) the 60-second maintenance window. */
    if (g_uart2_activity_flag) {
      __disable_irq();
      g_uart2_activity_flag = 0;
      __enable_irq();
      g_serial_session_active = 1;
      g_last_uart2_activity = HAL_GetTick();
    }

    switch (g_sys_state)
    {
    case STATE_IDLE:
      /* 执行来自中断回调的待处理命令 (主循环上下文, 可安全调用阻塞函数) */
      if (g_pending_cmd != CMD_NONE) {
        PendingCmd_t cmd = g_pending_cmd;
        g_pending_cmd = CMD_NONE;

        switch (cmd) {
        case CMD_ON: {
          float v_avg = ADC_Get_Battery_Voltage_Avg();
          if (!Volt_Fuse_Check(v_avg)) {
            char fuse_msg[48];
            int vm = (int)(v_avg * 1000.0f + 0.5f);
            int fn = snprintf(fuse_msg, sizeof(fuse_msg),
                             "\r\n[FUSE] V=%d.%03d skip ON\r\n",
                             vm / 1000, vm % 1000);
            HAL_UART_Transmit(&huart2, (uint8_t*)fuse_msg, fn, 100);
            break;
          }
          HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[CMD] !ON\r\n", 14, 100);
          Turn_On_ML307C();
          break;
        }
        case CMD_OFF:
          HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[CMD] !OFF\r\n", 15, 100);
          Turn_Off_ML307C();
          break;
        case CMD_SLEEP:
          HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[CMD] !SLEEP\r\n", 16, 100);
          Turn_Off_ML307C();
          g_serial_session_active = 0;
          break;
        case CMD_VBAT: {
          float vbat = ADC_Get_Battery_Voltage();
          uint8_t pct = ADC_Battery_Voltage_To_Percentage(vbat);
          char buf[64]; int n;
          if (vbat < 0.0f) {
            n = snprintf(buf, sizeof(buf), "\r\n[VBAT] ADC read failed!\r\n");
          } else {
            int v_mv = (int)(vbat * 1000.0f + 0.5f);
            n = snprintf(buf, sizeof(buf), "\r\n[VBAT] %d.%03dV  %d%%\r\n",
                         v_mv / 1000, v_mv % 1000, (int)pct);
          }
          HAL_UART_Transmit(&huart2, (uint8_t *)buf, n, 100);
          break;
        }
        case CMD_MQTT: {
          uint32_t t_start, t_pwr = 0, t_net = 0, t_conn = 0, t_pub = 0;
          ML307C_Network_Status_t ns = {0};
          char log[200]; int n;

          {
            float v_avg = ADC_Get_Battery_Voltage_Avg();
            if (!Volt_Fuse_Check(v_avg)) {
              int vm = (int)(v_avg * 1000.0f + 0.5f);
              n = snprintf(log, sizeof(log),
                           "\r\n[FUSE] V=%d.%03d skip 4G\r\n",
                           vm / 1000, vm % 1000);
              HAL_UART_Transmit(&huart2, (uint8_t*)log, n, 100);
              break;
            }
          }

          HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[MQTT] start\r\n", 16, 100);

          t_start = HAL_GetTick();

          if (g_net_fail_count >= NET_FAIL_HARD_RESET_THRESHOLD) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] Hard Reset!\r\n", 19, 100);
            ML307C_Hard_Reset();
            g_net_fail_count = 0;
          }

          Turn_On_ML307C();
          {
            uint32_t to = HAL_GetTick();
            while (HAL_GetTick() - to < 8000) {
              HAL_IWDG_Refresh(&hiwdg);
              if (ML307C_Send_CMD("AT", "OK", 200) == 1) { t_pwr = HAL_GetTick(); break; }
              HAL_Delay(200);
            }
          }
          if (!t_pwr) { HAL_UART_Transmit(&huart2, (uint8_t*)"[MQTT] PWR TO\r\n", 15, 100); goto mqtt_end; }

          ML307C_Network_Init(&ns);
          if (ns.csq == 99) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"[ERR] CSQ=99! Meltdown.\r\n", 24, 100);
            g_net_fail_count++;
            goto mqtt_end;
          }
          {
            uint32_t to = HAL_GetTick();
            while (HAL_GetTick() - to < 15000) {
              HAL_IWDG_Refresh(&hiwdg);
              if (ML307C_Send_CMD("AT+CGATT?", "+CGATT: 1", 500) == 1) { t_net = HAL_GetTick(); break; }
              HAL_Delay(500);
            }
          }
          if (!t_net) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"[MQTT] NET TO\r\n", 15, 100);
            g_net_fail_count++;
            goto mqtt_end;
          }
          g_net_fail_count = 0;

          if (ML307C_MQTT_Connect("101.34.217.153", 1883, "solar_imu", "solar_imu") == 1) {
            t_conn = HAL_GetTick();
          } else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"[MQTT] MQTT fail!\r\n", 19, 100);
            goto mqtt_end;
          }

          {
            float v = ADC_Get_Battery_Voltage();
            float tilt = LSM6DS_Get_Tilt_Angle();
            int v100 = (int)(v * 100.0f + 0.5f);
            int t100 = (int)(tilt * 100.0f + 0.5f);
            float acc_mg[3] = {0};
            LSM6DS_Read_Storage(acc_mg, (float[3]){0});
            {
              char dyn_topic[40];
              snprintf(dyn_topic, sizeof(dyn_topic), "device/%s/data", ML307C_Get_IMEI_Str());
              if (ML307C_Send_CustomData((int16_t)v100, (int16_t)t100,
                                         (int16_t)(acc_mg[0] + 0.5f),
                                         (int16_t)(acc_mg[1] + 0.5f),
                                         (int16_t)(acc_mg[2] + 0.5f),
                                         dyn_topic) == 1) {
                t_pub = HAL_GetTick();
              } else {
                HAL_UART_Transmit(&huart2, (uint8_t*)"[MQTT] Pub fail!\r\n", 18, 100);
                goto mqtt_end;
              }
            }
          }

          Check_MQTT_Downlink();

          n = snprintf(log, sizeof(log),
                       "\r\n--- REPORT ---\r\n"
                       "PWR:%d NET:%d MQTT:%d PUB:%d\r\n"
                       "TOT:%d ms\r\n",
                       (int)(t_pwr - t_start), (int)(t_net - t_pwr),
                       (int)(t_conn - t_net), (int)(t_pub - t_conn),
                       (int)(t_pub - t_start));
          HAL_UART_Transmit(&huart2, (uint8_t *)log, n, 200);

        mqtt_end:
          Turn_Off_ML307C();
          break;
        }
        case CMD_TEST: {
          uint8_t wu_flag = g_report_wu;
          uint8_t d6d_flag = g_report_6d;
          uint8_t rtc_flag = g_report_rtc;
          g_report_wu = 0;
          g_report_6d = 0;
          g_report_rtc = 0;

          HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[REPORT] Start\r\n", 18, 100);

          /* ===== 醒来: 采集传感器 (时间等联网后再读) ===== */
          float v = ADC_Get_Battery_Voltage_Avg();
          float tilt = LSM6DS_Get_Tilt_Angle();
          float acc_mg[3] = {0}, gyro_dps[3] = {0};
          LSM6DS_Read_Storage(acc_mg, gyro_dps);

          int v100 = (int)(v * 100.0f + 0.5f);
          int t100 = (int)(tilt * 100.0f + 0.5f);

          const char *wake_src = "UNK";
          if (wu_flag && d6d_flag) wake_src = "BOTH";
          else if (d6d_flag)       wake_src = "6D";
          else if (wu_flag)        wake_src = "WU";
          else if (rtc_flag)       wake_src = "RTC";
          else                     wake_src = "MANUAL";

          {
            char info[64];
            int n = snprintf(info, sizeof(info),
                             "[T] V=%d.%02d T=%d.%02d W=%s\r\n",
                             v100 / 100, (v100 >= 0 ? v100 : -v100) % 100,
                             t100 / 100, (t100 >= 0 ? t100 : -t100) % 100,
                             wake_src);
            HAL_UART_Transmit(&huart2, (uint8_t*)info, n, 100);
          }

          /* ===== 低电量熔断检查: 电压不足则禁止启动4G模组 ===== */
          if (!Volt_Fuse_Check(v)) {
            char fuse_msg[48];
            int fn = snprintf(fuse_msg, sizeof(fuse_msg),
                              "[FUSE] V=%d.%02d<%d skip\r\n",
                              v100 / 100, (v100 >= 0 ? v100 : -v100) % 100,
                              g_cfg.v_low_mv);
            HAL_UART_Transmit(&huart2, (uint8_t*)fuse_msg, fn, 100);
            goto test_end;
          }

          /* ===== 开机 → 网络 → 同步RTC → MQTT → 全量上报 ===== */
          if (g_net_fail_count >= NET_FAIL_HARD_RESET_THRESHOLD) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] Hard Reset!\r\n", 19, 100);
            ML307C_Hard_Reset();
            g_net_fail_count = 0;
          }

          Turn_On_ML307C();
          {
            uint32_t to = HAL_GetTick();
            int ok = 0;
            while (HAL_GetTick() - to < 8000) {
              HAL_IWDG_Refresh(&hiwdg);
              if (ML307C_Send_CMD("AT", "OK", 200) == 1) { ok = 1; break; }
              HAL_Delay(200);
            }
            if (!ok) { HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] PWR TO\r\n", 15, 100); goto test_end; }
          }

          {
            ML307C_Network_Status_t ns = {0};
            ML307C_Network_Init(&ns);
            if (ns.csq == 99) {
              HAL_UART_Transmit(&huart2, (uint8_t*)"[ERR] CSQ=99! Meltdown.\r\n", 24, 100);
              g_net_fail_count++;
              goto test_end;
            }
            uint32_t to = HAL_GetTick();
            int ok = 0;
            while (HAL_GetTick() - to < 15000) {
              HAL_IWDG_Refresh(&hiwdg);
              if (ML307C_Send_CMD("AT+CGATT?", "+CGATT: 1", 500) == 1) { ok = 1; break; }
              HAL_Delay(500);
            }
            if (!ok) {
              HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] NET TO\r\n", 15, 100);
              g_net_fail_count++;
              goto test_end;
            }
            g_net_fail_count = 0;
          }

          if (ML307C_Sync_RTC() == 1) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] RTC OK\r\n", 15, 100);
          } else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] RTC fail\r\n", 17, 100);
          }

          {
            RTC_TimeTypeDef rtc_time = {0};
            RTC_DateTypeDef rtc_date = {0};
            HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
            HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
            int yr = 2000 + (int)rtc_date.Year;
            int mo = (int)rtc_date.Month;
            int dy = (int)rtc_date.Date;
            int hr = (int)rtc_time.Hours;
            int mi = (int)rtc_time.Minutes;
            int sc = (int)rtc_time.Seconds;

            char tbuf[48];
            int n = snprintf(tbuf, sizeof(tbuf), "[T] %04d-%02d-%02d %02d:%02d:%02d\r\n",
                             yr, mo, dy, hr, mi, sc);
            HAL_UART_Transmit(&huart2, (uint8_t*)tbuf, n, 100);

            if (ML307C_MQTT_Connect("101.34.217.153", 1883, "solar_imu", "solar_imu") != 1) {
              HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] MQTT fail!\r\n", 19, 100);
              goto test_end;
            }
            HAL_IWDG_Refresh(&hiwdg);

            ML307C_GPS_Start();
            HAL_Delay(2000);
            ML307C_GPS_Data_t gps = {0};
            ML307C_LBS_Data_t lbs = {0};
            {
              ML307C_Send_CMD("AT+CGNSINF", "+CGNSINF:", 3000);
              ML307C_GPS_Parse(ML307C_Get_RxBuffer(), &gps);
            }

            if (!gps.is_fixed) {
              HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] no GPS,LBS\r\n", 19, 100);
              if (ML307C_Get_LBS_Info(&lbs) == 1) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] LBS OK\r\n", 15, 100);
              } else {
                HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] LBS fail\r\n", 17, 100);
              }
            }

            {
              char dyn_topic[40];
              snprintf(dyn_topic, sizeof(dyn_topic), "device/%s/data", ML307C_Get_IMEI_Str());
              if (ML307C_Send_FullReport((int16_t)v100, (int16_t)t100,
                                         acc_mg, gyro_dps, &gps, &lbs,
                                         yr, mo, dy, hr, mi, sc,
                                         wake_src, dyn_topic) == 1) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] Sent\r\n", 13, 100);
              } else {
                HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] Pub fail!\r\n", 18, 100);
              }
            }

            Check_MQTT_Downlink();
          }

        test_end:
          Turn_Off_ML307C();
          break;
        }
        default: break;
        }

        /* Keep a full maintenance minute after a command has completed. */
        if (g_serial_session_active) {
          g_last_uart2_activity = HAL_GetTick();
        }
      }


      if (g_serial_session_active &&
          (HAL_GetTick() - g_last_uart2_activity >= SERIAL_IDLE_TIMEOUT_MS) &&
          g_pending_cmd == CMD_NONE) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] UART idle, sleep\r\n", 24, 100);
        Turn_Off_ML307C();
        g_serial_session_active = 0;
      }

      /* No serial maintenance session: sleep immediately and dispatch the
       * complete report flow after an RTC/IMU wake. */
      if (!g_serial_session_active && g_pending_cmd == CMD_NONE) {
        uint8_t wu_flag = 0, d6d_flag = 0, rtc_flag = 0, uart_flag = 0;
        uint8_t source_fallback = 0U;

        HAL_Delay(2);
        Enter_Stop1_Mode(&wu_flag, &d6d_flag, &rtc_flag, &uart_flag,
                         &source_fallback);

        if (uart_flag) {
          g_serial_session_active = 1;
          g_last_uart2_activity = HAL_GetTick();
          HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] UART WAKE READY\r\n", 23, 100);
        }

        if (wu_flag || d6d_flag || rtc_flag) {
          g_report_wu = wu_flag;
          g_report_6d = d6d_flag;
          g_report_rtc = rtc_flag;
          g_report_source_fallback = source_fallback;
          g_pending_cmd = CMD_TEST;
        }
      }

      HAL_IWDG_Refresh(&hiwdg);
      break;

    case STATE_SLEEP:
      HAL_IWDG_Refresh(&hiwdg);
      g_sys_state = STATE_IDLE;
      break;
    }
#endif

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
      g_uart1_rearm_needed = 1U;
    }

    if (g_uart2_activity_flag) {
      __disable_irq();
      g_uart2_activity_flag = 0U;
      __enable_irq();
      g_serial_session_active = 1U;
      g_last_uart2_activity = HAL_GetTick();
    }

    Service_Task(0U);

    if (g_pending_cmd == CMD_TEST) {
      uint8_t wu = g_report_wu;
      uint8_t d6d = g_report_6d;
      uint8_t rtc = g_report_rtc;
      uint8_t source_fallback = g_report_source_fallback;
      g_report_wu = 0U;
      g_report_6d = 0U;
      g_report_rtc = 0U;
      g_report_source_fallback = 0U;
      g_pending_cmd = CMD_NONE;
      g_service_busy = 1U;
      (void)Run_Event_Report(wu, d6d, rtc,
                            (uint8_t)(!wu && !d6d && !rtc),
                            source_fallback);
      g_service_busy = 0U;
      if (g_serial_session_active) g_last_uart2_activity = HAL_GetTick();
    }

    if (g_serial_session_active &&
        (HAL_GetTick() - g_last_uart2_activity) >= SERIAL_IDLE_TIMEOUT_MS &&
        g_pending_cmd == CMD_NONE) {
      Turn_Off_ML307C();
      g_serial_session_active = 0U;
    }

    if (!g_serial_session_active && g_pending_cmd == CMD_NONE) {
      uint8_t wu = 0U, d6d = 0U, rtc = 0U, uart = 0U, fallback = 0U;
      Enter_Stop1_Mode(&wu, &d6d, &rtc, &uart, &fallback);
      if (uart) {
        g_serial_session_active = 1U;
        g_last_uart2_activity = HAL_GetTick();
        ServiceProtocol_SendWakeAck();
      }
      if (wu || d6d || rtc) {
        g_report_wu = wu;
        g_report_6d = d6d;
        g_report_rtc = rtc;
        g_report_source_fallback = fallback;
        g_pending_cmd = CMD_TEST;
      }
    }

    HAL_IWDG_Refresh(&hiwdg);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2) {
    if (Size > 0) {
      g_uart2_activity_flag = 1;
      ServiceProtocol_Feed(g_uart2_drv.dma_rx_buf, Size);
    }
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, g_uart2_drv.dma_rx_buf,
                                   g_uart2_drv.dma_rx_buf_size) != HAL_OK) {
      g_uart2_rearm_needed = 1U;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    g_uart1_rearm_needed = 1U;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                UART_CLEAR_NEF | UART_CLEAR_OREF |
                                UART_CLEAR_IDLEF);
  } else if (huart->Instance == USART2) {
    ServiceProtocol_NotifyUartError();
    g_uart2_activity_flag = 1U;
    g_uart2_rearm_needed = 1U;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                UART_CLEAR_NEF | UART_CLEAR_OREF |
                                UART_CLEAR_IDLEF);
  }
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc_cb)
{
  if (hrtc_cb->Instance == RTC) {
    g_rtc_wakeup_flag = 1;
    if (g_rtc_callback_count < 0xFFFFU) g_rtc_callback_count++;
  }
}

/* Release the IMU's latched push-pull INT1 before arming another rising-edge
 * Stop1 wake. Reading the source registers is what makes the sensor drive the
 * pin low; clearing the STM32 EXTI flag alone cannot release the pin. */
static uint8_t IMU_Drain_INT1_Latch(uint8_t *out_wu, uint8_t *out_6d)
{
  uint8_t wu = 0U, d6d = 0U;

  if (out_wu != NULL) *out_wu = 0U;
  if (out_6d != NULL) *out_6d = 0U;

  for (uint8_t attempt = 0U; attempt < 5U; attempt++) {
    wu = 0U;
    d6d = 0U;
    LSM6DS_Clear_All_Interrupts_Ex(&wu, &d6d);
    if (out_wu != NULL) *out_wu |= wu;
    if (out_6d != NULL) *out_6d |= d6d;
    HAL_Delay(2U);
    if (HAL_GPIO_ReadPin(IMU_INT1_WAKEUP_GPIO_Port,
                         IMU_INT1_WAKEUP_Pin) == GPIO_PIN_RESET) return 1U;
  }

  /* Abnormal recovery: masking the two INT1 routes forces the shared output
   * low. Re-entering sleep mode clears all sources before restoring WU+6D. */
  (void)LSM6DS_Set_Active_Mode();
  HAL_Delay(2U);
  wu = 0U;
  d6d = 0U;
  LSM6DS_Clear_All_Interrupts_Ex(&wu, &d6d);
  if (out_wu != NULL) *out_wu |= wu;
  if (out_6d != NULL) *out_6d |= d6d;
  (void)LSM6DS_Set_Sleep_Mode();

  for (uint8_t attempt = 0U; attempt < 5U; attempt++) {
    wu = 0U;
    d6d = 0U;
    LSM6DS_Clear_All_Interrupts_Ex(&wu, &d6d);
    if (out_wu != NULL) *out_wu |= wu;
    if (out_6d != NULL) *out_6d |= d6d;
    HAL_Delay(2U);
    if (HAL_GPIO_ReadPin(IMU_INT1_WAKEUP_GPIO_Port,
                         IMU_INT1_WAKEUP_Pin) == GPIO_PIN_RESET) return 1U;
  }
  return 0U;
}

static void Enter_Stop1_Mode(uint8_t *out_wu, uint8_t *out_6d,
                             uint8_t *out_rtc, uint8_t *out_uart,
                             uint8_t *out_source_fallback)
{
  GPIO_InitTypeDef g = {0};
  uint32_t requested_sleep;
  uint32_t rtc_interval;
  uint8_t entered_stop = 0U;
  uint8_t rtc_event = 0U;
  uint8_t pre_wu = 0U, pre_6d = 0U;
  uint8_t imu_exti_event = 0U;
  uint8_t source_fallback = 0U;

  if (out_wu)   *out_wu = 0;
  if (out_6d)   *out_6d = 0;
  if (out_rtc)  *out_rtc = 0;
  if (out_uart) *out_uart = 0;
  if (out_source_fallback) *out_source_fallback = 0U;

  g_uart2_wakeup_flag = 0;

  /* STM32G031 USART2 cannot generate a Stop1 wake event. Temporarily use
   * USART2_RX (PA3) as a falling-edge EXTI input. 0x00 is the dedicated,
   * sacrificial wake token; a command frame is sent only after WAKE READY. */
  ServiceProtocol_ResetReceiver();
  HAL_UART_AbortReceive(&huart2);
  g.Pin = GPIO_PIN_3;
  g.Mode = GPIO_MODE_IT_FALLING;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &g);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
  HAL_NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  UART1_StopReceive();
  g.Pin  = GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &g);

  (void)LSM6DS_Set_Sleep_Mode();
  /* Preserve an event that arrived after the previous report but before WFI;
   * it is a real event, not merely a stale latch to discard. */
  (void)IMU_Drain_INT1_Latch(&pre_wu, &pre_6d);
  __disable_irq();
  __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT1_WAKEUP_Pin);
  HAL_NVIC_ClearPendingIRQ(IMU_INT1_WAKEUP_EXTI_IRQn);
  g_imu_exti_wakeup_flag =
      (uint8_t)((pre_wu || pre_6d ||
          HAL_GPIO_ReadPin(IMU_INT1_WAKEUP_GPIO_Port,
                           IMU_INT1_WAKEUP_Pin) == GPIO_PIN_SET) ? 1U : 0U);
  __enable_irq();
  requested_sleep = g_cfg.sleep_sec;
  if (g_retry_delay_sec > 0U) {
    if (g_low_volt_fuse) requested_sleep = g_retry_delay_sec;
    else if (g_retry_delay_sec < requested_sleep) requested_sleep = g_retry_delay_sec;
  }
  if (requested_sleep < 1U) requested_sleep = 1U;
  if (requested_sleep > 65535U) requested_sleep = 65535U;
  g_rtc_last_requested_sleep = requested_sleep;

  for (;;) {
    uint32_t remaining_sleep = requested_sleep - g_guard_sleep_accum_sec;
    rtc_event = 0U;
    entered_stop = 0U;
    rtc_interval = remaining_sleep;
    if (rtc_interval > RTC_MAX_CHUNK_SEC)
      rtc_interval = RTC_MAX_CHUNK_SEC;
    if (g_iwdg_runs_in_stop && rtc_interval > IWDG_STOP_GUARD_SEC)
      rtc_interval = IWDG_STOP_GUARD_SEC;

    if (g_rtc_timer_active) {
      g_rtc_deactivate_status =
          (uint8_t)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
      if (g_rtc_deactivate_status == HAL_OK) g_rtc_timer_active = 0U;
    }

    g_rtc_last_interval = (uint16_t)rtc_interval;
    if (g_rtc_arm_count < 0xFFFFU) g_rtc_arm_count++;
    g_rtc_arm_status = (uint8_t)HAL_RTCEx_SetWakeUpTimer_IT(
        &hrtc, rtc_interval * RTC_LSI_DIV16_TICKS_PER_SEC - 1U,
        RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    g_rtc_timer_active = (g_rtc_arm_status == HAL_OK) ? 1U : 0U;
    if (g_rtc_arm_status != HAL_OK) {
      g_rtc_deactivate_status =
          (uint8_t)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
      g_rtc_timer_active = 0U;
      g_rtc_arm_status = (uint8_t)HAL_RTCEx_SetWakeUpTimer_IT(
          &hrtc, rtc_interval * RTC_LSI_DIV16_TICKS_PER_SEC - 1U,
          RTC_WAKEUPCLOCK_RTCCLK_DIV16);
      g_rtc_timer_active = (g_rtc_arm_status == HAL_OK) ? 1U : 0U;
    }
    g_rtc_last_cr = RTC->CR;
    g_rtc_last_sr = RTC->SR;

    /* A previous RTC IRQ can leave an NVIC pending bit after the timer has
     * been reprogrammed. Clear it before WFI so it cannot cause a false wake. */
    HAL_NVIC_ClearPendingIRQ(RTC_TAMP_IRQn);
    __disable_irq();
    g_rtc_wakeup_flag = 0U;
    __enable_irq();
    HAL_IWDG_Refresh(&hiwdg);
    HAL_SuspendTick();

    /* Close the last race before WFI. IRQs remain masked only across WFI. */
    __disable_irq();
    if (g_rtc_arm_status != HAL_OK) {
      rtc_event = 1U;
    } else if (g_uart2_wakeup_flag || g_uart2_activity_flag ||
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET ||
        __HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != 0U) {
      g_uart2_wakeup_flag = 1U;
    } else if (g_imu_exti_wakeup_flag ||
        HAL_GPIO_ReadPin(IMU_INT1_WAKEUP_GPIO_Port,
                         IMU_INT1_WAKEUP_Pin) == GPIO_PIN_SET ||
        __HAL_GPIO_EXTI_GET_IT(IMU_INT1_WAKEUP_Pin) != 0U) {
      g_imu_exti_wakeup_flag = 1U;
    } else {
      entered_stop = 1U;
      HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    }

    if (__HAL_RTC_WAKEUPTIMER_GET_FLAG(&hrtc, RTC_FLAG_WUTF) != 0U) {
      rtc_event = 1U;
      if (g_rtc_hw_wake_count < 0xFFFFU) g_rtc_hw_wake_count++;
    }

    HAL_IWDG_Refresh(&hiwdg);
    if (entered_stop &&
        __HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK) {
      SystemClock_Config();
    }
    HAL_ResumeTick();
    __enable_irq();

    if (g_rtc_wakeup_flag) rtc_event = 1U;
    g_rtc_wakeup_flag = 0U;
    if (!rtc_event) {
      g_guard_sleep_accum_sec = 0U;
      break;
    }

    /* A simultaneous service/IMU interrupt takes priority over the periodic
     * guard wake. The heartbeat interval restarts after that real event. */
    if (g_uart2_wakeup_flag || g_uart2_activity_flag ||
        g_imu_exti_wakeup_flag ||
        HAL_GPIO_ReadPin(IMU_INT1_WAKEUP_GPIO_Port,
                         IMU_INT1_WAKEUP_Pin) == GPIO_PIN_SET ||
        __HAL_GPIO_EXTI_GET_IT(IMU_INT1_WAKEUP_Pin) != 0U) {
      rtc_event = 0U;
      g_guard_sleep_accum_sec = 0U;
      break;
    }

    if (g_rtc_consumed_count < 0xFFFFU) g_rtc_consumed_count++;
    if (g_rtc_arm_status != HAL_OK) {
      g_guard_sleep_accum_sec = 0U;
      if (g_rtc_ready_count < 0xFFFFU) g_rtc_ready_count++;
      break;
    }
    g_guard_sleep_accum_sec += rtc_interval;
    if (g_guard_sleep_accum_sec >= requested_sleep) {
      g_guard_sleep_accum_sec = 0U;
      if (g_rtc_ready_count < 0xFFFFU) g_rtc_ready_count++;
      break;
    }
    /* Intermediate watchdog guard wake: refresh and sleep the next chunk
     * without returning through the main state machine. */
  }

  /* Resume directly after WFI. SRAM and peripheral registers are retained. */
  HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
  HAL_NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
  /* Restore USART1 pins and the USART2 RX alternate function. Peripheral
   * configuration and I2C state stay intact; only USART2 reception restarts. */
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF0_USART1;
  HAL_GPIO_Init(GPIOB, &g);
  if (!UART1_RestartReceive()) g_uart1_rearm_needed = 1U;

  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
  g.Pin = GPIO_PIN_3;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &g);

  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                  UART_CLEAR_NEF | UART_CLEAR_OREF |
                                  UART_CLEAR_IDLEF);
  __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
  ServiceProtocol_ResetReceiver();
  if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, g_uart2_drv.dma_rx_buf,
                                 g_uart2_drv.dma_rx_buf_size) != HAL_OK) {
    g_uart2_rearm_needed = 1U;
  } else {
    g_uart2_rearm_needed = 0U;
  }

  /* INT1 is the OR of WAKE_UP and 6D. Leave enough time for the 52 Hz
   * embedded source status to settle before reading and clearing each latch. */
  if (g_imu_exti_wakeup_flag ||
      HAL_GPIO_ReadPin(IMU_INT1_WAKEUP_GPIO_Port,
                       IMU_INT1_WAKEUP_Pin) == GPIO_PIN_SET ||
      __HAL_GPIO_EXTI_GET_IT(IMU_INT1_WAKEUP_Pin) != 0U) {
    HAL_Delay(IMU_SOURCE_SETTLE_MS);
  }

  uint8_t wu_now = 0U, d6d_now = 0U;
  uint8_t wu_flag = pre_wu, d6d_flag = pre_6d;
  (void)IMU_Drain_INT1_Latch(&wu_now, &d6d_now);
  wu_flag |= wu_now;
  d6d_flag |= d6d_now;
  /* The IMU line is low now, so clearing EXTI cannot consume the next edge. */
  __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT1_WAKEUP_Pin);
  HAL_NVIC_ClearPendingIRQ(IMU_INT1_WAKEUP_EXTI_IRQn);

  __disable_irq();
  imu_exti_event = g_imu_exti_wakeup_flag;
  g_imu_exti_wakeup_flag = 0U;
  __enable_irq();
  if (imu_exti_event && !wu_flag && !d6d_flag) {
    /* PB1 is definitive evidence that the IMU woke the MCU. Source registers
     * are only classification metadata and must not be allowed to lose it.
     * Final WU/6D classification is corrected after the 3-second capture. */
    wu_flag = 1U;
    source_fallback = 1U;
    if (g_imu_source_fallback_count < 0xFFU)
      g_imu_source_fallback_count++;
  }

  if (wu_flag && d6d_flag) {
    if (g_imu_both_source_count < 0xFFFFU) g_imu_both_source_count++;
  } else if (d6d_flag) {
    if (g_imu_6d_source_count < 0xFFFFU) g_imu_6d_source_count++;
  } else if (wu_flag) {
    if (g_imu_wu_source_count < 0xFFFFU) g_imu_wu_source_count++;
  }

  if (g_rtc_timer_active) {
    g_rtc_deactivate_status =
        (uint8_t)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    if (g_rtc_deactivate_status == HAL_OK) g_rtc_timer_active = 0U;
  }

  if (out_wu)  *out_wu  = wu_flag;
  if (out_6d)  *out_6d  = d6d_flag;
  if (out_rtc) *out_rtc = rtc_event;
  if (out_uart) *out_uart = g_uart2_wakeup_flag;
  if (out_source_fallback) *out_source_fallback = source_fallback;
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == IMU_INT1_WAKEUP_Pin) {
    g_imu_exti_wakeup_flag = 1U;
    if (g_imu_exti_wake_count < 0xFFFFU) g_imu_exti_wake_count++;
  }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_3) {
    g_uart2_wakeup_flag = 1;
    g_uart2_activity_flag = 1;
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
  __disable_irq();
  while (1) {}
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
