/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 智能路牌低功耗监测、事件确认、4G/MQTT上报与维护串口主流程
  * @details        : main()仅初始化一次，之后在while(1)中循环执行：
  *                   Stop1布防 -> IMU/RTC/串口唤醒 -> 3秒采样与4G并行启动
  *                   -> 网络/MQTT上报 -> 失败事件入Flash -> 模组关机 -> 再次Stop1。
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
#include "sys_config.h"
#include "power_manager.h"
#include "event_report.h"
#include "low_power.h"
#include "service_handler.h"
#include "rtc_utils.h"
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* (宏已迁移至对应模块: sys_config, power_manager, event_report, service_handler, low_power) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ── 主循环核心状态（HAL回调写入） ── */

/** @brief 主循环待执行任务；CMD_TEST代表运行一遍完整事件上报流程。 */
volatile PendingCmd_t g_pending_cmd = CMD_NONE; /**< 协议处理与主循环间的任务标志。 */

/* Stop1唤醒源、串口恢复和诊断统计。volatile变量可能在HAL回调中修改。 */
volatile uint8_t  g_uart2_wakeup_flag = 0;     /**< PA3下降沿表明维护口请求唤醒。 */
volatile uint8_t  g_uart2_activity_flag = 0;   /**< USART2收到任意数据，刷新60秒会话。 */
volatile uint8_t  g_uart2_rearm_needed = 0;    /**< USART2接收中断需由主循环恢复。 */
volatile uint8_t  g_uart1_rearm_needed = 0;    /**< USART1循环DMA需由主循环恢复。 */
volatile uint8_t  g_rtc_wakeup_flag = 0;       /**< RTC唤醒定时器HAL回调标志。 */
volatile uint8_t  g_imu_exti_wakeup_flag = 0U; /**< PB1/IMU INT1上升沿标志。 */
volatile uint16_t g_imu_exti_wake_count = 0U; /**< IMU EXTI累计唤醒次数。 */
volatile uint16_t g_rtc_callback_count = 0U;  /**< HAL RTC回调执行次数。 */

/* Wake information consumed by the automatic full-report command. */
uint8_t g_report_wu = 0;                  /**< 下一次上报是否由WAKE-UP触发。 */
uint8_t g_report_6d = 0;                  /**< 下一次上报是否由6D触发。 */
uint8_t g_report_rtc = 0;                 /**< 下一次上报是否由RTC心跳触发。 */
uint8_t g_report_source_fallback = 0U;    /**< 下一次上报是否使用INT1来源兜底分类。 */
uint8_t g_iwdg_runs_in_stop = 0;          /**< Option Bytes是否允许IWDG在Stop继续计数。 */
uint8_t g_imu_ok = 0;                      /**< IMU初始化和门卫配置是否成功。 */
uint8_t g_reset_reason = 0;                /**< RCC复位原因位图，上报到事件记录。 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* (函数原型已迁移至对应模块头文件) */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* (业务逻辑已迁移至对应模块: sys_config, power_manager, rtc_utils,
    service_handler, event_report, low_power) */
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

            ML307C_GPS_Data_t gps = {0};
            ML307C_LBS_Data_t lbs = {0};
            if (ML307C_GPS_Start()) {
              if (!ML307C_GPS_Wait_Fix(&gps, 5000U)) {
                /* 单次定位仅成功后自动关闭；失败/超时必须显式停止。 */
                (void)ML307C_GPS_Stop();
                gps.is_fixed = 0;
              }
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

/**
 * @brief HAL ReceiveToIdle回调：把USART2新字节直接交给流式协议解析器并重装接收。
 * @param huart 触发回调的UART句柄；本应用仅处理USART2。
 * @param Size 本次接收区中的有效字节数。
 * @note 回调内不执行命令和阻塞发送，业务由Service_Task()在主循环完成。
 */
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

/**
 * @brief UART错误回调：清错误标志并把接收恢复工作移交主循环。
 * @param huart 出错的USART1或USART2句柄。
 */
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

/**
 * @brief RTC唤醒定时器回调，设置Stop1退出原因并累计诊断值。
 * @param hrtc_cb HAL传入的RTC句柄。
 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc_cb)
{
  if (hrtc_cb->Instance == RTC) {
    g_rtc_wakeup_flag = 1;
    if (g_rtc_callback_count < 0xFFFFU) g_rtc_callback_count++;
  }
}

/**
 * @brief GPIO上升沿回调，记录IMU INT1唤醒；源分类在退出Stop后读取寄存器完成。
 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == IMU_INT1_WAKEUP_Pin) {
    g_imu_exti_wakeup_flag = 1U;
    if (g_imu_exti_wake_count < 0xFFFFU) g_imu_exti_wake_count++;
  }
}

/**
 * @brief GPIO下降沿回调，Stop1期间把PA3低电平识别为维护口唤醒请求。
 */
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
