/**
  * @file    service_handler.c
  * @brief   维护串口协议处理：命令分发、UART恢复、AT后台轮询与延时
  */

#include "service_handler.h"
#include "main.h"
#include "uart_driver.h"
#include "service_protocol.h"
#include "event_store.h"
#include "at_ml307c.h"
#include "lsm6ds.h"
#include "adc.h"
#include "iwdg.h"
#include "power_manager.h"
#include "sys_config.h"
#include "event_report.h"
#include "low_power.h"
#include <string.h>

/* ========================== 私有宏 ========================== */

#define SERIAL_IDLE_TIMEOUT_MS 60000U
#define FLASH_SAFE_VOLTAGE_MV  3450U
#define CRITICAL_VOLTAGE_MV    3350U

/* ========================== 全局变量 ========================== */

uint8_t  g_serial_session_active = 0;     /**< 1表示维护会话打开，60秒无数据后关闭。 */
uint32_t g_last_uart2_activity = 0;       /**< 最近维护口活动的HAL毫秒时刻。 */
uint8_t g_service_busy = 0U;              /**< 完整上报中只允许维护命令得到BUSY响应。 */
uint8_t g_service_task_running = 0U;      /**< 防止AT后台回调递归进入Service_Task。 */

/* ========================== 函数实现 ========================== */

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

/**
 * @brief 执行一条已通过CRC校验的USART2维护命令并发送对应响应。
 * @param frame 请求帧；功能码、payload格式见service_protocol.h和README。
 * @note 耗时操作只在主循环调用；HAL接收回调仅负责Feed字节。
 */
void Handle_Service_Frame(const ServiceFrame_t *frame)
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

/** @brief 主循环中尝试恢复因HAL错误或重装失败而停止的USART2接收。 */
void Service_UART2_Recover(void)
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

/** @brief 主循环中尝试恢复ML307C USART1循环DMA接收。 */
void Service_UART1_Recover(void)
{
  if (!g_uart1_rearm_needed) return;
  if (UART1_RestartReceive()) g_uart1_rearm_needed = 0U;
}

/**
 * @brief 维护两个UART接收状态并处理唤醒令牌、完整帧和协议错误。
 * @param busy_only 1时不执行命令，只对所有完整请求回复BUSY；上报阻塞等待时使用。
 * @note g_service_task_running阻止AT等待回调造成递归执行。
 */
void Service_Task(uint8_t busy_only)
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

/**
 * @brief 覆盖ML307C驱动弱回调，使长AT等待期间维护口仍能及时回复BUSY。
 */
void ML307C_Background_Poll(void)
{
  Service_Task(1U);
}

/** @brief 在指定延时内持续维护USART2并刷新IWDG。 */
void Delay_With_Service(uint32_t delay_ms)
{
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < delay_ms) {
    Service_Task(1U);
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(5U);
  }
}
