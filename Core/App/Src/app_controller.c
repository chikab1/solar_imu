/**
 * @file app_controller.c
 * @brief 设备上电初始化、事件调度和 HAL 回调归属。
 */

#include "app_controller.h"

#include "main.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "lsm6ds.h"
#include "at_ml307c.h"
#include "uart_driver.h"
#include "event_store.h"
#include "service_protocol.h"
#include "sys_config.h"
#include "event_report.h"
#include "low_power.h"
#include "service_handler.h"

/** 与维护协议和低功耗模块共享的异步状态。 */
volatile PendingCmd_t g_pending_cmd = CMD_NONE;
volatile uint8_t g_uart2_wakeup_flag = 0U;
volatile uint8_t g_uart2_activity_flag = 0U;
volatile uint8_t g_uart2_rearm_needed = 0U;
volatile uint8_t g_uart1_rearm_needed = 0U;
volatile uint8_t g_rtc_wakeup_flag = 0U;
volatile uint8_t g_imu_exti_wakeup_flag = 0U;
volatile uint16_t g_imu_exti_wake_count = 0U;
volatile uint16_t g_rtc_callback_count = 0U;

/** 下一次自动上报的唤醒来源及启动诊断状态。 */
uint8_t g_report_wu = 0U;
uint8_t g_report_6d = 0U;
uint8_t g_report_rtc = 0U;
uint8_t g_report_source_fallback = 0U;
uint8_t g_report_include_gps = 1U;
uint8_t g_report_boot = 0U;
uint8_t g_imu_ok = 0U;
uint8_t g_reset_reason = 0U;

/**
 * @brief 读取并清除 RCC 复位原因，供首次事件上报。
 */
static void App_Read_Reset_Reason(void)
{
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PWRRST))  g_reset_reason |= 0x01U;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))  g_reset_reason |= 0x02U;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  g_reset_reason |= 0x04U;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) g_reset_reason |= 0x08U;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) g_reset_reason |= 0x10U;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) g_reset_reason |= 0x20U;
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

void App_Init(void)
{
  __HAL_DBGMCU_FREEZE_IWDG();
  Config_Load();
  App_Read_Reset_Reason();

  if (LSM6DS_Init(&hi2c2) == 1) {
    g_imu_ok = LSM6DS_Config_Gatekeeper(g_cfg.wu_mg,
                                         (uint8_t)g_cfg.tilt_deg);
  }

  HAL_GPIO_WritePin(LTE_PWRKEY_GPIO_Port, LTE_PWRKEY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LTE_RESET_GPIO_Port, LTE_RESET_Pin, GPIO_PIN_RESET);

  UART_Driver_Init();
  ServiceProtocol_Init();
  (void)EventStore_Init();
  (void)LSM6DS_Set_Sleep_Mode();

  g_report_boot = 1U;
  g_pending_cmd = CMD_TEST;
}

/**
 * @brief 处理一个待执行的自动上报任务。
 */
static void App_Run_Pending_Report(void)
{
  uint8_t wu = g_report_wu;
  uint8_t d6d = g_report_6d;
  uint8_t rtc = g_report_rtc;
  uint8_t source_fallback = g_report_source_fallback;
  uint8_t include_gps = g_report_include_gps;

  g_report_wu = 0U;
  g_report_6d = 0U;
  g_report_rtc = 0U;
  g_report_source_fallback = 0U;
  /* 手动请求的选择仅作用于本次任务；自动唤醒始终保留位置跟踪。 */
  g_report_include_gps = 1U;
  g_pending_cmd = CMD_NONE;
  g_service_busy = 1U;
  (void)Run_Event_Report(wu, d6d, rtc,
                         (uint8_t)(!wu && !d6d && !rtc && !source_fallback),
                         source_fallback, include_gps);
  g_report_boot = 0U;
  g_service_busy = 0U;
  if (g_serial_session_active) g_last_uart2_activity = HAL_GetTick();
}

/**
 * @brief 在没有维护会话和待执行任务时进入 Stop1，并把有效唤醒原因转为上报任务。
 */
static void App_Enter_Stop_When_Idle(void)
{
  uint8_t wu = 0U;
  uint8_t d6d = 0U;
  uint8_t rtc = 0U;
  uint8_t uart = 0U;
  uint8_t fallback = 0U;

  Enter_Stop1_Mode(&wu, &d6d, &rtc, &uart, &fallback);
  if (uart) {
    g_serial_session_active = 1U;
    g_last_uart2_activity = HAL_GetTick();
    ServiceProtocol_SendWakeAck();
  }
  if (wu || d6d || rtc || fallback) {
    g_report_wu = wu;
    g_report_6d = d6d;
    g_report_rtc = rtc;
    g_report_source_fallback = fallback;
    g_report_include_gps = 1U;
    g_pending_cmd = CMD_TEST;
  }
}

void App_Run(void)
{
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

  if (g_pending_cmd == CMD_TEST) App_Run_Pending_Report();

  if (g_serial_session_active &&
      (HAL_GetTick() - g_last_uart2_activity) >= SERIAL_IDLE_TIMEOUT_MS &&
      g_pending_cmd == CMD_NONE) {
    Turn_Off_ML307C();
    g_serial_session_active = 0U;
  }

  if (!g_serial_session_active && g_pending_cmd == CMD_NONE) {
    App_Enter_Stop_When_Idle();
  }

  HAL_IWDG_Refresh(&hiwdg);
}

/**
 * @brief USART2 ReceiveToIdle 回调：将收到的字节交给维护协议解析器。
 * @param huart 触发回调的串口句柄，仅处理 USART2。
 * @param Size 本次 DMA 缓冲区有效字节数。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2) {
    if (Size > 0U) {
      g_uart2_activity_flag = 1U;
      ServiceProtocol_Feed(g_uart2_drv.dma_rx_buf, Size);
    }
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, g_uart2_drv.dma_rx_buf,
                                    g_uart2_drv.dma_rx_buf_size) != HAL_OK) {
      g_uart2_rearm_needed = 1U;
    }
  }
}

/**
 * @brief UART 错误回调：记录恢复请求，实际恢复工作由主循环执行。
 * @param huart 出错串口句柄。
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
 * @brief RTC 唤醒回调：记录 Stop1 唤醒原因。
 * @param hrtc_cb 触发回调的 RTC 句柄。
 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc_cb)
{
  if (hrtc_cb->Instance == RTC) {
    g_rtc_wakeup_flag = 1U;
    if (g_rtc_callback_count < 0xFFFFU) g_rtc_callback_count++;
  }
}

/**
 * @brief IMU INT1 上升沿回调：记录 IMU 唤醒，来源分类在退出 Stop1 后完成。
 * @param GPIO_Pin 产生中断的 GPIO 编号。
 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == IMU_INT1_WAKEUP_Pin) {
    g_imu_exti_wakeup_flag = 1U;
    if (g_imu_exti_wake_count < 0xFFFFU) g_imu_exti_wake_count++;
  }
}

/**
 * @brief PA3 下降沿回调：记录维护串口唤醒请求。
 * @param GPIO_Pin 产生中断的 GPIO 编号。
 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_3) {
    g_uart2_wakeup_flag = 1U;
    g_uart2_activity_flag = 1U;
  }
}
