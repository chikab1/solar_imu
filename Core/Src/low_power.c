/**
  * @file    low_power.c
  * @brief   Stop1低功耗休眠、IMU/RTC/串口唤醒与诊断统计
  */

#include "low_power.h"
#include "main.h"
#include "lsm6ds.h"
#include "uart_driver.h"
#include "service_protocol.h"
#include "service_handler.h"
#include "sys_config.h"
#include "event_report.h"
#include "power_manager.h"
#include "rtc.h"
#include "iwdg.h"
#include "gpio.h"
#include "usart.h"

/* ========================== 私有宏 ========================== */

#define RTC_LSI_DIV16_TICKS_PER_SEC 2000U
#define RTC_MAX_CHUNK_SEC       20U
#define IWDG_STOP_GUARD_SEC    20U
#define IMU_SOURCE_SETTLE_MS   2U

/* ========================== 全局变量 ========================== */

/* MX_RTC_Init() arms the wakeup timer once during boot. */
uint8_t  g_rtc_timer_active = 1U;          /**< 1表示RTC唤醒定时器当前已启用。 */
uint16_t g_rtc_arm_count = 0U;            /**< RTC定时器布防尝试累计数。 */
uint16_t g_rtc_last_interval = 0U;        /**< 最近一次分段Stop时长，秒。 */
uint32_t g_rtc_last_cr = 0U;              /**< 布防后的RTC->CR诊断快照。 */
uint32_t g_rtc_last_sr = 0U;              /**< 布防后的RTC->SR诊断快照。 */
uint32_t g_rtc_last_requested_sleep = 0U; /**< 最近请求的总休眠秒数。 */
uint16_t g_rtc_consumed_count = 0U;       /**< 被内部看门狗分段消费的RTC唤醒数。 */
uint16_t g_rtc_ready_count = 0U;          /**< 已达到业务心跳时间的RTC唤醒数。 */
uint8_t  g_rtc_arm_status = HAL_OK;            /**< 最近一次RTC唤醒定时器布防结果。 */
uint8_t  g_rtc_deactivate_status = HAL_OK;     /**< 最近一次RTC定时器停用结果。 */
uint16_t g_imu_wu_source_count = 0U;          /**< 仅WAKE-UP源累计次数。 */
uint16_t g_imu_6d_source_count = 0U;          /**< 仅6D源累计次数。 */
uint16_t g_imu_both_source_count = 0U;        /**< WU和6D同时置位累计次数。 */
uint32_t g_guard_sleep_accum_sec = 0;     /**< 为喂IWDG而分段休眠的累计秒数。 */
uint16_t g_rtc_hw_wake_count = 0U;   /**< 直接观察到RTC WUTF的次数。 */
uint8_t  g_imu_source_fallback_count = 0U;     /**< INT1有效但源寄存器已清时的兜底次数。 */

/* ========================== 函数实现 ========================== */

/**
 * @brief 读取IMU源寄存器释放锁存INT1，并在异常持续拉高时重新配置中断路由。
 * @param out_wu 累积输出本轮读到的WAKE-UP标志，可为NULL。
 * @param out_6d 累积输出本轮读到的6D标志，可为NULL。
 * @return 1确认INT1已经回到低电平，0两轮恢复后仍然拉高。
 * @note 只清STM32 EXTI不能释放传感器推挽输出，必须读取WAKE_UP_SRC/D6D_SRC。
 */
uint8_t IMU_Drain_INT1_Latch(uint8_t *out_wu, uint8_t *out_6d)
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

/**
 * @brief 布防IMU/RTC/维护口，进入可重复的Stop1分段休眠并恢复外设引脚。
 * @param out_wu 返回WAKE-UP源，非NULL时写0/1。
 * @param out_6d 返回6D源，非NULL时写0/1。
 * @param out_rtc 返回是否达到业务RTC周期。
 * @param out_uart 返回维护口PA3下降沿唤醒。
 * @param out_source_fallback 返回是否仅凭INT1确认、未读到具体IMU源。
 * @details USART2_RX在Stop前临时改为下降沿EXTI，0x00为牺牲唤醒字节；唤醒后
 * 恢复复用功能并重启ReceiveToIdle。若IWDG在Stop计数，RTC按不超过20秒分段，
 * 中间唤醒只喂狗并继续睡，直到事件或总周期到达。函数从WFI之后原位置继续执行，
 * 不会重新运行main()初始化。
 */
void Enter_Stop1_Mode(uint8_t *out_wu, uint8_t *out_6d,
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

  /* Re-arm and verify the actual sensor registers.  One full gatekeeper
   * rewrite recovers a transient I2C error or a sensor brownout/reset instead
   * of entering Stop1 with an inert INT1 line. */
  if (!LSM6DS_Set_Sleep_Mode()) {
    g_imu_ok = LSM6DS_Config_Gatekeeper(g_cfg.wu_mg,
                                        (uint8_t)g_cfg.tilt_deg);
    if (g_imu_ok) g_imu_ok = LSM6DS_Set_Sleep_Mode();
  } else {
    g_imu_ok = 1U;
  }
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
  /* 网络失败不创建额外RTC重试；只按用户配置的正常心跳周期再次唤醒。 */
  requested_sleep = g_cfg.sleep_sec;
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
