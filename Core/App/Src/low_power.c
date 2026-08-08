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
#include "gpio.h"
#include "usart.h"

/* ========================== 私有宏 ========================== */

#define RTC_LSI_DIV16_TICKS_PER_SEC 2000U
#define RTC_MAX_CHUNK_SEC       32U
#define IMU_SOURCE_SETTLE_MS   2U

/* ========================== 全局变量 ========================== */

/* MX_RTC_Init() 在上电时已首次启用 RTC 唤醒定时器。 */
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
uint32_t g_guard_sleep_accum_sec = 0;     /**< RTC分段休眠的累计秒数。 */
uint16_t g_rtc_hw_wake_count = 0U;   /**< 直接观察到RTC WUTF的次数。 */
uint8_t  g_imu_source_fallback_count = 0U;     /**< INT1有效但源寄存器已清时的兜底次数。 */

/* ========================== 函数实现 ========================== */

/**
 * @brief 读取IMU源寄存器释放锁存INT1，并在异常持续拉高时重新配置中断路由。
 * @param out_wu 输出首次源寄存器快照中的WAKE-UP标志，可为NULL。
 * @param out_6d 输出首次源寄存器快照中的6D标志，可为NULL。
 * @return 1确认INT1已经回到低电平，0两轮恢复后仍然拉高。
 * @note 只清STM32 EXTI不能释放传感器推挽输出，必须读取WAKE_UP_SRC/D6D_SRC。
 *       后续读取仅用于释放仍保持的锁存，不再与第一次快照按位或；这样不会把同一
 *       停机窗口中先后发生的两个来源误报为一次"WU+6D"事件。
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
    if (attempt == 0U) {
      if (out_wu != NULL) *out_wu = wu;
      if (out_6d != NULL) *out_6d = d6d;
    }
    HAL_Delay(2U);
    if (HAL_GPIO_ReadPin(IMU_INT1_WAKEUP_GPIO_Port,
                         IMU_INT1_WAKEUP_Pin) == GPIO_PIN_RESET) return 1U;
  }

  /* 异常恢复时先屏蔽两路 INT1，使共享输出回到低电平；重新进入休眠前清除来源后
   * 再恢复 WU 与 6D 路由。 */
  (void)LSM6DS_Set_Active_Mode();
  HAL_Delay(2U);
  wu = 0U;
  d6d = 0U;
  LSM6DS_Clear_All_Interrupts_Ex(&wu, &d6d);
  (void)LSM6DS_Set_Sleep_Mode();

  for (uint8_t attempt = 0U; attempt < 5U; attempt++) {
    wu = 0U;
    d6d = 0U;
    LSM6DS_Clear_All_Interrupts_Ex(&wu, &d6d);
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
 * 恢复复用功能并重启ReceiveToIdle。IWDG在Stop1下已通过Option Byte冻结，
 * RTC按不超过32秒分段（16位WUT硬件上限），直到事件或总周期到达。
 * 函数从WFI之后原位置继续执行，不会重新运行main()初始化。
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
  uint8_t imu_exti_event = 0U;
  uint8_t source_fallback = 0U;

  if (out_wu)   *out_wu = 0;
  if (out_6d)   *out_6d = 0;
  if (out_rtc)  *out_rtc = 0;
  if (out_uart) *out_uart = 0;
  if (out_source_fallback) *out_source_fallback = 0U;

  g_uart2_wakeup_flag = 0;

  /* STM32G031 的 USART2 不能直接唤醒 Stop1，临时将 PA3 配为下降沿 EXTI。
   * 0x00 是专用唤醒令牌，收到 READY 后才发送完整命令帧。 */
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

  /* 重新布防并校验传感器寄存器；完整重写门卫配置可恢复瞬时 I2C 错误或 IMU
   * 欠压复位，避免 INT1 失效时进入 Stop1。 */
  if (!LSM6DS_Set_Sleep_Mode()) {
    g_imu_ok = LSM6DS_Config_Gatekeeper(g_cfg.wu_mg,
                                        (uint8_t)g_cfg.tilt_deg);
    if (g_imu_ok) g_imu_ok = LSM6DS_Set_Sleep_Mode();
  } else {
    g_imu_ok = 1U;
  }
  /* Stop1 前的读取只用于释放上一次的锁存输出。来源位属于上一工作窗口，不能
   * 并入下一次唤醒；否则慢速转动时先后发生的WU与6D会被误报为同一事件。 */
  (void)IMU_Drain_INT1_Latch(NULL, NULL);
  __disable_irq();
  __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT1_WAKEUP_Pin);
  HAL_NVIC_ClearPendingIRQ(IMU_INT1_WAKEUP_EXTI_IRQn);
  /* 丢弃布防前由模式切换、历史锁存或先前工作窗口留下的软件标志。此前用旧标志
   * 与当前引脚电平的或逻辑，会把已经回到低电平、且WU/6D源寄存器均为0的旧EX​​TI
   * 当成新事件，导致桌面静置时也触发上报。清除后仍检查实际引脚和EXTI硬件挂起位，
   * 不会丢失这一临界窗口内真正到达的新中断。 */
  g_imu_exti_wakeup_flag = 0U;
  if (HAL_GPIO_ReadPin(IMU_INT1_WAKEUP_GPIO_Port,
                       IMU_INT1_WAKEUP_Pin) == GPIO_PIN_SET ||
      __HAL_GPIO_EXTI_GET_IT(IMU_INT1_WAKEUP_Pin) != 0U) {
    g_imu_exti_wakeup_flag = 1U;
  }
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

    /* 上一次 RTC 中断可能在重设定时器后留下 NVIC 挂起位；WFI 前清除，避免误唤醒。 */
    HAL_NVIC_ClearPendingIRQ(RTC_TAMP_IRQn);
    __disable_irq();
    g_rtc_wakeup_flag = 0U;
    __enable_irq();
    HAL_SuspendTick();

    /* 消除进入 WFI 前最后的竞争窗口；仅在 WFI 前后短暂屏蔽中断。 */
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

    /* 维护口或 IMU 中断优先于周期性RTC分段唤醒；真实事件后重新计算心跳周期。 */
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
    /* RTC分段唤醒：继续休眠下一段，不返回业务主状态机。 */
  }

  /* WFI 返回后直接继续执行，SRAM 与外设寄存器状态均保留。 */
  HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
  HAL_NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
  /* 恢复 USART1 引脚及 USART2 RX 复用；外设配置和 I2C 状态保持，仅重启 USART2 接收。 */
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

  /* INT1 是 WAKE_UP 与 6D 的或逻辑；先等待 52Hz 嵌入式状态稳定，再读取和清除锁存位。 */
  if (g_imu_exti_wakeup_flag ||
      HAL_GPIO_ReadPin(IMU_INT1_WAKEUP_GPIO_Port,
                       IMU_INT1_WAKEUP_Pin) == GPIO_PIN_SET ||
      __HAL_GPIO_EXTI_GET_IT(IMU_INT1_WAKEUP_Pin) != 0U) {
    HAL_Delay(IMU_SOURCE_SETTLE_MS);
  }

  uint8_t wu_now = 0U, d6d_now = 0U;
  uint8_t wu_flag = 0U, d6d_flag = 0U;
  (void)IMU_Drain_INT1_Latch(&wu_now, &d6d_now);
  wu_flag = wu_now;
  d6d_flag = d6d_now;
  /* 当前 IMU 引脚已为低电平，清除 EXTI 不会吞掉下一次上升沿。 */
  __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT1_WAKEUP_Pin);
  HAL_NVIC_ClearPendingIRQ(IMU_INT1_WAKEUP_EXTI_IRQn);

  __disable_irq();
  imu_exti_event = g_imu_exti_wakeup_flag;
  g_imu_exti_wakeup_flag = 0U;
  __enable_irq();
  if (imu_exti_event && !wu_flag && !d6d_flag) {
    /* PB1 是 IMU 唤醒 MCU 的直接证据；来源寄存器仅用于分类，不能据此丢弃事件。
     * 最终 WU/6D 分类在 3 秒采样复核后修正。 */
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