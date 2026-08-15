/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/** @brief 可通过维护串口/服务器下发并保存到RTC备份寄存器的运行配置。 */
typedef struct {
    uint32_t magic;      /**< 配置有效标记；不匹配时恢复出厂值。 */
    uint16_t wu_mg;      /**< IMU运动唤醒阈值，单位mg，默认750。 */
    uint16_t tilt_deg;   /**< 相对安装零轴的倾角报警阈值，单位度，默认30。 */
    uint32_t sleep_sec;  /**< 无事件时RTC心跳/上报周期，单位秒，默认3600。 */
    uint16_t v_low_mv;   /**< 禁止启动4G的低压熔断阈值，单位mV，默认3550。 */
    uint8_t  mount_axis; /**< 固定零度参考轴，取MountAxis_t。 */
} SysConfig_t;

/** @brief 设备安装时“0度”所对应的重力正/负轴方向。 */
typedef enum {
    MOUNT_AXIS_Z_POS = 0, /**< +Z与重力同向时为0度。 */
    MOUNT_AXIS_Z_NEG = 1, /**< -Z与重力同向时为0度。 */
    MOUNT_AXIS_X_POS = 2, /**< +X与重力同向时为0度。 */
    MOUNT_AXIS_X_NEG = 3, /**< -X与重力同向时为0度。 */
    MOUNT_AXIS_Y_POS = 4, /**< +Y与重力同向时为0度。 */
    MOUNT_AXIS_Y_NEG = 5  /**< -Y与重力同向时为0度。 */
} MountAxis_t;

/** @brief 主循环待执行任务；CMD_TEST代表运行一遍完整事件上报流程。 */
typedef enum { CMD_NONE = 0, CMD_ON, CMD_OFF, CMD_SLEEP, CMD_VBAT, CMD_MQTT, CMD_TEST } PendingCmd_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* 函数原型 — 跨模块调用 */
void SystemClock_Config(void);

/* 模块间共享变量 — 定义在 main.c, 其他模块通过 extern 引用 */

extern volatile PendingCmd_t g_pending_cmd;

/* Stop1唤醒源和串口恢复 flags (HAL回调写入) */
extern volatile uint8_t  g_uart2_wakeup_flag;
extern volatile uint8_t  g_uart2_activity_flag;
extern volatile uint8_t  g_uart2_rearm_needed;
extern volatile uint8_t  g_uart1_rearm_needed;
extern volatile uint8_t  g_rtc_wakeup_flag;
extern volatile uint8_t  g_imu_exti_wakeup_flag;
extern volatile uint16_t g_imu_exti_wake_count;
extern volatile uint16_t g_rtc_callback_count;

/* 主循环与事件上报间的状态传递 */
extern uint8_t  g_report_wu;
extern uint8_t  g_report_6d;
extern uint8_t  g_report_rtc;
extern uint8_t  g_report_source_fallback;
extern uint8_t  g_report_include_gps;
extern uint8_t  g_report_boot;

/* 启动诊断 */
extern uint8_t g_imu_ok;
extern uint8_t g_reset_reason;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_INT1_WAKEUP_Pin GPIO_PIN_1
#define IMU_INT1_WAKEUP_GPIO_Port GPIOB
#define IMU_INT1_WAKEUP_EXTI_IRQn EXTI0_1_IRQn
#define LTE_RESET_Pin GPIO_PIN_3
#define LTE_RESET_GPIO_Port GPIOB
#define LTE_PWRKEY_Pin GPIO_PIN_4
#define LTE_PWRKEY_GPIO_Port GPIOB
#define LTE_STATE_Pin GPIO_PIN_8
#define LTE_STATE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
