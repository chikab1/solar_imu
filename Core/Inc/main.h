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

typedef struct {
    uint32_t magic;     /* 0x55AA55AA - 魔法数字, 判断备份域是否已初始化 */
    uint16_t wu_mg;     /* 加速度计唤醒门限 (mg), 默认 300 */
    uint16_t tilt_deg;  /* 6D 倾角触发门限 (度), 默认 30 */
    uint32_t sleep_sec; /* RTC 周期唤醒间隔 (秒), 默认 3600 */
    uint16_t v_low_mv;  /* 低压熔断阈值 (mV), 默认 3300 */
    uint8_t  mount_axis; /* Fixed zero reference: Z+, Z-, X+, X-, Y+, Y- */
} SysConfig_t;

typedef enum {
    MOUNT_AXIS_Z_POS = 0,
    MOUNT_AXIS_Z_NEG = 1,
    MOUNT_AXIS_X_POS = 2,
    MOUNT_AXIS_X_NEG = 3,
    MOUNT_AXIS_Y_POS = 4,
    MOUNT_AXIS_Y_NEG = 5
} MountAxis_t;

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
