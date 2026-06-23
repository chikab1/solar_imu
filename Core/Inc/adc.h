/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */

/* ===== 电池电压监测参数 ===== */
#define ADC_VREFINT_CAL_ADDR  0x1FFF75AA   /* STM32G0 VREFINT 出厂校准值存储地址     */
#define ADC_VBAT_DIVIDER          2.0f     /* 分压比: 1M+1M, VBAT = V_PA1 × 2      */
#define ADC_AVERAGE_SAMPLES         5      /* 均值滤波采样次数                       */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

/**
  * @brief  读取锂电池真实电压 (5次均值滤波 + VREFINT 标尺反推)
  * @note   内部自动完成 ADC 校准 → 双通道扫描 (IN1 + VREFINT) → 公式反推
  * @retval 电池电压 (V), 范围 3.4~4.2V, 异常返回 -1.0f
  */
float ADC_Get_Battery_Voltage(void);

/**
  * @brief  锂电池电压 → 剩余电量百分比 (三元锂电池放电曲线分段映射)
  * @param  voltage: 电池电压 (V)
  * @retval 0 ~ 100 (%)
  */
uint8_t ADC_Battery_Voltage_To_Percentage(float voltage);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

