/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* 保持 CubeMX 默认单通道模式，VREFINT 在运行时动态切换 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA1     ------> ADC1_IN1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA1     ------> ADC1_IN1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  * @brief  读取锂电池真实电压 (5 次均值滤波 + VREFINT 标尺反推)
  */
float ADC_Get_Battery_Voltage(void)
{
  uint32_t sum_ch1 = 0, sum_vref = 0;
  ADC_ChannelConfTypeDef sCfg = {0};

  /* ADC 硬件自校准 (使能 VREFINT) */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
    return -1.0f;
  }

  for (uint8_t i = 0; i < ADC_AVERAGE_SAMPLES; i++)
  {
    /* ---- 第一枪: Channel 1 (PA1 电池分压) ---- */
    sCfg.Channel      = ADC_CHANNEL_1;
    sCfg.Rank         = ADC_REGULAR_RANK_1;
    sCfg.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    HAL_ADC_ConfigChannel(&hadc1, &sCfg);

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
      HAL_ADC_Stop(&hadc1);
      return -1.0f;
    }
    sum_ch1 += HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    /* ---- 第二枪: VREFINT (内部参考) ---- */
    sCfg.Channel      = ADC_CHANNEL_VREFINT;
    sCfg.Rank         = ADC_REGULAR_RANK_1;
    sCfg.SamplingTime = ADC_SAMPLINGTIME_COMMON_2;
    HAL_ADC_ConfigChannel(&hadc1, &sCfg);

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
      HAL_ADC_Stop(&hadc1);
      return -1.0f;
    }
    sum_vref += HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
  }

  /* ---- 防御: 采样均值为零 ---- */
  if (sum_ch1 == 0 || sum_vref == 0) {
    return -1.0f;
  }

  /* ---- 标尺反推公式 (STM32 标准 VREFINT 校准) ----
     VREFINT_CAL:  厂存 ADC 计数值 @ VREF+ = 3.0V  (地址 0x1FFF75AA)
     VREFINT_DATA:  当前实测 VREFINT ADC 计数值
     ADC_CH1:       当前 PA1 实测 ADC 计数值

     VREF+_actual  = 3.0V × VREFINT_CAL / VREFINT_DATA
     V_PA1         = VREF+_actual × ADC_CH1 / 4095
                   = 3000mV × VREFINT_CAL × ADC_CH1 / (VREFINT_DATA × 4095)
     V_BAT         = V_PA1 × 分压比 / 1000  (V)
  ---- */
  uint16_t vrefint_cal = *((uint16_t *)ADC_VREFINT_CAL_ADDR);
  float v_pa1_mv = 3000.0f * (float)vrefint_cal * (float)sum_ch1
                 / ((float)sum_vref * 4095.0f);
  float v_bat    = v_pa1_mv * ADC_VBAT_DIVIDER / 1000.0f;

  return v_bat;
}

/**
  * @brief  锂电池电压 → 剩余电量百分比 (三元锂放电曲线分段映射)
  * @note   4.15V+→100%  4.00~4.15→90~100%  3.70~4.00→30~90%
  *         3.50~3.70→10~30%  3.40~3.50→0~10%  ≤3.40→0%
  */
uint8_t ADC_Battery_Voltage_To_Percentage(float voltage)
{
  if (voltage >= 4.15f) return 100;
  if (voltage <= 3.40f) return 0;

  if (voltage > 4.00f)
  {
    /* 4.00~4.15V → 90~100% */
    return (uint8_t)(90.0f + (voltage - 4.00f) / 0.15f * 10.0f);
  }
  else if (voltage > 3.70f)
  {
    /* 3.70~4.00V → 30~90% (平台期, 宽范围) */
    return (uint8_t)(30.0f + (voltage - 3.70f) / 0.30f * 60.0f);
  }
  else if (voltage > 3.50f)
  {
    /* 3.50~3.70V → 10~30% */
    return (uint8_t)(10.0f + (voltage - 3.50f) / 0.20f * 20.0f);
  }
  else /* 3.40 < V ≤ 3.50 */
  {
    /* 3.40~3.50V → 0~10% */
    return (uint8_t)((voltage - 3.40f) / 0.10f * 10.0f);
  }
}

/* USER CODE END 1 */
