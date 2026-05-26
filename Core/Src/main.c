/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "at_ml307c.h"
#include "i2c.h"
#include "lsm6ds.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MQTT_TOPIC  "my_imu_lab_2026/data"  /* MQTT 测试主题 */
#define USE_MOCK_IMU 0                        /* Mock IMU数据开关：1=使用模拟数据，0=使用真实硬件 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t last_send_time = 0;          /* 上次发送时间 */
#define SEND_INTERVAL_MS 50            /* 发送间隔（毫秒），对应20Hz高频上报 */
float temp = 0.0f, humi = 0.0f;       /* 温湿度变量 */
char json_buf[128];                   /* JSON数据缓冲区 */

/* IMU 物理量数据 */
float acceleration_mg[3];
float angular_rate_dps[3];
char uart_log_buf[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  执行 Stop 1 模式进入与 EXTI0 外部中断唤醒测试
  * @param  None
  * @retval None
  */
void LowPower_Stop1_Test(void)
{
    // ========== Stop 模式代码已注释，保留函数框架供后续使用 ==========
    // 1. 发送即将休眠的串口提示
    // HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n[SYS] Ready to enter Stop 1 mode...\r\n", 38, HAL_MAX_DELAY);
    // HAL_UART_Transmit(&huart2, (uint8_t *)"[SYS] Wakeup sources: RTC(10s) | EXTI0(PA0 rising)\r\n", 55, HAL_MAX_DELAY);
    
    // 2. 挂起系统滴答定时器
    // HAL_SuspendTick();
    
    // 3. 清除 EXTI0 通道的挂起标志位
    // __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    
    // 4. 刷新独立看门狗
    // HAL_IWDG_Refresh(&hiwdg);
    
    // 5. 配置 RTC 唤醒定时器：10秒定时唤醒
    // HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 10, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    
    // 6. 进入 Stop 1 模式
    // HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
    
    // 7. 唤醒后恢复时钟
    // SystemClock_Config();
    
    // 8. 恢复系统滴答定时器
    // HAL_ResumeTick();
    
    // 9. 关闭 RTC 唤醒定时器
    // HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    
    // 10. 清除 EXTI0 标志
    // __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    
    // 11. 打印唤醒成功提示
    // HAL_UART_Transmit(&huart2, (uint8_t *)"[SYS] Wakeup Success! Core and Clock Resumed.\r\n", 47, HAL_MAX_DELAY);
    
    // 临时：输出函数被调用的提示
    HAL_UART_Transmit(&huart2, (uint8_t *)"[INFO] LowPower_Stop1_Test() called (Stop mode disabled)\r\n", 60, HAL_MAX_DELAY);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  // I2C Scanner implementation using Hardware I2C (I2C2)
  uint8_t msg[64];
  uint8_t device_count = 0;
  HAL_StatusTypeDef status;
  
  // 发送扫描开始提示
  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n=== Starting I2C Scanner (Hardware I2C2) ===\r\n", 46, 100);
  
  // // 遍历7位地址 0x01 到 0x7F
  // for (uint16_t i = 1; i < 128; i++) {
  //     // 检查设备是否就绪（使用硬件I2C，注意：HAL库需要将7位地址左移1位变为8位地址形式传入）
  //     status = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i << 1), 3, 1000);
      
  //     if (status == HAL_OK) {
  //         sprintf((char*)msg, "Device found at 7-bit address: 0x%02X (8-bit: 0x%02X)\r\n", i, (i << 1));
  //         HAL_UART_Transmit(&huart2, msg, strlen((char*)msg), 100);
  //         device_count++;
  //     } else if (status == HAL_BUSY) {
  //         // 总线处于忙状态 - SCL或SDA被拉低或短路
  //         sprintf((char*)msg, "Addr 0x%02X: I2C Bus BUSY! (Short detected)\r\n", i);
  //         HAL_UART_Transmit(&huart2, msg, strlen((char*)msg), 100);
  //     } else if (status == HAL_TIMEOUT) {
  //         // 超时未响应 - 芯片未应答
  //         sprintf((char*)msg, "Addr 0x%02X: I2C Timeout! (No response)\r\n", i);
  //         HAL_UART_Transmit(&huart2, msg, strlen((char*)msg), 100);
  //     } else if (status == HAL_ERROR) {
  //         // 硬件错误（未收到ACK）
  //         sprintf((char*)msg, "Addr 0x%02X: I2C Error (No ACK)!\r\n", i);
  //         HAL_UART_Transmit(&huart2, msg, strlen((char*)msg), 100);
  //     }
  // }
  
  // 输出扫描结果
  if (device_count == 0) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"No I2C devices found.\r\n", 23, 100);
  } else {
      sprintf((char*)msg, "Scan finished. Total devices found: %d\r\n", device_count);
      HAL_UART_Transmit(&huart2, msg, strlen((char*)msg), 100);
  }
  
  HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Scan Complete.\r\n", 20, 100);

  /* IMU 模块化初始化 */
  if (LSM6DS_Init(&hi2c2) == 1) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] IMU Module Initialized Successfully!\r\n", 44, 100);

      LSM6DS_Config_Wakeup(60, 0);
      HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] 6D Wakeup Armed! Ready for Multimeter Test.\r\n", 51, 100);
  } else {
      HAL_UART_Transmit(&huart2, (uint8_t*)"[CRITICAL] IMU Module Initialization Failed!\r\n", 46, 100);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
    if (LSM6DS_Read_Storage(acceleration_mg, angular_rate_dps) == 1)
    {
        sprintf(uart_log_buf, "ACC[mg]: X:%6.1f Y:%6.1f Z:%6.1f | GYR[dps]: X:%5.1f Y:%5.1f Z:%5.1f\r\n",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
                angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2]);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_log_buf, strlen(uart_log_buf), 100);
    }
    */

    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(500);
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
  * @brief  EXTI 外部中断回调函数
  * @param  GPIO_Pin: 触发中断的引脚
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        // 内核已被拉醒，此处留空，交由 main 轴向后执行时钟恢复
    }
}

/**
  * @brief  RTC 唤醒定时器中断回调函数
  * @param  hrtc: RTC 句柄
  * @retval None
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    // RTC 定时到期触发的中断
    // 此处无需做复杂处理，内核已被拉醒，退出中断后会自动向下执行 main 循环
    // 唤醒后的时钟恢复和定时器清理在 LowPower_Stop1_Test() 中处理
    UNUSED(hrtc);
}
// RTC 唤醒回调函数保留，供后续恢复 Stop 模式使用
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */