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
#include "iwdg.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "at_ml307c.h"
#include "i2c_sw.h"
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
LSM6DS_RawData_t imu_raw;             /* IMU原始数据 */
LSM6DS_FloatData_t imu_data;          /* IMU物理量数据 */
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
  /* USER CODE BEGIN 2 */
  // 冻结独立看门狗在低功耗模式下的计数，防止休眠期间复位
  __HAL_RCC_DBGMCU_CLK_ENABLE();
  __HAL_DBGMCU_FREEZE_IWDG();
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  // USART2测试：发送欢迎消息
  uint8_t welcome_msg[] = "USART2 Test Ready!\r\nBaud Rate: 115200\r\n";
  HAL_UART_Transmit(&huart2, welcome_msg, sizeof(welcome_msg)-1, HAL_MAX_DELAY);

  // 初始化 LSM6DS IMU传感器
  HAL_UART_Transmit(&huart2, (uint8_t *)"Initializing LSM6DS...\r\n", 22, HAL_MAX_DELAY);
#if USE_MOCK_IMU
  HAL_UART_Transmit(&huart2, (uint8_t *)"Using MOCK IMU data\r\n", 20, HAL_MAX_DELAY);
#else
  if (LSM6DS_Init() == 1) {
      HAL_UART_Transmit(&huart2, (uint8_t *)"LSM6DS initialized OK\r\n", 21, HAL_MAX_DELAY);
      // 配置唤醒检测
      LSM6DS_Config_Wakeup(0x10, 0x08);
  } else {
      HAL_UART_Transmit(&huart2, (uint8_t *)"LSM6DS init failed\r\n", 19, HAL_MAX_DELAY);
  }
#endif

  // 初始化 ML307C 模组网络
  HAL_UART_Transmit(&huart2, (uint8_t *)"Initializing ML307C...\r\n", 26, HAL_MAX_DELAY);
  if (ML307C_Network_Init(NULL) == 1) {
      HAL_UART_Transmit(&huart2, (uint8_t *)"Network initialized OK\r\n", 23, HAL_MAX_DELAY);
      
      // 连接 MQTT 服务器
      HAL_UART_Transmit(&huart2, (uint8_t *)"Connecting to MQTT broker...\r\n", 30, HAL_MAX_DELAY);
      if (ML307C_MQTT_Connect("broker.emqx.io", 1883, NULL, NULL) == 1) {
          HAL_UART_Transmit(&huart2, (uint8_t *)"MQTT connected OK\r\n", 18, HAL_MAX_DELAY);
      } else {
          HAL_UART_Transmit(&huart2, (uint8_t *)"MQTT connect failed\r\n", 20, HAL_MAX_DELAY);
      }
  } else {
      HAL_UART_Transmit(&huart2, (uint8_t *)"Network init failed\r\n", 21, HAL_MAX_DELAY);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 执行低功耗进入与唤醒测试（Stop模式已注释）
    // LowPower_Stop1_Test();
    
    // 正常运行模式：周期性喂狗和心跳输出
    HAL_UART_Transmit(&huart2, (uint8_t *)"[MODE] Normal operation mode\r\n", 30, HAL_MAX_DELAY);
    HAL_IWDG_Refresh(&hiwdg);  // 喂狗
    HAL_Delay(1000);           // 1秒周期
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