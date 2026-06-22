/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — Multi-Algorithm AHRS + VOFA+ JustFloat
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
#include "lsm6ds.h"
#include "at_ml307c.h"  /* Turn_On_ML307C — 4G模组开机控制 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* ---------- 串口透传：环形缓冲区 + 中断接收字节 ---- */
#define RB_SIZE     128                       /* 环形缓冲区容量 (2的幂可优化) */
static uint8_t rb_pc2gsm[RB_SIZE];            /* 电脑→模组 环形缓冲区 */
static uint8_t rb_gsm2pc[RB_SIZE];            /* 模组→电脑 环形缓冲区 */
static volatile uint8_t rb_pc2gsm_head = 0;   /* 写指针 (ISR) */
static volatile uint8_t rb_pc2gsm_tail = 0;   /* 读指针 (主循环) */
static volatile uint8_t rb_gsm2pc_head = 0;
static volatile uint8_t rb_gsm2pc_tail = 0;
static uint8_t pc_rx_byte  = 0;               /* USART2 中断接收缓冲 */
static uint8_t gsm_rx_byte = 0;               /* USART1 中断接收缓冲 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Enter_Stop1_Mode(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* 冻结 IWDG — 避免调试断点时复位 */
  __HAL_DBGMCU_FREEZE_IWDG();

  /* IMU 模块初始化 */
  if (LSM6DS_Init(&hi2c2) == 1) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] IMU OK\r\n", 14, 100);
    /* 配置 IMU 6D 倾斜唤醒 (用于 Stop 模式) */
    if (LSM6DS_Config_6D_Wakeup() == 1) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] 6D WKUP OK\r\n", 18, 100);
    } else {
      HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] 6D WKUP FAIL\r\n", 20, 100);
    }
  } else {
    HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] IMU FAIL\r\n", 16, 100);
  }

  /* ML307C 4G 模组开机 (仅执行一次) */
  Turn_On_ML307C();

  /* 启动中断接收：硬件自动捕获每个字节，触发 HAL_UART_RxCpltCallback 转发 */
  HAL_UART_Receive_IT(&huart1, &gsm_rx_byte, 1);  /* 监听模组 */
  HAL_UART_Receive_IT(&huart2, &pc_rx_byte,  1);  /* 监听电脑 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ================================================================ *
     *  透传主循环：环形缓冲转发 + !OFF/!ON/!SLEEP 命令拦截               *
     * ================================================================ */

    /* 命令拦截状态机: 0=空闲, 1=!, 2=!O, 3=!OF, 4=!OFF, 10=!ON,         *
     *                  20=!S, 21=!SL, 22=!SLE, 23=!SLEE, 24=!SLEEP       */
    static uint8_t  cmd_state       = 0;
    static uint8_t  cmd_pending[8]  = {0};
    static uint8_t  cmd_pending_cnt = 0;
    static uint32_t cmd_last_tick   = 0;

    /* 超时保护 */
    if (cmd_state > 0 && HAL_GetTick() - cmd_last_tick > 500U) {
      for (uint8_t i = 0; i < cmd_pending_cnt; i++) {
        HAL_UART_Transmit(&huart1, &cmd_pending[i], 1, 10);
      }
      cmd_state = 0;  cmd_pending_cnt = 0;
    }

    /* 电脑→模组方向：逐字节处理 */
    while (rb_pc2gsm_tail != rb_pc2gsm_head)
    {
      uint8_t ch = rb_pc2gsm[rb_pc2gsm_tail];
      rb_pc2gsm_tail = (rb_pc2gsm_tail + 1) % RB_SIZE;
      uint8_t handled = 0;

      /* ---- 状态 0: 等待 '!' ---- */
      if (cmd_state == 0 && ch == '!') {
        cmd_pending[0] = ch;  cmd_pending_cnt = 1;  cmd_state = 1;
        cmd_last_tick = HAL_GetTick();  handled = 1;
      }
      /* ---- 状态 1: 等待 'O' 或 'S' ---- */
      else if (cmd_state == 1) {
        if (ch == 'O') {
          cmd_pending[1] = ch;  cmd_pending_cnt = 2;  cmd_state = 2;
          cmd_last_tick = HAL_GetTick();  handled = 1;
        } else if (ch == 'S' || ch == 's') {
          cmd_pending[1] = ch;  cmd_pending_cnt = 2;  cmd_state = 20;
          cmd_last_tick = HAL_GetTick();  handled = 1;
        }
      }
      /* ---- 状态 2 (!O): 等待 'F' 或 'N' ---- */
      else if (cmd_state == 2) {
        if (ch == 'F') {
          cmd_pending[2] = ch;  cmd_pending_cnt = 3;  cmd_state = 3;
          cmd_last_tick = HAL_GetTick();  handled = 1;
        } else if (ch == 'N' || ch == 'n') {
          cmd_pending[2] = ch;  cmd_pending_cnt = 3;  cmd_state = 10;
          cmd_last_tick = HAL_GetTick();  handled = 1;
        }
      }
      /* ---- 状态 3 (!OF): 等待 'F' ---- */
      else if (cmd_state == 3 && ch == 'F') {
        cmd_pending[3] = ch;  cmd_pending_cnt = 4;  cmd_state = 4;
        cmd_last_tick = HAL_GetTick();  handled = 1;
      }
      /* ---- 状态 4/10: !OFF 或 !ON 等待 \r \n ---- */
      else if ((cmd_state == 4 || cmd_state == 10) && (ch == '\r' || ch == '\n'))
      {
        if (cmd_state == 4) {
          HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[CMD] !OFF -> Shutting down...\r\n", 38, 100);
          Turn_Off_ML307C();
        } else {
          HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[CMD] !ON -> Powering on...\r\n", 36, 100);
          Turn_On_ML307C();
        }
        if (ch == '\r' && rb_pc2gsm_tail != rb_pc2gsm_head) {
          uint8_t next = rb_pc2gsm[rb_pc2gsm_tail];
          if (next == '\n') rb_pc2gsm_tail = (rb_pc2gsm_tail + 1) % RB_SIZE;
        }
        cmd_state = 0;  cmd_pending_cnt = 0;  handled = 1;
      }
      /* ---- 状态 20 (!S): 等待 'L' ---- */
      else if (cmd_state == 20 && (ch == 'L' || ch == 'l')) {
        cmd_pending[2] = ch;  cmd_pending_cnt = 3;  cmd_state = 21;
        cmd_last_tick = HAL_GetTick();  handled = 1;
      }
      /* ---- 状态 21 (!SL): 等待 'E' ---- */
      else if (cmd_state == 21 && (ch == 'E' || ch == 'e')) {
        cmd_pending[3] = ch;  cmd_pending_cnt = 4;  cmd_state = 22;
        cmd_last_tick = HAL_GetTick();  handled = 1;
      }
      /* ---- 状态 22 (!SLE): 等待 'E' ---- */
      else if (cmd_state == 22 && (ch == 'E' || ch == 'e')) {
        cmd_pending[4] = ch;  cmd_pending_cnt = 5;  cmd_state = 23;
        cmd_last_tick = HAL_GetTick();  handled = 1;
      }
      /* ---- 状态 23 (!SLEE): 等待 'P' ---- */
      else if (cmd_state == 23 && (ch == 'P' || ch == 'p')) {
        cmd_pending[5] = ch;  cmd_pending_cnt = 6;  cmd_state = 24;
        cmd_last_tick = HAL_GetTick();  handled = 1;
      }
      /* ---- 状态 24 (!SLEEP): 等待 \r \n → 进入 Stop1 ---- */
      else if (cmd_state == 24 && (ch == '\r' || ch == '\n'))
      {
        if (ch == '\r' && rb_pc2gsm_tail != rb_pc2gsm_head) {
          uint8_t next = rb_pc2gsm[rb_pc2gsm_tail];
          if (next == '\n') rb_pc2gsm_tail = (rb_pc2gsm_tail + 1) % RB_SIZE;
        }
        cmd_state = 0;  cmd_pending_cnt = 0;  handled = 1;
        Enter_Stop1_Mode();
      }

      if (!handled) {
        /* 模式不匹配 → 吐出暂存字节 + 当前字节 */
        for (uint8_t i = 0; i < cmd_pending_cnt; i++) {
          HAL_UART_Transmit(&huart1, &cmd_pending[i], 1, 10);
        }
        HAL_UART_Transmit(&huart1, &ch, 1, 10);
        cmd_state = 0;  cmd_pending_cnt = 0;
      }
    }

    /* 模组→电脑方向 */
    while (rb_gsm2pc_tail != rb_gsm2pc_head) {
      HAL_UART_Transmit(&huart2, &rb_gsm2pc[rb_gsm2pc_tail], 1, 10);
      rb_gsm2pc_tail = (rb_gsm2pc_tail + 1) % RB_SIZE;
    }

    /* 喂狗 */
    HAL_IWDG_Refresh(&hiwdg);
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
  * @brief  进入 Stop1 低功耗模式 (IMU 6D 倾斜唤醒)
  * @note   进入前需先调用 LSM6DS_Config_6D_Wakeup() 配置 IMU。
  *         唤醒源：PA0 (IMU_INT1) 上升沿。唤醒后自动恢复系统时钟。
  * @retval 无
  */
static void Enter_Stop1_Mode(void)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)"[SLEEP] Entering Stop1... (tilt IMU to wake)\r\n", 51, 100);

  /* 清除 PA0 残留中断标志 */
  __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT1_WAKEUP_Pin);

  /* 关闭 RTC 唤醒定时器 (CubeMX 默认开启了 10s 周期唤醒) */
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  /* 喂狗：确保睡眠期间 IWDG 不超时 */
  HAL_IWDG_Refresh(&hiwdg);

  /* 挂起 SysTick，防止 SysTick 中断唤醒 */
  HAL_SuspendTick();

  /* 进入 Stop1 模式 (WFI: 等待中断唤醒) */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

  /* ---- 以下代码在唤醒后执行 ---- */

  /* 刷新 IWDG */
  HAL_IWDG_Refresh(&hiwdg);

  /* 恢复 SysTick */
  HAL_ResumeTick();

  /* 重新配置系统时钟 (Stop 模式后 HSI/PLL 已停，需恢复) */
  SystemClock_Config();

  /* 重新启动串口中断接收 (Stop 模式后 UART 状态可能丢失) */
  HAL_UART_Receive_IT(&huart1, &gsm_rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &pc_rx_byte,  1);

  /* ★关键★ 清除 IMU 6D 中断锁存 (D6D_SRC 寄存器，读即清零)
     用错函数会只清理运动唤醒标志，6D 标志仍在锁存，
     INT1 保持高电平 → 下次进 Stop 等不到上升沿 → 永久沉睡 */
  uint8_t wakeup_src = LSM6DS_Clear_6D_Wakeup();

  HAL_UART_Transmit(&huart2, (uint8_t*)"[SLEEP] Woken up!", 18, 100);
  if (wakeup_src) {
    HAL_UART_Transmit(&huart2, (uint8_t*)" Source: IMU 6D tilt.\r\n", 24, 100);
  } else {
    HAL_UART_Transmit(&huart2, (uint8_t*)" Source: other.\r\n", 18, 100);
  }
}

/**
  * @brief  UART 接收完成中断回调 — 串口透传核心
  * @note   每收到 1 个字节硬件自动触发此函数，转发到对面串口后重新装填
  * @param  huart: 触发中断的 UART 句柄
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    /* 模组→电脑：字节写入环形缓冲即返回 (微秒级) */
    uint8_t next = (rb_gsm2pc_head + 1) % RB_SIZE;
    if (next != rb_gsm2pc_tail) {              /* 缓冲区未满 */
      rb_gsm2pc[rb_gsm2pc_head] = gsm_rx_byte;
      rb_gsm2pc_head = next;
    }
    HAL_UART_Receive_IT(&huart1, &gsm_rx_byte, 1);
  }
  else if (huart->Instance == USART2) {
    /* 电脑→模组：字节写入环形缓冲即返回 (微秒级) */
    uint8_t next = (rb_pc2gsm_head + 1) % RB_SIZE;
    if (next != rb_pc2gsm_tail) {              /* 缓冲区未满 */
      rb_pc2gsm[rb_pc2gsm_head] = pc_rx_byte;
      rb_pc2gsm_head = next;
    }
    HAL_UART_Receive_IT(&huart2, &pc_rx_byte, 1);
  }
}

/**
  * @brief  EXTI 上升沿中断回调 (STM32G0 系列)
  * @param  GPIO_Pin: 触发中断的引脚
  * @retval None
  */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  UNUSED(GPIO_Pin);
}

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
