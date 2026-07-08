/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 智能路牌 低功耗固件 (DMA串口)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "lsm6ds.h"
#include "at_ml307c.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  STATE_IDLE   = 0,
  STATE_SLEEP  = 1
} SystemState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GATEKEEPER_WU_MG      300
#define GATEKEEPER_6D_DEG     30

#define USART1_RX_BUF_SIZE    512    /* USART1 DMA接收缓冲 (4G模组) */
#define USART2_RX_BUF_SIZE    256    /* USART2 IT接收缓冲  (PC指令) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static SystemState_t g_sys_state = STATE_IDLE;

/* 命令队列 (中断设标志, 主循环执行) */
typedef enum { CMD_NONE = 0, CMD_ON, CMD_OFF, CMD_SLEEP, CMD_VBAT, CMD_MQTT, CMD_TEST } PendingCmd_t;
static volatile PendingCmd_t g_pending_cmd = CMD_NONE;

/* DMA/IT 接收缓冲区 */
static uint8_t usart1_rx_buf[USART1_RX_BUF_SIZE] __attribute__((aligned(4)));
static uint8_t usart2_rx_buf[USART2_RX_BUF_SIZE] __attribute__((aligned(4)));

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Enter_Stop1_Mode(void);
static void Process_PC_Command(const uint8_t *data, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  处理 PC 发来的指令 (整行匹配)
  */
static void Process_PC_Command(const uint8_t *data, uint16_t len)
{
  /* 去掉尾部 \r\n */
  while (len > 0 && (data[len-1] == '\r' || data[len-1] == '\n')) len--;
  if (len == 0) return;

  if (len == 3 && data[0] == '!' && data[1] == 'O' && data[2] == 'N') {
    g_pending_cmd = CMD_ON;
  }
  else if (len == 4 && data[0] == '!' && data[1] == 'O' && data[2] == 'F' && data[3] == 'F') {
    g_pending_cmd = CMD_OFF;
  }
  else if (len == 6 && data[0] == '!' && data[1] == 'S' && data[2] == 'L'
                    && data[3] == 'E' && data[4] == 'E' && data[5] == 'P') {
    g_pending_cmd = CMD_SLEEP;
  }
  else if (len == 5 && data[0] == '!' && data[1] == 'V' && data[2] == 'B'
                    && data[3] == 'A' && data[4] == 'T') {
    g_pending_cmd = CMD_VBAT;
  }
  else if (len == 5 && data[0] == '!' && data[1] == 'M' && data[2] == 'Q'
                    && data[3] == 'T' && data[4] == 'T') {
    g_pending_cmd = CMD_MQTT;
  }
  else if (len == 5 && data[0] == '!' && data[1] == 'T' && data[2] == 'E'
                    && data[3] == 'S' && data[4] == 'T') {
    g_pending_cmd = CMD_TEST;
  }
  else {
    /* 非指令 → 透传给 4G 模组 (补回被剥离的 \r\n) */
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 100);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 100);
  }
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  __HAL_DBGMCU_FREEZE_IWDG();

  /* IMU + 门卫 */
  if (LSM6DS_Init(&hi2c2) == 1) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] IMU OK\r\n", 14, 100);
    if (LSM6DS_Config_Gatekeeper(GATEKEEPER_WU_MG, GATEKEEPER_6D_DEG) == 1) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] Gatekeeper OK\r\n", 20, 100);
    }
  } else {
    HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] IMU FAIL\r\n", 16, 100);
  }

  HAL_GPIO_WritePin(LTE_PWRKEY_GPIO_Port, LTE_PWRKEY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LTE_RESET_GPIO_Port,  LTE_RESET_Pin,  GPIO_PIN_RESET);

  /* 启动 DMA/IT 接收 (整行模式, 非逐字节) */
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, usart1_rx_buf, USART1_RX_BUF_SIZE);
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, usart2_rx_buf, USART2_RX_BUF_SIZE);

  HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] Ready. !ON !OFF !SLEEP !VBAT\r\n", 40, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
    }

    switch (g_sys_state)
    {
    case STATE_IDLE:
      /* 执行来自中断回调的待处理命令 (主循环上下文, 可安全调用阻塞函数) */
      if (g_pending_cmd != CMD_NONE) {
        PendingCmd_t cmd = g_pending_cmd;
        g_pending_cmd = CMD_NONE;

        switch (cmd) {
        case CMD_ON:
          HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[CMD] !ON\r\n", 14, 100);
          Turn_On_ML307C();
          break;
        case CMD_OFF:
          HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[CMD] !OFF\r\n", 15, 100);
          Turn_Off_ML307C();
          break;
        case CMD_SLEEP:
          HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[CMD] !SLEEP\r\n", 16, 100);
          HAL_Delay(10);
          Enter_Stop1_Mode();
          g_sys_state = STATE_IDLE;
          break;
        case CMD_VBAT: {
          float vbat = ADC_Get_Battery_Voltage();
          uint8_t pct = ADC_Battery_Voltage_To_Percentage(vbat);
          char buf[64]; int n;
          if (vbat < 0.0f) {
            n = snprintf(buf, sizeof(buf), "\r\n[VBAT] ADC read failed!\r\n");
          } else {
            int v_mv = (int)(vbat * 1000.0f + 0.5f);
            n = snprintf(buf, sizeof(buf), "\r\n[VBAT] %d.%03dV  %d%%\r\n",
                         v_mv / 1000, v_mv % 1000, (int)pct);
          }
          HAL_UART_Transmit(&huart2, (uint8_t *)buf, n, 100);
          break;
        }
        case CMD_MQTT: {
          HAL_UART_AbortReceive(&huart1);
          HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[MQTT] Connecting...\r\n", 23, 100);

          /* 1. 开机 + 轮询 AT (不再盲等 5s) */
          Turn_On_ML307C();
          {
            uint32_t to = HAL_GetTick();
            int ok = 0;
            while (HAL_GetTick() - to < 8000) {
              HAL_IWDG_Refresh(&hiwdg);
              if (ML307C_Send_CMD("AT", "OK", 200) == 1) { ok = 1; break; }
              HAL_Delay(200);
            }
            if (!ok) { HAL_UART_Transmit(&huart2, (uint8_t*)"[MQTT] PWR timeout\r\n", 20, 100); goto mqtt_end; }
          }

          /* 2. 基础握手仅一次, 然后只轮询 CGATT */
          {
            ML307C_Network_Status_t ns = {0};
            ML307C_Network_Init(&ns);  /* AT/ATE0/CPIN 一次过 */
            uint32_t to = HAL_GetTick();
            int ok = 0;
            while (HAL_GetTick() - to < 15000) {
              HAL_IWDG_Refresh(&hiwdg);
              if (ML307C_Send_CMD("AT+CGATT?", "+CGATT: 1", 500) == 1) { ok = 1; break; }
              HAL_Delay(500);
            }
            if (!ok) { HAL_UART_Transmit(&huart2, (uint8_t*)"[MQTT] NET timeout\r\n", 20, 100); goto mqtt_end; }
          }

          /* 3. MQTT 握手 */
          if (ML307C_MQTT_Connect("101.34.217.153", 1883, "solar_imu", "solar_imu") != 1) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"[MQTT] Connect FAIL\r\n", 21, 100);
            goto mqtt_end;
          }
          HAL_IWDG_Refresh(&hiwdg);

          /* 4. 秒发数据 */
          {
            float v = ADC_Get_Battery_Voltage();
            float tilt = LSM6DS_Get_Tilt_Angle();
            int v100 = (int)(v * 100.0f + 0.5f);
            int t100 = (int)(tilt * 100.0f + 0.5f);
            ML307C_Send_CustomData((int16_t)v100, (int16_t)t100, "solar_imu/test");
            HAL_UART_Transmit(&huart2, (uint8_t*)"[MQTT] Data sent!\r\n", 18, 100);
          }

        mqtt_end:
          Turn_Off_ML307C();
          HAL_UARTEx_ReceiveToIdle_IT(&huart1, usart1_rx_buf, USART1_RX_BUF_SIZE);
          HAL_UART_Transmit(&huart2, (uint8_t*)"[MQTT] Done.\r\n", 14, 100);
          break;
        }
        case CMD_TEST: {
          uint32_t t_start, t_pwr = 0, t_net = 0, t_conn = 0, t_pub = 0;
          ML307C_Network_Status_t ns = {0};
          char log[200]; int n;

          HAL_UART_AbortReceive(&huart1);
          HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[TEST] Timer start...\r\n", 24, 100);

          t_start = HAL_GetTick();

          /* 1. 开机 */
          Turn_On_ML307C();
          {
            uint32_t to = HAL_GetTick();
            while (HAL_GetTick() - to < 8000) {
              HAL_IWDG_Refresh(&hiwdg);
              if (ML307C_Send_CMD("AT", "OK", 200) == 1) { t_pwr = HAL_GetTick(); break; }
              HAL_Delay(200);
            }
          }
          if (!t_pwr) { HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] PWR timeout!\r\n", 21, 100); goto test_end; }

          /* 2. 网络附着: 基础握手仅一次, 然后只轮询 CGATT */
          ML307C_Network_Init(&ns);
          {
            uint32_t to = HAL_GetTick();
            while (HAL_GetTick() - to < 15000) {
              HAL_IWDG_Refresh(&hiwdg);
              if (ML307C_Send_CMD("AT+CGATT?", "+CGATT: 1", 500) == 1) { t_net = HAL_GetTick(); break; }
              HAL_Delay(500);
            }
          }
          if (!t_net) { HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] NET timeout!\r\n", 21, 100); goto test_end; }

          /* 3. MQTT 握手 */
          if (ML307C_MQTT_Connect("101.34.217.153", 1883, "solar_imu", "solar_imu") == 1) {
            t_conn = HAL_GetTick();
          } else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] MQTT fail!\r\n", 19, 100);
            goto test_end;
          }

          /* 4. 发布数据 (直发, 无盲等) */
          {
            float v = ADC_Get_Battery_Voltage();
            float tilt = LSM6DS_Get_Tilt_Angle();
            int v100 = (int)(v * 100.0f + 0.5f);
            int t100 = (int)(tilt * 100.0f + 0.5f);
            if (ML307C_Send_CustomData((int16_t)v100, (int16_t)t100, "solar_imu/test") == 1) {
              t_pub = HAL_GetTick();
            } else {
              HAL_UART_Transmit(&huart2, (uint8_t*)"[TEST] Pub fail!\r\n", 18, 100);
              goto test_end;
            }
          }

          /* 报表 */
          n = snprintf(log, sizeof(log),
                       "\r\n==== PROFILER REPORT ====\r\n"
                       "1.Power-On  : %4d ms\r\n"
                       "2.Network   : %4d ms\r\n"
                       "3.MQTT Conn : %4d ms\r\n"
                       "4.Publish   : %4d ms\r\n"
                       "------------------------\r\n"
                       "TOTAL       : %4d ms\r\n"
                       "========================\r\n",
                       (int)(t_pwr - t_start), (int)(t_net - t_pwr),
                       (int)(t_conn - t_net), (int)(t_pub - t_conn),
                       (int)(t_pub - t_start));
          HAL_UART_Transmit(&huart2, (uint8_t *)log, n, 200);

        test_end:
          Turn_Off_ML307C();
          HAL_UARTEx_ReceiveToIdle_IT(&huart1, usart1_rx_buf, USART1_RX_BUF_SIZE);
          break;
        }
        default: break;
        }
      }
      HAL_IWDG_Refresh(&hiwdg);
      break;

    case STATE_SLEEP:
      HAL_IWDG_Refresh(&hiwdg);
      Enter_Stop1_Mode();
      g_sys_state = STATE_IDLE;
      break;
    }
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
  * @brief  UART 接收完成回调 (DMA/IT 整行接收)
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1) {
    /* 4G 模组 → PC: 直接转发 */
    if (Size > 0) {
      HAL_UART_Transmit(&huart2, usart1_rx_buf, Size, 100);
    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, usart1_rx_buf, USART1_RX_BUF_SIZE);
  }
  else if (huart->Instance == USART2) {
    /* PC 指令: 解析执行或透传 */
    if (Size > 0) {
      Process_PC_Command(usart2_rx_buf, Size);
    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, usart2_rx_buf, USART2_RX_BUF_SIZE);
  }
}

/**
  * @brief  Stop1 深度休眠
  */
static void Enter_Stop1_Mode(void)
{
  GPIO_InitTypeDef g = {0};
  g.Pin  = GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &g);

  { uint8_t _wu, _6d; LSM6DS_Clear_All_Interrupts_Ex(&_wu, &_6d); }
  HAL_Delay(1);
  __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT1_WAKEUP_Pin);
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  HAL_IWDG_Refresh(&hiwdg);
  HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

  /* ===== 醒来 ===== */
  HAL_IWDG_Refresh(&hiwdg);
  HAL_ResumeTick();
  SystemClock_Config();

  HAL_UART_DeInit(&huart1);
  HAL_UART_DeInit(&huart2);
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  HAL_Delay(2);

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, usart1_rx_buf, USART1_RX_BUF_SIZE);
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, usart2_rx_buf, USART2_RX_BUF_SIZE);

  HAL_I2C_DeInit(&hi2c2);
  MX_I2C2_Init();

  uint8_t wu_flag = 0, d6d_flag = 0;
  LSM6DS_Clear_All_Interrupts_Ex(&wu_flag, &d6d_flag);
  __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT1_WAKEUP_Pin);

  HAL_UART_Transmit(&huart2, (uint8_t*)"[WAKE] ", 7, 100);
  if (wu_flag && d6d_flag) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"WU+6D\r\n", 8, 100);
  } else if (wu_flag) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"WAKE-UP\r\n", 10, 100);
  } else if (d6d_flag) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"6D tilt\r\n", 10, 100);
  } else {
    HAL_UART_Transmit(&huart2, (uint8_t*)"Unknown\r\n", 10, 100);
  }
}

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
  __disable_irq();
  while (1) {}
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
