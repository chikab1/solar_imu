/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 智能路牌：3态低功耗防盗固件 + 串口指令系统
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
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
/* ===== 双门卫阈值 (修改此处即可调参，无需改其他代码) ===== */
#define GATEKEEPER_WU_MG      300   /* WAKE-UP: 0~1968 mg, 填物理量                     */
#define GATEKEEPER_6D_DEG     30    /* 6D: 填入期望角度°, 自动映射最近硬件档 (~10/20/30/40°) */

#define RB_SIZE               128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ---------- 系统状态 ---------- */
static SystemState_t g_sys_state = STATE_IDLE;

/* ---------- 透传环形缓冲区 ---------- */
static uint8_t rb_pc2gsm[RB_SIZE];
static uint8_t rb_gsm2pc[RB_SIZE];
static volatile uint8_t rb_pc2gsm_head = 0;
static volatile uint8_t rb_pc2gsm_tail = 0;
static volatile uint8_t rb_gsm2pc_head = 0;
static volatile uint8_t rb_gsm2pc_tail = 0;
static uint8_t pc_rx_byte  = 0;
static uint8_t gsm_rx_byte = 0;

/* ---------- 命令拦截状态机 ---------- */
static uint8_t  cmd_state       = 0;
static uint8_t  cmd_pending[8]  = {0};
static uint8_t  cmd_pending_cnt = 0;
static uint32_t cmd_last_tick   = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Enter_Stop1_Mode(void);
static void Passthrough_Drain_With_Cmds(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  透传排空 + 命令拦截 (!OFF / !ON / !SLEEP)
  */
static void Passthrough_Drain_With_Cmds(void)
{
  /* 超时保护：命令半截卡住超过 500ms → 清空 */
  if (cmd_state > 0 && HAL_GetTick() - cmd_last_tick > 500U) {
    for (uint8_t i = 0; i < cmd_pending_cnt; i++) {
      HAL_UART_Transmit(&huart1, &cmd_pending[i], 1, 10);
    }
    cmd_state = 0;  cmd_pending_cnt = 0;
  }

  /* PC→模组：逐字节处理，边转发边检测命令 */
  while (rb_pc2gsm_tail != rb_pc2gsm_head)
  {
    uint8_t ch = rb_pc2gsm[rb_pc2gsm_tail];
    rb_pc2gsm_tail = (rb_pc2gsm_tail + 1) % RB_SIZE;
    uint8_t handled = 0;

    /* 状态 0: 等待 '!' */
    if (cmd_state == 0 && ch == '!') {
      cmd_pending[0] = ch;  cmd_pending_cnt = 1;  cmd_state = 1;
      cmd_last_tick = HAL_GetTick();  handled = 1;
    }
    /* 状态 1 (!): 等 O/S/V */
    else if (cmd_state == 1) {
      if (ch == 'O') {
        cmd_pending[1] = ch;  cmd_pending_cnt = 2;  cmd_state = 2;
        cmd_last_tick = HAL_GetTick();  handled = 1;
      } else if (ch == 'S' || ch == 's') {
        cmd_pending[1] = ch;  cmd_pending_cnt = 2;  cmd_state = 20;
        cmd_last_tick = HAL_GetTick();  handled = 1;
      } else if (ch == 'V' || ch == 'v') {
        cmd_pending[1] = ch;  cmd_pending_cnt = 2;  cmd_state = 30;
        cmd_last_tick = HAL_GetTick();  handled = 1;
      }
    }
    /* 状态 2 (!O): 等 'F'/'N' → !OFF/!ON */
    else if (cmd_state == 2) {
      if (ch == 'F') {
        cmd_pending[2] = ch;  cmd_pending_cnt = 3;  cmd_state = 3;
        cmd_last_tick = HAL_GetTick();  handled = 1;
      } else if (ch == 'N' || ch == 'n') {
        cmd_pending[2] = ch;  cmd_pending_cnt = 3;  cmd_state = 10;
        cmd_last_tick = HAL_GetTick();  handled = 1;
      }
    }
    /* 状态 3 (!OF): 等 'F' */
    else if (cmd_state == 3 && ch == 'F') {
      cmd_pending[3] = ch;  cmd_pending_cnt = 4;  cmd_state = 4;
      cmd_last_tick = HAL_GetTick();  handled = 1;
    }
    /* 状态 4/10: !OFF 或 !ON 等 \r \n */
    else if ((cmd_state == 4 || cmd_state == 10) && (ch == '\r' || ch == '\n'))
    {
      if (cmd_state == 4) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[CMD] !OFF running...\r\n", 24, 100);
        Turn_Off_ML307C();
      } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[CMD] !ON running...\r\n", 23, 100);
        Turn_On_ML307C();
      }
      if (ch == '\r' && rb_pc2gsm_tail != rb_pc2gsm_head) {
        uint8_t n = rb_pc2gsm[rb_pc2gsm_tail];
        if (n == '\n') rb_pc2gsm_tail = (rb_pc2gsm_tail + 1) % RB_SIZE;
      }
      cmd_state = 0;  cmd_pending_cnt = 0;  handled = 1;
    }
    /* 状态 20 (!S): 等 'L' */
    else if (cmd_state == 20 && (ch == 'L' || ch == 'l')) {
      cmd_pending[2] = ch;  cmd_pending_cnt = 3;  cmd_state = 21;
      cmd_last_tick = HAL_GetTick();  handled = 1;
    }
    /* 状态 21 (!SL): 等 'E' */
    else if (cmd_state == 21 && (ch == 'E' || ch == 'e')) {
      cmd_pending[3] = ch;  cmd_pending_cnt = 4;  cmd_state = 22;
      cmd_last_tick = HAL_GetTick();  handled = 1;
    }
    /* 状态 22 (!SLE): 等 'E' */
    else if (cmd_state == 22 && (ch == 'E' || ch == 'e')) {
      cmd_pending[4] = ch;  cmd_pending_cnt = 5;  cmd_state = 23;
      cmd_last_tick = HAL_GetTick();  handled = 1;
    }
    /* 状态 23 (!SLEE): 等 'P' */
    else if (cmd_state == 23 && (ch == 'P' || ch == 'p')) {
      cmd_pending[5] = ch;  cmd_pending_cnt = 6;  cmd_state = 24;
      cmd_last_tick = HAL_GetTick();  handled = 1;
    }
    /* 状态 24 (!SLEEP): 等 \r \n → 进入 Stop1 */
    else if (cmd_state == 24 && (ch == '\r' || ch == '\n'))
    {
      if (ch == '\r' && rb_pc2gsm_tail != rb_pc2gsm_head) {
        uint8_t n = rb_pc2gsm[rb_pc2gsm_tail];
        if (n == '\n') rb_pc2gsm_tail = (rb_pc2gsm_tail + 1) % RB_SIZE;
      }
      cmd_state = 0;  cmd_pending_cnt = 0;  handled = 1;
      /* Drain GSM buffer, then enter sleep */
      while (rb_gsm2pc_tail != rb_gsm2pc_head) {
        HAL_UART_Transmit(&huart2, &rb_gsm2pc[rb_gsm2pc_tail], 1, 10);
        rb_gsm2pc_tail = (rb_gsm2pc_tail + 1) % RB_SIZE;
      }
      HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n[CMD] !SLEEP Sleeping...\r\n", 27, 100);
      HAL_Delay(10);
      Enter_Stop1_Mode();
      g_sys_state = STATE_IDLE;
    }
    /* ---- 状态 30 (!V): 等 'B' ---- */
    else if (cmd_state == 30 && (ch == 'B' || ch == 'b')) {
      cmd_pending[2] = ch;  cmd_pending_cnt = 3;  cmd_state = 31;
      cmd_last_tick = HAL_GetTick();  handled = 1;
    }
    /* ---- 状态 31 (!VB): 等 'A' ---- */
    else if (cmd_state == 31 && (ch == 'A' || ch == 'a')) {
      cmd_pending[3] = ch;  cmd_pending_cnt = 4;  cmd_state = 32;
      cmd_last_tick = HAL_GetTick();  handled = 1;
    }
    /* ---- 状态 32 (!VBA): 等 'T' ---- */
    else if (cmd_state == 32 && (ch == 'T' || ch == 't')) {
      cmd_pending[4] = ch;  cmd_pending_cnt = 5;  cmd_state = 33;
      cmd_last_tick = HAL_GetTick();  handled = 1;
    }
    /* ---- 状态 33 (!VBAT): 等 \r \n → 发送电池电压 ---- */
    else if (cmd_state == 33 && (ch == '\r' || ch == '\n'))
    {
      if (ch == '\r' && rb_pc2gsm_tail != rb_pc2gsm_head) {
        uint8_t n = rb_pc2gsm[rb_pc2gsm_tail];
        if (n == '\n') rb_pc2gsm_tail = (rb_pc2gsm_tail + 1) % RB_SIZE;
      }
      cmd_state = 0;  cmd_pending_cnt = 0;  handled = 1;

      float vbat = ADC_Get_Battery_Voltage();
      uint8_t pct = ADC_Battery_Voltage_To_Percentage(vbat);
      char buf[64];
      int len;
      if (vbat < 0.0f) {
        len = snprintf(buf, sizeof(buf), "\r\n[VBAT] ADC read failed!\r\n");
      } else {
        /* 用整数避免 %f 依赖 _printf_float (未链接) */
        int v_mv = (int)(vbat * 1000.0f + 0.5f);
        len = snprintf(buf, sizeof(buf), "\r\n[VBAT] %d.%03dV  %d%%\r\n",
                       v_mv / 1000, v_mv % 1000, (int)pct);
      }
      HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 100);
    }

    if (!handled) {
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  __HAL_DBGMCU_FREEZE_IWDG();

  /* IMU + 双重复合门卫 */
  if (LSM6DS_Init(&hi2c2) == 1) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] IMU OK\r\n", 22, 100);
    if (LSM6DS_Config_Gatekeeper(GATEKEEPER_WU_MG, GATEKEEPER_6D_DEG) == 1) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] Gatekeeper OK\r\n", 20, 100);
    } else {
      HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] Gatekeeper FAIL\r\n", 22, 100);
    }
  } else {
    HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] IMU FAIL!\r\n", 22, 100);
  }

  /* 串口中断接收 */
  HAL_UART_Receive_IT(&huart1, &gsm_rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &pc_rx_byte,  1);

  /* 4G 引脚初始态 */
  HAL_GPIO_WritePin(LTE_PWRKEY_GPIO_Port, LTE_PWRKEY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LTE_RESET_GPIO_Port,  LTE_RESET_Pin,  GPIO_PIN_RESET);

  HAL_UART_Transmit(&huart2, (uint8_t*)"[SYS] Ready. Cmds: !ON !OFF !SLEEP\r\n", 40, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ORE 防锁死 */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
    }

    switch (g_sys_state)
    {

    /* ======== 状态 0：空闲 (透传+指令) ======== */
    case STATE_IDLE:
      Passthrough_Drain_With_Cmds();
      HAL_IWDG_Refresh(&hiwdg);
      break;

    /* ======== 状态 1：深度休眠 ======== */
    case STATE_SLEEP:
      Passthrough_Drain_With_Cmds();
      HAL_IWDG_Refresh(&hiwdg);
      Enter_Stop1_Mode();
      g_sys_state = STATE_IDLE;
      break;

    } /* switch */
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
  * @brief  Stop1 深度休眠 (IMU 双门卫唤醒 + GPIO 漏电切断 + 锁存清理)
  */
static void Enter_Stop1_Mode(void)
{
  /* 睡前：USART1 TX/RX 切模拟输入 (防漏电) */
  GPIO_InitTypeDef g = {0};
  g.Pin  = GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &g);

  /* ★关键★ 先读清 IMU 全部锁存 → INT1 回 LOW → 再清 EXTI → 才能捕获下次上升沿 */
  { uint8_t _wu, _6d; LSM6DS_Clear_All_Interrupts_Ex(&_wu, &_6d); }
  HAL_Delay(1);  /* 等 INT1 引脚物理放电 */
  __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT1_WAKEUP_Pin);
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  HAL_IWDG_Refresh(&hiwdg);
  HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

  /* ========== 醒来 ========== */
  HAL_IWDG_Refresh(&hiwdg);
  HAL_ResumeTick();
  SystemClock_Config();

  /* ★关键★ DeInit 强制重置 HAL 状态 → Init 才会真正恢复硬件 */
  HAL_UART_DeInit(&huart1);
  HAL_UART_DeInit(&huart2);
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  HAL_Delay(2);

  HAL_UART_Receive_IT(&huart1, &gsm_rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &pc_rx_byte,  1);

  /* I2C 唤醒后重初始化 (HAL gState 防跳) */
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    uint8_t next = (rb_gsm2pc_head + 1) % RB_SIZE;
    if (next != rb_gsm2pc_tail) {
      rb_gsm2pc[rb_gsm2pc_head] = gsm_rx_byte;
      rb_gsm2pc_head = next;
    }
    HAL_UART_Receive_IT(&huart1, &gsm_rx_byte, 1);
  }
  else if (huart->Instance == USART2) {
    uint8_t next = (rb_pc2gsm_head + 1) % RB_SIZE;
    if (next != rb_pc2gsm_tail) {
      rb_pc2gsm[rb_pc2gsm_head] = pc_rx_byte;
      rb_pc2gsm_head = next;
    }
    HAL_UART_Receive_IT(&huart2, &pc_rx_byte, 1);
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
