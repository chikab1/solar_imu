/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    at_ml307c.c
  * @brief   优化版中移 OneMO ML307C 4G 模组驱动 (生产级)
  * @date    2026-07-08
  ******************************************************************************
  */
/* USER CODE END Header */

#include "at_ml307c.h"
#include "main.h"
#include "iwdg.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static uint8_t  s_uart_rx_buf[ML307C_MAX_BUF_SIZE];
static uint16_t s_rx_buf_len = 0;
static char     s_at_tx_buf[256];  /* 静态复用, 防栈溢出 */

/* Private function prototypes -----------------------------------------------*/
static int ML307C_Receive_Data(uint32_t timeout_ms);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  发送 AT 指令并等待期望响应 (非阻塞流接收, IWDG 安全)
  */
int ML307C_Send_CMD(char *cmd, char *expected_resp, uint32_t timeout_ms)
{
    uint32_t start_tick;

    if (cmd == NULL || expected_resp == NULL) return -1;

    ML307C_Clear_Buffer();
    snprintf(s_at_tx_buf, sizeof(s_at_tx_buf), "%s\r\n", cmd);

    if (HAL_UART_Transmit(ML307C_UART_HANDLE, (uint8_t *)s_at_tx_buf,
                          strlen(s_at_tx_buf), 500) != HAL_OK) {
        return -1;
    }

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        HAL_IWDG_Refresh(&hiwdg);
        if (ML307C_Receive_Data(10) > 0) {
            if (strstr((char *)s_uart_rx_buf, expected_resp) != NULL) return 1;
            if (strstr((char *)s_uart_rx_buf, "ERROR")   != NULL) return 0;
        }
    }
    return 0;
}

/**
  * @brief  模组初始化与 4G 网络附着检查
  */
int ML307C_Network_Init(ML307C_Network_Status_t *status)
{
    if (ML307C_Send_CMD("AT", "OK", 2000) != 1) return 0;
    ML307C_Send_CMD("ATE0", "OK", 1000);  /* 关回显, 省中断 */

    if (ML307C_Send_CMD("AT+CPIN?", "+CPIN: READY", 3000) != 1) return 0;

    if (ML307C_Send_CMD("AT+CSQ", "OK", 3000) != 1) return 0;
    if (status != NULL) {
        char *ptr = strstr((char *)s_uart_rx_buf, "+CSQ:");
        status->csq = (ptr && sscanf(ptr, "+CSQ: %d", &status->csq) == 1)
                      ? status->csq : 99;
    }

    if (ML307C_Send_CMD("AT+CGATT?", "+CGATT: 1", 5000) != 1) return 0;
    if (status != NULL) status->is_attached = 1;

    return 1;
}

/**
  * @brief  连接 MQTT 服务器 (腾讯云 EMQX)
  */
int ML307C_MQTT_Connect(char *broker_url, int port, char *username, char *password)
{
    if (ML307C_Send_CMD("AT+MQTTCFG=\"clean\",0,1",     "OK", 2000) != 1) return 0;
    if (ML307C_Send_CMD("AT+MQTTCFG=\"keepalive\",0,120","OK", 2000) != 1) return 0;

    snprintf(s_at_tx_buf, sizeof(s_at_tx_buf),
             "AT+MQTTCONN=0,\"%s\",%d,\"stm32_road\",\"%s\",\"%s\"",
             broker_url, port, username, password);

    return ML307C_Send_CMD(s_at_tx_buf, "+MQTTURC: \"conn\",0,0", 12000);
}

/**
  * @brief  发布 MQTT 消息
  */
int ML307C_MQTT_Publish(char *topic, char *payload)
{
    int len = (int)strlen(payload);
    if (topic == NULL || payload == NULL || len <= 0) return 0;

    snprintf(s_at_tx_buf, sizeof(s_at_tx_buf),
             "AT+MQTTPUB=0,\"%s\",0,0,0,%d,\"%s\"", topic, len, payload);

    return ML307C_Send_CMD(s_at_tx_buf, "OK", 5000);
}

/**
  * @brief  纯整数 JSON 上报 (杜绝 %f, 省 Flash)
  */
int ML307C_Send_CustomData(int16_t voltage_x100, int16_t tilt_x100, char *topic)
{
    char json[96];
    int v_abs = (voltage_x100 >= 0) ? voltage_x100 : -voltage_x100;
    int t_abs = (tilt_x100   >= 0) ? tilt_x100   : -tilt_x100;

    snprintf(json, sizeof(json),
             "{\"voltage\":%d.%02d,\"tilt\":%d.%02d}",
             voltage_x100 / 100, v_abs % 100,
             tilt_x100   / 100, t_abs % 100);

    return ML307C_MQTT_Publish(topic, json);
}

char* ML307C_Get_RxBuffer(void)
{
    return (char *)s_uart_rx_buf;
}

void ML307C_Clear_Buffer(void)
{
    memset(s_uart_rx_buf, 0, sizeof(s_uart_rx_buf));
    s_rx_buf_len = 0;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  工业级空闲断帧接收 (防单字节退出导致误匹配)
  */
static int ML307C_Receive_Data(uint32_t timeout_ms)
{
    uint8_t  rx_byte;
    uint32_t start_tick = HAL_GetTick();
    uint16_t prev_len   = s_rx_buf_len;

    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        if (HAL_UART_Receive(ML307C_UART_HANDLE, &rx_byte, 1, 2) == HAL_OK) {
            if (s_rx_buf_len < (ML307C_MAX_BUF_SIZE - 1)) {
                s_uart_rx_buf[s_rx_buf_len++] = rx_byte;
                s_uart_rx_buf[s_rx_buf_len]   = '\0';
            }
            start_tick = HAL_GetTick();  /* 刷新空闲窗 */
        } else {
            if (s_rx_buf_len > prev_len) {
                HAL_Delay(5);  /* 等最后的 \r\n 落盘 */
                return (s_rx_buf_len - prev_len);
            }
        }
    }
    return 0;
}

/* USER CODE BEGIN 1 */

void Turn_On_ML307C(void)
{
    HAL_GPIO_WritePin(LTE_PWRKEY_GPIO_Port, LTE_PWRKEY_Pin, GPIO_PIN_SET);
    HAL_Delay(1500);
    HAL_GPIO_WritePin(LTE_PWRKEY_GPIO_Port, LTE_PWRKEY_Pin, GPIO_PIN_RESET);
}

void Turn_Off_ML307C(void)
{
    char *cmd = "AT+MPOF=0\r\n";
    HAL_UART_Transmit(ML307C_UART_HANDLE, (uint8_t *)cmd, strlen(cmd), 100);

    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(LTE_STATE_GPIO_Port, LTE_STATE_Pin) == GPIO_PIN_SET) {
        HAL_IWDG_Refresh(&hiwdg);
        if (HAL_GetTick() - t0 > 5000U) {
            HAL_GPIO_WritePin(LTE_RESET_GPIO_Port, LTE_RESET_Pin, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(LTE_RESET_GPIO_Port, LTE_RESET_Pin, GPIO_PIN_RESET);
            break;
        }
    }
}

/* USER CODE END 1 */
