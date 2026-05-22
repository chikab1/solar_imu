/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    at_ml307c.h
  * @brief   中移 OneMO ML307C-GC-CN 4G/GPS 模组 AT 指令驱动头文件
  * @author  Embedded Architect
  * @date    2026-05-18
  * @version V1.1.0
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __AT_ML307C_H__
#define __AT_ML307C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* Exported constants --------------------------------------------------------*/
#define ML307C_UART_HANDLE    (&huart1)  /* UART1 句柄 */
#define ML307C_MAX_BUF_SIZE   (512)      /* 最大接收缓冲区大小 */
#define ML307C_DEFAULT_TIMEOUT (3000)    /* 默认超时时间(ms) */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief GPS 解析后数据结构体
 */
typedef struct {
    float latitude;     // 纬度 (度)，北纬为正，南纬为负
    float longitude;    // 经度 (度)，东经为正，西经为负
    int satellites;     // 卫星数量
    int is_fixed;       // 是否定位成功 (1-成功, 0-失败)
} ML307C_GPS_Data_t;

/**
 * @brief 网络状态结构体
 */
typedef struct {
    int csq;           // 信号强度值 (0-31, 99表示未知)
    int is_attached;   // 是否附着网络 (1-已附着, 0-未附着)
} ML307C_Network_Status_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  发送 AT 指令并等待期望响应（带超时）
 * @param  cmd: 要发送的 AT 指令（不含 "\r\n"）
 * @param  expected_resp: 期望的响应字符串（如 "OK"、"+CPIN: READY"）
 * @param  timeout_ms: 超时时间（毫秒）
 * @retval 1-匹配成功，0-超时或匹配失败，-1-发送失败
 */
int ML307C_Send_CMD(char *cmd, char *expected_resp, uint32_t timeout_ms);

/**
 * @brief  模组初始化与网络检查
 * @param  status: 输出网络状态信息（可选，传NULL则不输出）
 * @retval 1-初始化成功，0-失败
 */
int ML307C_Network_Init(ML307C_Network_Status_t *status);

/**
 * @brief  GPS 引擎配置与启动
 * @retval 1-启动成功，0-失败
 */
int ML307C_GPS_Start(void);

/**
 * @brief  GPS 数据解析函数
 * @param  uart_rx_buf: UART 接收缓冲区
 * @param  gps_data: 输出 GPS 数据结构体
 * @retval 1-解析成功，0-未找到有效数据，-1-解析失败
 */
int ML307C_GPS_Parse(char *uart_rx_buf, ML307C_GPS_Data_t *gps_data);

/**
 * @brief  连接公网 MQTT 服务器
 * @param  broker_url: 服务器域名 (如 "broker.emqx.io")
 * @param  port: 端口号 (如 1883)
 * @retval 1-连接成功，0-失败
 */
int ML307C_MQTT_Connect(char *broker_url, int port, char *username, char *password);

/**
 * @brief  发布 MQTT 消息到指定主题
 * @param  topic: 目标主题字符串
 * @param  payload: 要发送的文本内容（长度由内部自动计算）
 * @retval 1-发布成功，0-失败
 */
int ML307C_MQTT_Publish(char *topic, char *payload);

/**
 * @brief  获取接收缓冲区指针（供外部读取）
 * @retval 缓冲区指针
 */
char* ML307C_Get_RxBuffer(void);

/**
 * @brief  清空 UART 接收缓冲区
 * @retval None
 */
void ML307C_Clear_Buffer(void);

#ifdef __cplusplus
}
#endif

#endif /* __AT_ML307C_H__ */