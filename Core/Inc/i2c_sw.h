/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : i2c_sw.h
  * @brief          : Header for i2c_sw.c file.
  *                   This file contains software I2C driver definitions.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __I2C_SW_H
#define __I2C_SW_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private defines -----------------------------------------------------------*/
// I2C引脚控制宏
#define SCL_H  HAL_GPIO_WritePin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin, GPIO_PIN_SET)
#define SCL_L  HAL_GPIO_WritePin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin, GPIO_PIN_RESET)
#define SDA_H  HAL_GPIO_WritePin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin, GPIO_PIN_SET)
#define SDA_L  HAL_GPIO_WritePin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin, GPIO_PIN_RESET)
#define READ_SDA HAL_GPIO_ReadPin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin)

/* Exported functions prototypes ---------------------------------------------*/
// I2C基础时序函数
void I2C_Delay(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t byte);
uint8_t I2C_WaitAck(void); // 返回 0 代表收到ACK，返回 1 代表无应答
void I2C_Ack(void);        // 产生ACK应答
void I2C_NAck(void);       // 产生NACK非应答
uint8_t I2C_ReadByte(uint8_t ack_mode); // 读取一个字节，ack_mode: 1发送ACK，0发送NACK

#ifdef __cplusplus
}
#endif

#endif /* __I2C_SW_H */