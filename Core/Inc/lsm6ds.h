/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lsm6ds.h
  * @brief          : Header for lsm6ds.c file.
  *                   This file contains LSM6DSDTR six-axis sensor driver definitions.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __LSM6DS_H
#define __LSM6DS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private defines -----------------------------------------------------------*/
// LSM6DSDTR寄存器地址映射
#define LSM6DS_WHO_AM_I_REG      0x0F
#define LSM6DS_CTRL1_XL          0x10  // 加速度计控制
#define LSM6DS_CTRL2_G           0x11  // 陀螺仪控制
#define LSM6DS_CTRL3_C           0x12  // 基础配置(BDU, 自增等)
#define LSM6DS_STATUS_REG        0x1E  // 状态寄存器
#define LSM6DS_OUTX_L_G          0x22  // 陀螺仪X轴低字节(连续12字节输出起点)
#define LSM6DS_WAKE_UP_SRC       0x1B  // 唤醒源寄存器
#define LSM6DS_TAP_CFG0          0x56  // 中断全局配置
#define LSM6DS_TAP_CFG2          0x58  // 中断使能
#define LSM6DS_WAKE_UP_THS       0x5B  // 唤醒阈值
#define LSM6DS_WAKE_UP_DUR       0x5C  // 唤醒持续时间
#define LSM6DS_MD1_CFG           0x5E  // INT1路由配置

// WHO_AM_I验证码
#define LSM6DS_WHO_AM_I_VALUE    0x6C

// I2C设备地址（SDO引脚决定）
#define LSM6DS_ADDR_SDO_GND      0x6A  // SDO接地
#define LSM6DS_ADDR_SDO_VCC      0x6B  // SDO接高电平

/* Exported types ------------------------------------------------------------*/
// 原始数据结构体
typedef struct {
    int16_t ax;  // 加速度X轴原始值
    int16_t ay;  // 加速度Y轴原始值
    int16_t az;  // 加速度Z轴原始值
    int16_t gx;  // 陀螺仪X轴原始值
    int16_t gy;  // 陀螺仪Y轴原始值
    int16_t gz;  // 陀螺仪Z轴原始值
} LSM6DS_RawData_t;

// 物理量数据结构体
typedef struct {
    float acc_g[3];      // 加速度（g）
    float gyro_dps[3];   // 角速度（dps）
} LSM6DS_FloatData_t;

/* Exported functions prototypes ---------------------------------------------*/
uint8_t LSM6DS_Init(void);
uint8_t LSM6DS_Read_Storage(LSM6DS_RawData_t *raw_data);
void LSM6DS_Data_Convert(const LSM6DS_RawData_t *raw, LSM6DS_FloatData_t *out);
void LSM6DS_Config_Wakeup(uint8_t threshold, uint8_t duration);

#ifdef __cplusplus
}
#endif

#endif /* __LSM6DS_H */
