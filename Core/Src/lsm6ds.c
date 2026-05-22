/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lsm6ds.c
  * @brief          : LSM6DSDTR six-axis sensor driver implementation
  ******************************************************************************
  */
/* USER CODE END Header */

#include "lsm6ds.h"
#include "i2c_sw.h"

/* Private variables ---------------------------------------------------------*/
// 静态全局变量存储确定的设备地址
static uint8_t lsm6ds_dev_addr = 0x00;

/* Private function prototypes -----------------------------------------------*/
static uint8_t LSM6DS_Write_Reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
static uint8_t LSM6DS_Read_Bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *pBuffer, uint16_t length);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  LSM6DS初始化函数
  * @note   自适应器件地址盲扫识别，配置核心寄存器
  * @retval 1 - 初始化成功，0 - 初始化失败
  */
uint8_t LSM6DS_Init(void) {
    uint8_t who_am_i;
    
    // 尝试识别设备地址（SDO接地 0x6A）
    I2C_Start();
    I2C_SendByte((LSM6DS_ADDR_SDO_GND << 1) | 0x00);  // 写地址
    if (I2C_WaitAck() == 0) {
        // 发送寄存器地址
        I2C_SendByte(LSM6DS_WHO_AM_I_REG);
        if (I2C_WaitAck() == 0) {
            // Repeated Start: 不发送Stop，直接再次Start
            I2C_Start();
            I2C_SendByte((LSM6DS_ADDR_SDO_GND << 1) | 0x01);  // 读地址
            if (I2C_WaitAck() == 0) {
                who_am_i = I2C_ReadByte(0);  // 最后一个字节回复NACK
                I2C_Stop();
                if (who_am_i == LSM6DS_WHO_AM_I_VALUE) {
                    lsm6ds_dev_addr = LSM6DS_ADDR_SDO_GND;
                }
            } else {
                I2C_Stop();
            }
        } else {
            I2C_Stop();
        }
    }
    
    // 如果第一个地址失败，尝试第二个地址（SDO接高电平 0x6B）
    if (lsm6ds_dev_addr == 0x00) {
        I2C_Start();
        I2C_SendByte((LSM6DS_ADDR_SDO_VCC << 1) | 0x00);  // 写地址
        if (I2C_WaitAck() == 0) {
            // 发送寄存器地址
            I2C_SendByte(LSM6DS_WHO_AM_I_REG);
            if (I2C_WaitAck() == 0) {
                // Repeated Start: 不发送Stop，直接再次Start
                I2C_Start();
                I2C_SendByte((LSM6DS_ADDR_SDO_VCC << 1) | 0x01);  // 读地址
                if (I2C_WaitAck() == 0) {
                    who_am_i = I2C_ReadByte(0);  // 最后一个字节回复NACK
                    I2C_Stop();
                    if (who_am_i == LSM6DS_WHO_AM_I_VALUE) {
                        lsm6ds_dev_addr = LSM6DS_ADDR_SDO_VCC;
                    }
                } else {
                    I2C_Stop();
                }
            } else {
                I2C_Stop();
            }
        }
    }
    
    // 未找到有效设备
    if (lsm6ds_dev_addr == 0x00) {
        return 0;
    }
    
    // 配置核心寄存器
    
    // CTRL3_C (0x12) -> 0x44: BDU=1, IF_INC=1
    if (LSM6DS_Write_Reg(lsm6ds_dev_addr, LSM6DS_CTRL3_C, 0x44) != 1) {
        return 0;
    }
    
    // CTRL1_XL (0x10) -> 0x44: ODR=104Hz, FS=±16g
    if (LSM6DS_Write_Reg(lsm6ds_dev_addr, LSM6DS_CTRL1_XL, 0x44) != 1) {
        return 0;
    }
    
    // CTRL2_G (0x11) -> 0x4C: ODR=104Hz, FS=±2000dps
    if (LSM6DS_Write_Reg(lsm6ds_dev_addr, LSM6DS_CTRL2_G, 0x4C) != 1) {
        return 0;
    }
    
    return 1;
}

/**
  * @brief  读取6轴原始数据
  * @param  raw_data: 原始数据结构体指针（输出）
  * @retval 1 - 读取成功，0 - 读取失败
  */
uint8_t LSM6DS_Read_Storage(LSM6DS_RawData_t *raw_data) {
    uint8_t buf[12];
    
    if (lsm6ds_dev_addr == 0x00) {
        return 0;
    }
    
    // 从OUTX_L_G开始连续读取12个字节
    if (LSM6DS_Read_Bytes(lsm6ds_dev_addr, LSM6DS_OUTX_L_G, buf, 12) != 1) {
        return 0;
    }
    
    // 组合16位有符号数据（低字节在前，高字节在后）
    raw_data->gx = (int16_t)((buf[1] << 8) | buf[0]);
    raw_data->gy = (int16_t)((buf[3] << 8) | buf[2]);
    raw_data->gz = (int16_t)((buf[5] << 8) | buf[4]);
    raw_data->ax = (int16_t)((buf[7] << 8) | buf[6]);
    raw_data->ay = (int16_t)((buf[9] << 8) | buf[8]);
    raw_data->az = (int16_t)((buf[11] << 8) | buf[10]);
    
    return 1;
}

/**
  * @brief  原始数据转换为物理量
  * @param  raw: 原始数据结构体指针（输入）
  * @param  out: 物理量数据结构体指针（输出）
  * @retval None
  */
void LSM6DS_Data_Convert(const LSM6DS_RawData_t *raw, LSM6DS_FloatData_t *out) {
    // 加速度量程 ±16g，灵敏度 0.488 mg/LSB
    out->acc_g[0] = (float)raw->ax * 0.488f / 1000.0f;
    out->acc_g[1] = (float)raw->ay * 0.488f / 1000.0f;
    out->acc_g[2] = (float)raw->az * 0.488f / 1000.0f;
    
    // 陀螺仪量程 ±2000 dps，灵敏度 70 mdps/LSB
    out->gyro_dps[0] = (float)raw->gx * 70.0f / 1000.0f;
    out->gyro_dps[1] = (float)raw->gy * 70.0f / 1000.0f;
    out->gyro_dps[2] = (float)raw->gz * 70.0f / 1000.0f;
}

/**
  * @brief  配置唤醒与运动检测
  * @param  threshold: 唤醒阈值（最低6位有效）
  * @param  duration: 唤醒持续时间
  * @retval None
  */
void LSM6DS_Config_Wakeup(uint8_t threshold, uint8_t duration) {
    if (lsm6ds_dev_addr == 0x00) {
        return;
    }
    
    // TAP_CFG0 (0x56) -> 0x80: 使能中断全局
    LSM6DS_Write_Reg(lsm6ds_dev_addr, LSM6DS_TAP_CFG0, 0x80);
    
    // WAKE_UP_THS (0x5B) -> 写入阈值（最低6位）
    LSM6DS_Write_Reg(lsm6ds_dev_addr, LSM6DS_WAKE_UP_THS, threshold & 0x3F);
    
    // WAKE_UP_DUR (0x5C) -> 写入持续时间限制
    LSM6DS_Write_Reg(lsm6ds_dev_addr, LSM6DS_WAKE_UP_DUR, duration);
    
    // MD1_CFG (0x5E) -> 0x20: Wake-up事件路由至INT1，高电平有效
    LSM6DS_Write_Reg(lsm6ds_dev_addr, LSM6DS_MD1_CFG, 0x20);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  向指定寄存器写入一个字节
  * @param  dev_addr: 设备地址
  * @param  reg_addr: 寄存器地址
  * @param  data: 要写入的数据
  * @retval 1 - 写入成功，0 - 写入失败
  */
static uint8_t LSM6DS_Write_Reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    I2C_Start();
    I2C_SendByte((dev_addr << 1) | 0x00);  // 写地址
    if (I2C_WaitAck() != 0) {
        I2C_Stop();
        return 0;
    }
    
    I2C_SendByte(reg_addr);
    if (I2C_WaitAck() != 0) {
        I2C_Stop();
        return 0;
    }
    
    I2C_SendByte(data);
    if (I2C_WaitAck() != 0) {
        I2C_Stop();
        return 0;
    }
    
    I2C_Stop();
    return 1;
}

/**
  * @brief  从指定寄存器连续读取多个字节
  * @note   严格执行Repeated Start时序：发送reg_addr后不发送Stop，直接再次Start并发送读地址
  * @param  dev_addr: 设备地址
  * @param  reg_addr: 寄存器地址
  * @param  pBuffer: 数据缓冲区指针（输出）
  * @param  length: 要读取的字节数
  * @retval 1 - 读取成功，0 - 读取失败
  */
static uint8_t LSM6DS_Read_Bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *pBuffer, uint16_t length) {
    uint16_t i;
    
    // 发送起始信号和写地址
    I2C_Start();
    I2C_SendByte((dev_addr << 1) | 0x00);  // 写地址
    if (I2C_WaitAck() != 0) {
        I2C_Stop();
        return 0;
    }
    
    // 发送寄存器地址
    I2C_SendByte(reg_addr);
    if (I2C_WaitAck() != 0) {
        I2C_Stop();
        return 0;
    }
    
    // Repeated Start: 不发送Stop，直接再次Start
    I2C_Start();
    I2C_SendByte((dev_addr << 1) | 0x01);  // 读地址
    if (I2C_WaitAck() != 0) {
        I2C_Stop();
        return 0;
    }
    
    // 读取数据
    for (i = 0; i < length; i++) {
        // 最后一个字节回复NACK，其余回复ACK
        pBuffer[i] = I2C_ReadByte((i < length - 1) ? 1 : 0);
    }
    
    I2C_Stop();
    return 1;
}
