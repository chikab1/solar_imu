/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : i2c_sw.c
  * @brief          : Software I2C driver implementation for SHT30 sensor
  ******************************************************************************
  */
/* USER CODE END Header */

#include "i2c_sw.h"

/**
  * @brief  软件I2C延时函数
  * @note   适配STM32G0 64MHz，产生约10us延时，使I2C时钟频率在50K-100KHz
  * @retval None
  */
void I2C_Delay(void)
{
    volatile uint8_t i;
    for(i = 0; i < 60; i++);  // 约10us延时，确保50K-100KHz时钟频率
}

/**
  * @brief  I2C起始信号
  * @note   SCL高电平时，SDA由高变低
  * @retval None
  */
void I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_Delay();
    SDA_L;
    I2C_Delay();
    SCL_L;
    I2C_Delay();
}

/**
  * @brief  I2C停止信号
  * @note   SCL高电平时，SDA由低变高
  * @retval None
  */
void I2C_Stop(void)
{
    SDA_L;
    SCL_H;
    I2C_Delay();
    SDA_H;
    I2C_Delay();
}

/**
  * @brief  I2C发送一个字节
  * @param  byte: 要发送的数据
  * @retval None
  */
void I2C_SendByte(uint8_t byte)
{
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        SCL_L;
        I2C_Delay();
        
        if(byte & 0x80)  // 发送最高位
            SDA_H;
        else
            SDA_L;
        
        byte <<= 1;      // 左移一位
        SCL_H;
        I2C_Delay();
    }
    SCL_L;
    I2C_Delay();
}

/**
  * @brief  I2C等待应答信号
  * @retval 0 - 收到ACK应答，1 - 无应答
  */
uint8_t I2C_WaitAck(void)
{
    uint8_t ack = 1; 
    
    SCL_L;       // 确保在 SCL 低电平期间操作
    SDA_H;       // 单片机开漏释放 SDA 总线
    I2C_Delay(); // 留出充足时间让总线电平回弹，并让 SHT30 有机会准备好应答
    
    SCL_H;       // 主机拉高时钟线，此时总线电平绝对稳定
    I2C_Delay(); // 稳定等待
    
    if(READ_SDA == GPIO_PIN_RESET)  // 此时安全采样：SDA被拉低表示收到真实的ACK
    {
        ack = 0;
    }
    
    SCL_L;       // 钳住时钟，为下一帧通信做准备
    I2C_Delay();
    
    return ack;
}

/**
  * @brief  SHT30设备检测函数
  * @note   通过发送写地址检查设备是否存在
  * @retval 1 - 检测到设备（收到ACK），0 - 未检测到设备（无ACK）
  */
int SHT30_Check_Device(void)
{
    I2C_Start();
    I2C_SendByte(0x88);  // SHT30的7位写地址0x44左移一位得到0x88
    if (I2C_WaitAck() == 0) {
        I2C_Stop();
        return 1;  // 证明总线上有这个设备，且它给单片机回了ACK！
    }
    I2C_Stop();
    return 0;  // 无应答，硬件连接有问题或时序不对
}

/**
  * @brief  产生ACK应答
  * @retval None
  */
void I2C_Ack(void) {
    SCL_L; I2C_Delay(); SDA_L; I2C_Delay(); SCL_H; I2C_Delay(); SCL_L; I2C_Delay();
}

/**
  * @brief  产生NACK非应答
  * @retval None
  */
void I2C_NAck(void) {
    SCL_L; I2C_Delay(); SDA_H; I2C_Delay(); SCL_H; I2C_Delay(); SCL_L; I2C_Delay();
}

/**
  * @brief  读取一个字节
  * @param  ack_mode: 1发送ACK，0发送NACK
  * @retval 读取到的数据
  */
uint8_t I2C_ReadByte(uint8_t ack_mode) {
    uint8_t i, receive = 0;
    SDA_H; // 释放数据线
    for(i=0; i<8; i++ ) {
        SCL_L; I2C_Delay(); SCL_H; I2C_Delay();
        receive <<= 1;
        if(READ_SDA == GPIO_PIN_SET) receive++;
    }
    if (!ack_mode) I2C_NAck(); else I2C_Ack();
    return receive;
}
