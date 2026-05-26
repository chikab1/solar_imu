#include "lsm6ds.h"

/* 静态全局驱动实例 */
static stmdev_ctx_t imu_ctx;
static const uint8_t imu_addr = (0x6A << 1);  /* 7位地址 0x6A → 8位 0xD4 */

/* I2C 平台桥接函数 —— 连接 lsm6ds3tr-c 驱动与 HAL 硬件 I2C */
static int32_t lsm6ds3tr_c_hal_write(void *handle, uint8_t reg,
                                      const uint8_t *buf, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(hi2c, imu_addr, reg,
                                              I2C_MEMADD_SIZE_8BIT,
                                              (uint8_t *)buf, len, 100);
    return (ret == HAL_OK) ? 0 : -1;
}

static int32_t lsm6ds3tr_c_hal_read(void *handle, uint8_t reg,
                                     uint8_t *buf, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, imu_addr, reg,
                                             I2C_MEMADD_SIZE_8BIT,
                                             buf, len, 100);
    return (ret == HAL_OK) ? 0 : -1;
}

/**
  * @brief  初始化 LSM6DS3TR-C 六轴传感器
  * @param  hi2c: HAL I2C 句柄指针
  * @retval 1 - 成功, 0 - 失败
  */
uint8_t LSM6DS_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t chip_id = 0;

    imu_ctx.write_reg = lsm6ds3tr_c_hal_write;
    imu_ctx.read_reg  = lsm6ds3tr_c_hal_read;
    imu_ctx.mdelay    = NULL;
    imu_ctx.handle    = hi2c;

    if (lsm6ds3tr_c_device_id_get(&imu_ctx, &chip_id) != 0) {
        return 0;
    }

    if (chip_id != LSM6DS3TR_C_ID) {
        return 0;
    }

    /* 软复位 */
    lsm6ds3tr_c_reset_set(&imu_ctx, PROPERTY_ENABLE);
    HAL_Delay(50);

    /* 加速度计：52Hz, ±2g */
    lsm6ds3tr_c_xl_data_rate_set(&imu_ctx, LSM6DS3TR_C_XL_ODR_52Hz);
    lsm6ds3tr_c_xl_full_scale_set(&imu_ctx, LSM6DS3TR_C_2g);

    /* 陀螺仪：52Hz, ±2000dps */
    lsm6ds3tr_c_gy_data_rate_set(&imu_ctx, LSM6DS3TR_C_GY_ODR_52Hz);
    lsm6ds3tr_c_gy_full_scale_set(&imu_ctx, LSM6DS3TR_C_2000dps);

    return 1;
}

/**
  * @brief  读取六轴传感器数据并转换为物理量
  * @param  acc_mg: 加速度输出数组 (mg)，长度 3 [X, Y, Z]
  * @param  gyro_dps: 陀螺仪输出数组 (dps)，长度 3 [X, Y, Z]
  * @retval 1 - 成功, 0 - 失败
  */
uint8_t LSM6DS_Read_Storage(float *acc_mg, float *gyro_dps)
{
    int16_t raw_acc[3];
    int16_t raw_gyr[3];

    if (lsm6ds3tr_c_acceleration_raw_get(&imu_ctx, raw_acc) != 0) {
        return 0;
    }

    if (lsm6ds3tr_c_angular_rate_raw_get(&imu_ctx, raw_gyr) != 0) {
        return 0;
    }

    /* ±2g 量程灵敏度 0.061 mg/LSB */
    acc_mg[0] = (float)raw_acc[0] * 0.061f;
    acc_mg[1] = (float)raw_acc[1] * 0.061f;
    acc_mg[2] = (float)raw_acc[2] * 0.061f;

    /* ±2000dps 量程灵敏度 70 mdps/LSB → 0.07 dps/LSB */
    gyro_dps[0] = (float)raw_gyr[0] * 0.07f;
    gyro_dps[1] = (float)raw_gyr[1] * 0.07f;
    gyro_dps[2] = (float)raw_gyr[2] * 0.07f;

    return 1;
}

/**
  * @brief  配置唤醒与运动检测
  * @param  threshold: 唤醒阈值（最低 6 位有效）
  * @param  duration: 唤醒持续时间
  * @retval None
  */
void LSM6DS_Config_Wakeup(uint8_t threshold, uint8_t duration)
{
    lsm6ds3tr_c_int1_route_t int1_route = {0};
    lsm6ds3tr_c_tap_cfg_t tap_cfg;

    /* 关键：打开全局嵌入式功能中断使能位（TAP_CFG bit7） */
    lsm6ds3tr_c_read_reg(&imu_ctx, LSM6DS3TR_C_TAP_CFG,
                         (uint8_t *)&tap_cfg, 1);
    tap_cfg.interrupts_enable = 1;
    lsm6ds3tr_c_write_reg(&imu_ctx, LSM6DS3TR_C_TAP_CFG,
                          (uint8_t *)&tap_cfg, 1);

    /* 读取当前 INT1 路由配置 */
    lsm6ds3tr_c_pin_int1_route_get(&imu_ctx, &int1_route);

    /* 使能 wake-up 中断路由到 INT1 */
    int1_route.int1_wu = 1;
    lsm6ds3tr_c_pin_int1_route_set(&imu_ctx, int1_route);

    /* 设置唤醒阈值（最低 6 位） */
    lsm6ds3tr_c_wkup_threshold_set(&imu_ctx, threshold & 0x3F);

    /* 设置唤醒持续时间 */
    lsm6ds3tr_c_wkup_dur_set(&imu_ctx, duration);
}
