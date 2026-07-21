#ifndef __LSM6DS_H
#define __LSM6DS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lsm6ds3tr-c_reg.h"
#include "i2c.h"
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/
uint8_t LSM6DS_Init(I2C_HandleTypeDef *hi2c);
uint8_t LSM6DS_Read_Storage(float *acc_mg, float *gyro_dps);
uint8_t LSM6DS_Set_Active_Mode(void);
uint8_t LSM6DS_Set_Sleep_Mode(void);
uint8_t LSM6DS_Config_Wakeup(uint8_t threshold, uint8_t duration);
uint8_t LSM6DS_Clear_Wakeup(void);

/**
  * @brief  重力基准互补滤波倾角解算（无需校准，上电即用）
  * @note   核心创新：以地球重力方向 (0,0,1) 为天然绝对基准，不再需要
  *         上电"去皮"校准。加速度计每时每刻用重力真相拉回陀螺漂移，
  *         滤波器在上电后数百毫秒内自动收敛到真实角度。
  *
  *         算法四部曲：
  *          1. 陀螺外积旋转 → dR = ω × R_est × dt
  *          2. 加速度计 2% 权重拉回估计向量 → 长效锁死重力方向
  *          3. 重归一化防浮点缩水
  *          4. acos(R_est · (0,0,1)) → 丝滑无漂移总倾角
  *
  * @param  ax, ay, az:     当前三轴加速度（单位：g）
  * @param  gx, gy, gz:     当前三轴角速度（单位：° / s）
  * @param  dt:              本次循环真实时间步长（单位：秒）
  * @param  out_raw_tilt:   输出：加速度计即时重力倾角 (°)，零延迟含噪
  * @param  out_fused_tilt: 输出：互补滤波融合倾角 (°)，丝滑无漂移
  * @retval 无
  */
void LSM6DS_Complementary_Tilt_Update(float ax, float ay, float az,
                                       float gx, float gy, float gz, float dt,
                                       float *out_raw_tilt,
                                       float *out_fused_tilt);

/**
  * @brief  一键读取重力倾角（纯加速度计，最简调用）
  * @note   内部自动完成 IMU 读取 + mg→g 换算 + acos(az/|g|) 倾角解算。
  *         无需校准、无需传参，一行代码拿到角度。
  * @retval 重力方向总倾角 (°)，范围 0~180。读失败返回 -1.0f
  */
float LSM6DS_Get_Tilt_Angle(void);

/**
  * @brief  配置 IMU 6D/4D 方向检测唤醒 (阈值 ~10° 倾角即触发)
  * @note   6D 检测：当设备从静止姿态倾斜超过阈值角度时，INT1 引脚拉高，
  *         可用于将 STM32 从 Stop 模式唤醒。无参数，阈值固定为最灵敏档。
  * @retval 1-成功, 0-失败
  */
uint8_t LSM6DS_Config_6D_Wakeup(void);

/**
  * @brief  清除 IMU 6D 唤醒中断标志
  * @retval 1-有唤醒事件, 0-无事件
  */
uint8_t LSM6DS_Clear_6D_Wakeup(void);

/**
  * @brief  工业级双重复合门卫：WAKE-UP + 6D 双中断源同时路由到 INT1
  * @note   通道一 (int1_wu)：运动唤醒，阈值=2 (62.5mg)，秒杀暴力破坏
  *         通道二 (int1_6d)：6D姿态检测，DEG_60 (30°边界)，防慢速偷盗
  *         强关4D模式防止贴地平躺Z轴盲区漏报
  *         中断锁存模式 (LATCHED)，确保上升沿万无一失
  * @retval 1-成功, 0-失败
  */
/**
  * @brief  双门卫配置：WAKE-UP + 6D 同时路由 INT1 (用户填入物理量，内部自动映射)
  *
  * @param  wu_mg:   WAKE-UP 加速度阈值，单位 mg (毫克)
  *                  填多少 mg 就约在多少 mg 的冲击下唤醒。
  *                  可填范围: 0 ~ 1968，步进 ~31.25mg (硬件 0~63 级)。
  *                  例如: 填 100  → 约 100mg 唤醒 (轻敲)
  *                        填 300  → 约 300mg 唤醒 (用力敲)
  *                        填 500  → 约 500mg 唤醒 (摔落级别)
  *                  实际值会就近取整到硬件支持的档位。
  *
  * @param  deg_6d:  6D 倾角唤醒阈值，单位 ° (度)
  *                  填多少度，倾斜超过该角度即唤醒。
  *                  硬件仅支持 4 档 (~10° / ~20° / ~30° / ~40°)，
  *                  填入值会自动映射到最近的一档:
  *                    0°~15°  → ~10° (最灵敏，轻微倾斜即醒)
  *                   16°~25° → ~20°
  *                   26°~35° → ~30°
  *                   36°~90° → ~40° (最迟钝，需大幅度倾斜)
  *                  例如: 填 10 → ~10° 唤醒
  *                        填 30 → ~30° 唤醒
  *                        填 80 → ~40° 唤醒 (超过最大档，封顶)
  *
  * @retval 1-成功, 0-失败
  */
uint8_t LSM6DS_Config_Gatekeeper(uint16_t wu_mg, uint8_t deg_6d);
uint8_t LSM6DS_Get_Gatekeeper_Diag(uint8_t out_regs[8]);

/**
  * @brief  读清 IMU 全部中断源锁存 (WAKE_UP + D6D + TAP 等全部源寄存器)
  * @note   必须调用此函数来一次性解锁 INT1 引脚，否则下次进 Stop 无法再唤醒
  * @retval 1-存在任意中断事件, 0-无事件
  */
void LSM6DS_Clear_All_Interrupts_Ex(uint8_t *out_wu, uint8_t *out_6d);

#ifdef __cplusplus
}
#endif

#endif /* __LSM6DS_H */
