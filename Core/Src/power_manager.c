/**
  * @file    power_manager.c
  * @brief   电池电压采集与低电量熔断保护
  */

#include "power_manager.h"
#include "main.h"
#include "adc.h"
#include "sys_config.h"

/* ========================== 全局变量 ========================== */

/* 低电量熔断状态机 (回线迟滞保护)
 * 硬熔断阈值: g_cfg.v_low_mv - 禁止启动4G模组 (防止200~300mA射频脉冲砸出Brownout)
 * 回线恢复阈值: 熔断阈值+200mV - 太阳能充回安全水位后才允许重新开机
 * g_low_volt_fuse == 0: 正常 (允许开机)
 * g_low_volt_fuse == 1: 熔断 (禁止开机, 等待充电恢复) */
uint8_t g_low_volt_fuse = 0; /**< 1表示电压未恢复到迟滞上限，禁止启动4G。 */

/* ========================== 函数实现 ========================== */

/**
  * @brief  电压过采样均值滤波 (16次ADC取均值, 抗瞬间毛刺)
  * @retval 均值电压 (V), 如 3.27f
  */
float ADC_Get_Battery_Voltage_Avg(void)
{
    float sum = 0.0f;
    for (int i = 0; i < 16; i++)
        sum += ADC_Get_Battery_Voltage();
    return sum / 16.0f;
}

/**
  * @brief  低电量熔断检查 (三级硬熔断与回线迟滞)
  * @param  v_avg: 过采样均值电压 (V)
  * @retval 1: 允许启动4G模组, 0: 电压过低禁止启动
  * @note   熔断后即使电压回升到3.3~3.5V之间也不解除,
  *         必须充回3.5V以上才解除封印 (迟滞区间=0.2V)
  */
uint8_t Volt_Fuse_Check(float v_avg)
{
    float cutoff = g_cfg.v_low_mv / 1000.0f;
    float hyst   = (g_cfg.v_low_mv + 200) / 1000.0f;
    if (g_low_volt_fuse == 0) {
        if (v_avg < cutoff) {
            g_low_volt_fuse = 1;
            return 0;
        }
        return 1;
    } else {
        if (v_avg >= hyst) {
            g_low_volt_fuse = 0;
            return 1;
        }
        return 0;
    }
}
