#ifndef __POWER_MANAGER_H__
#define __POWER_MANAGER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 全局变量声明 */
extern uint8_t g_low_volt_fuse;

/* 函数原型 */
float   ADC_Get_Battery_Voltage_Avg(void);
uint8_t Volt_Fuse_Check(float v_avg);

#ifdef __cplusplus
}
#endif

#endif /* __POWER_MANAGER_H__ */
