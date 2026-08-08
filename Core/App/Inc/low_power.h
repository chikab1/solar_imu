#ifndef __LOW_POWER_H__
#define __LOW_POWER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 全局变量声明 — RTC/Stop诊断 */
extern uint8_t  g_rtc_timer_active;
extern uint16_t g_rtc_arm_count;
extern uint16_t g_rtc_last_interval;
extern uint32_t g_rtc_last_cr;
extern uint32_t g_rtc_last_sr;
extern uint32_t g_rtc_last_requested_sleep;
extern uint16_t g_rtc_consumed_count;
extern uint16_t g_rtc_ready_count;
extern uint8_t  g_rtc_arm_status;
extern uint8_t  g_rtc_deactivate_status;

/* 全局变量声明 — IMU源诊断 */
extern uint16_t g_imu_wu_source_count;
extern uint16_t g_imu_6d_source_count;
extern uint16_t g_imu_both_source_count;

/* 全局变量声明 — IMU/唤醒累计诊断 (跨模块访问) */
extern uint16_t g_rtc_hw_wake_count;
extern uint8_t  g_imu_source_fallback_count;

/* 全局变量声明 — 休眠累积 */
extern uint32_t g_guard_sleep_accum_sec;

/* 函数原型 */
uint8_t IMU_Drain_INT1_Latch(uint8_t *out_wu, uint8_t *out_6d);
void    Enter_Stop1_Mode(uint8_t *out_wu, uint8_t *out_6d,
                         uint8_t *out_rtc, uint8_t *out_uart,
                         uint8_t *out_source_fallback);

#ifdef __cplusplus
}
#endif

#endif /* __LOW_POWER_H__ */
