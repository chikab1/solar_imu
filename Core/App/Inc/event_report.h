#ifndef __EVENT_REPORT_H__
#define __EVENT_REPORT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "event_store.h"

/* 全局变量声明 */
extern uint32_t g_volatile_event_seq;
extern uint8_t  g_last_report_ok;
extern uint8_t  g_last_report_stage;
extern uint8_t  g_last_report_fail;
extern uint8_t  g_last_report_csq;
extern uint8_t  g_last_report_attached;
extern uint32_t g_last_report_duration_ms;
extern uint32_t g_last_imu_event_time;
extern uint8_t  g_last_imu_class;
extern uint8_t  g_last_imu_wake;
extern uint16_t g_imu_false_wake_count;

/* 工具函数（服务协议使用） */
uint16_t Read_LE16(const uint8_t *p);
uint32_t Read_LE32(const uint8_t *p);
void     Write_LE16(uint8_t *p, uint16_t value);
void     Write_LE32(uint8_t *p, uint32_t value);

/* 核心上报链路 */
uint8_t  Capture_Event_And_Start_Modem(EventRecord_t *event,
                                       uint8_t start_modem,
                                       uint16_t *out_dynamic_peak_mg);
uint8_t  Run_Event_Report(uint8_t wu_flag, uint8_t d6d_flag,
                          uint8_t rtc_flag, uint8_t manual,
                          uint8_t source_fallback);

#ifdef __cplusplus
}
#endif

#endif /* __EVENT_REPORT_H__ */
