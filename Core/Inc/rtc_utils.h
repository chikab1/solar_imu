#ifndef __RTC_UTILS_H__
#define __RTC_UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"

/* RTC本地时间相对UTC的分钟偏移（本地 = UTC + 偏移；北京+8区为480）。
 * 由ML307C_Sync_RTC()解析AT+CCLK时区后缀后写入；上电默认0。 */
extern int16_t g_rtc_tz_offset_min;

/* 函数原型 */
uint32_t RTC_Get_Context(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);

#ifdef __cplusplus
}
#endif

#endif /* __RTC_UTILS_H__ */
