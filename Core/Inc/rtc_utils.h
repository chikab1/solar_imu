#ifndef __RTC_UTILS_H__
#define __RTC_UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"

/* 函数原型 */
uint32_t RTC_Get_Context(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);

#ifdef __cplusplus
}
#endif

#endif /* __RTC_UTILS_H__ */
