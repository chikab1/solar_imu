/**
  * @file    rtc_utils.c
  * @brief   RTC时间/日期原子读取与Unix时间戳换算
  */

#include "rtc_utils.h"
#include "rtc.h"

/* RTC本地时间相对UTC的分钟偏移，由ML307C_Sync_RTC()设置；上电默认0。 */
int16_t g_rtc_tz_offset_min = 0;


/**
 * @brief 原子顺序读取RTC时间/日期并换算为Unix时间戳。
 * @param time 输出HAL RTC时间结构（本地时间，已含时区偏移）。
 * @param date 输出HAL RTC日期结构（本地日期）。
 * @return 从1970-01-01 UTC起的秒数；RTC存本地时间，换算时减回时区偏移。
 * @note HAL要求先读Time再读Date才能解除影子寄存器锁定。
 */
uint32_t RTC_Get_Context(RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
  static const uint8_t month_days[12] =
      {31,28,31,30,31,30,31,31,30,31,30,31};
  uint32_t days = 0U;
  uint16_t full_year;

  HAL_RTC_GetTime(&hrtc, time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, date, RTC_FORMAT_BIN);
  full_year = (uint16_t)(2000U + date->Year);

  for (uint16_t year = 2000U; year < full_year; year++) {
    days += ((year % 4U) == 0U &&
             ((year % 100U) != 0U || (year % 400U) == 0U)) ? 366U : 365U;
  }
  for (uint8_t month = 1U; month < date->Month && month <= 12U; month++) {
    days += month_days[month - 1U];
    if (month == 2U && (full_year % 4U) == 0U &&
        ((full_year % 100U) != 0U || (full_year % 400U) == 0U)) {
      days++;
    }
  }
  if (date->Date > 0U) days += (uint32_t)date->Date - 1U;
  return (uint32_t)((int32_t)(946684800UL + days * 86400UL +
                              (uint32_t)time->Hours * 3600UL +
                              (uint32_t)time->Minutes * 60UL +
                              time->Seconds) - (int32_t)g_rtc_tz_offset_min * 60);
}
