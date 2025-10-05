#include "sys/sys.h"
#include "rtc_ex.h"

void RTC_Init(void) {
  uint16_t initFlag = HAL_RTCEx_BKUPRead(&hrtc, RTC_BACKUP_REGISTER);
  if (initFlag == RTC_INIT_FLAG) {
    return;
  }

  struct tm time = {
      .tm_year = 2025 - 1900,
      .tm_mon  = 10 - 1,
      .tm_mday = 5,
      .tm_hour = 15,
      .tm_min  = 37,
      .tm_sec  = 55,
      .tm_wday = 0,
  };
  rtc_set_datetime(&time);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BACKUP_REGISTER, RTC_INIT_FLAG);
}

void rtc_get_datetime(struct tm *tm_info) {
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  tm_info->tm_year = sDate.Year + 100; // 1900 + 100 + Year
  tm_info->tm_mon  = sDate.Month - 1;
  tm_info->tm_mday = sDate.Date;
  tm_info->tm_hour = sTime.Hours;
  tm_info->tm_min  = sTime.Minutes;
  tm_info->tm_sec  = sTime.Seconds;
}

HAL_StatusTypeDef rtc_set_datetime(struct tm *tm_info) {
  RTC_DateTypeDef rtc_date_handle;
  RTC_TimeTypeDef rtc_time_handle;

  rtc_date_handle.Year    = tm_info->tm_year - 100;
  rtc_date_handle.Month   = tm_info->tm_mon + 1;
  rtc_date_handle.Date    = tm_info->tm_mday;
  rtc_date_handle.WeekDay = tm_info->tm_wday;

  rtc_time_handle.Hours          = tm_info->tm_hour;
  rtc_time_handle.Minutes        = tm_info->tm_min;
  rtc_time_handle.Seconds        = tm_info->tm_sec;
  rtc_time_handle.TimeFormat     = RTC_HOURFORMAT_24;
  rtc_time_handle.StoreOperation = RTC_STOREOPERATION_RESET;
  rtc_time_handle.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;

  HAL_StatusTypeDef ret =
      HAL_RTC_SetTime(&hrtc, &rtc_time_handle, RTC_FORMAT_BIN);
  if (ret != HAL_OK) {
    return ret;
  }
  ret = HAL_RTC_SetDate(&hrtc, &rtc_date_handle, RTC_FORMAT_BIN);

  return ret;
}
