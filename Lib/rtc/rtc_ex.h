#ifndef STM32_LIB_RTC_H
#define STM32_LIB_RTC_H

#include "sys/sys.h"
#include "rtc.h"
#include "time.h"

#define RTC_INIT_FLAG 0x1
#define RTC_BACKUP_REGISTER RTC_BKP_DR1

void RTC_Init(void);

HAL_StatusTypeDef rtc_set_datetime(struct tm *tm_info);
void rtc_get_datetime(struct tm *tm_info);

#endif // STM32_LIB_RTC_H
