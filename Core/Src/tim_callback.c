#include <stdio.h>
#include "time.h"
#include "sys/sys.h"
#include "tim.h"
#include "rtc/rtc_ex.h"
#include "lcd/lcd.h"
#include "tim_callback.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM15) {
    tim_show_datetime();
  }
}

void tim_start_1s_timer(void) {
  HAL_TIM_Base_Start_IT(&htim15);
}

void tim_start_1us_timer(void) {
  HAL_TIM_Base_Start_IT(&htim2);
}

uint32_t tim_get_1us_timer_counter(void) {
  return __HAL_TIM_GET_COUNTER(&htim2);
}

void tim_show_datetime(void) {
  struct tm     now;
  unsigned char time_buf[82] = "";

  rtc_get_datetime(&now);
  snprintf((char *)time_buf, sizeof(time_buf),
           "DateTime: %04d-%02d-%02d %02d:%02d:%02d", now.tm_year + 1900,
           now.tm_mon + 1, now.tm_mday, now.tm_hour, now.tm_min, now.tm_sec);
  lcd_show_string(10, 30, 300, 16, 16, time_buf, BRRED);
}
