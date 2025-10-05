#ifndef STM32_LCD_TIM_CALLBACK_H
#define STM32_LCD_TIM_CALLBACK_H

void tim_start_1s_timer(void);
void tim_start_1us_timer(void);
uint32_t tim_get_1us_timer_counter(void);
void tim_show_datetime(void);

#endif // STM32_LCD_TIM_CALLBACK_H
