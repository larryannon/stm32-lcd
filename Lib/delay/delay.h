#ifndef STM32_LIB_DELAY_H
#define STM32_LIB_DELAY_H

#include "sys/sys.h"

typedef uint32_t (*get_tick_func_t)(void);

typedef struct {
  get_tick_func_t get_tick_func;
  uint32_t start_tick;
  uint32_t duration_us;
  uint8_t  done;
} delay_t;

void DELAY_Init(get_tick_func_t func);

void delay_us_start_async(delay_t *d, uint32_t us);
void delay_ms_start_async(delay_t *d, uint32_t ms);
void delay_update_async(delay_t *d);
void delay_us(uint32_t Delay);
void delay_ms(uint32_t Delay);

#endif // STM32_LIB_DELAY_H