#include "sys/sys.h"
#include "delay.h"

get_tick_func_t g_get_tick_func;

void DELAY_Init(get_tick_func_t func) {
  g_get_tick_func = func;
}

void delay_us_start_async(delay_t *d, uint32_t us) {
  d->get_tick_func = g_get_tick_func;
  d->start_tick  = d->get_tick_func();
  d->duration_us = us;
  d->done        = 0;
}

void delay_ms_start_async(delay_t *d, uint32_t ms) {
  delay_us_start_async(d, ms * 1000);
}

void delay_update_async(delay_t *d) {
  if (!d->done) {
    if (d->get_tick_func() - d->start_tick >= d->duration_us) {
      d->done = 1;
    }
  }
}

void delay_us(uint32_t Delay) {
  volatile uint32_t start = g_get_tick_func();
  while (g_get_tick_func() - start < Delay);
}

void delay_ms(uint32_t Delay) {
  delay_us(Delay * 1000);
}
