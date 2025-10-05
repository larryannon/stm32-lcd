#include "led.h"
#include "sys/sys.h"

void LED_Init(void) {
  GPIO_InitTypeDef gpio_init_struct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  led_all_off();

  gpio_init_struct.Pin   = LED0_PIN | LED1_PIN;
  gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull  = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_PORT, &gpio_init_struct);
}
