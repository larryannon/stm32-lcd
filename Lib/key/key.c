#include "key.h"
#include "sys/sys.h"

void KEY0_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(KEY0_GPIO_PIN);
  // __HAL_GPIO_EXTI_CLEAR_IT(KEY0_GPIO_PIN);
}

void KEY1_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(KEY1_GPIO_PIN);
  // __HAL_GPIO_EXTI_CLEAR_IT(KEY1_GPIO_PIN);
}

void KEY2_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(KEY2_GPIO_PIN);
  // __HAL_GPIO_EXTI_CLEAR_IT(KEY2_GPIO_PIN);
}

void WKUP_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(WKUP_GPIO_PIN);
  // __HAL_GPIO_EXTI_CLEAR_IT(WKUP_GPIO_PIN);
}

void KEY_Init(void) {
  GPIO_InitTypeDef gpio_init_struct; // = {0};
  // KEY0/KEY1
  __HAL_RCC_GPIOH_CLK_ENABLE();
  // KEY2
  __HAL_RCC_GPIOC_CLK_ENABLE();
  // WKUP
  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio_init_struct.Pin   = KEY0_GPIO_PIN;
  gpio_init_struct.Mode  = GPIO_MODE_INPUT;
  gpio_init_struct.Pull  = GPIO_PULLUP;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.Pin   = KEY1_GPIO_PIN;
  gpio_init_struct.Mode  = GPIO_MODE_INPUT;
  gpio_init_struct.Pull  = GPIO_PULLUP;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.Pin   = KEY2_GPIO_PIN;
  gpio_init_struct.Mode  = GPIO_MODE_INPUT;
  gpio_init_struct.Pull  = GPIO_PULLUP;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(KEY2_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.Pin   = WKUP_GPIO_PIN;
  gpio_init_struct.Mode  = GPIO_MODE_INPUT;
  gpio_init_struct.Pull  = GPIO_PULLDOWN;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct);
}
