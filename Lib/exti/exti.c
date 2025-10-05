#include "exti.h"
#include "key/key.h"
#include "sys/sys.h"

void EXTI_Init(void) {
  GPIO_InitTypeDef gpio_init_struct = {0};

  KEY_Init();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

  gpio_init_struct.Pin  = KEY0_GPIO_PIN;
  gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;
  gpio_init_struct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.Pin  = KEY1_GPIO_PIN;
  gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;
  gpio_init_struct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.Pin  = KEY2_GPIO_PIN;
  gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;
  gpio_init_struct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY2_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.Pin  = WKUP_GPIO_PIN;
  gpio_init_struct.Mode = GPIO_MODE_IT_RISING;
  gpio_init_struct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct);

  HAL_NVIC_SetPriority(KEY0_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(KEY0_IRQn);

  HAL_NVIC_SetPriority(KEY1_IRQn, 1, 2);
  HAL_NVIC_EnableIRQ(KEY1_IRQn);

  HAL_NVIC_SetPriority(KEY2_IRQn, 2, 2);
  HAL_NVIC_EnableIRQ(KEY2_IRQn);

  HAL_NVIC_SetPriority(WKUP_IRQn, 3, 2);
  HAL_NVIC_EnableIRQ(WKUP_IRQn);
}
