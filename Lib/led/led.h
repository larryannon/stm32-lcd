#ifndef STM32_LIB_LED_H
#define STM32_LIB_LED_H

#include "sys/sys.h"

#define LED_GPIO_PORT GPIOB

/* LED0 = RED */
#define LED1_PIN GPIO_PIN_0

/* LED1 = GREEN */
#define LED0_PIN GPIO_PIN_1

void LED_Init(void);

static inline void led_on(int id) {
  if (id == 0) {
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED0_PIN, GPIO_PIN_RESET);
  } else if (id == 1) {
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
  }
}

static inline void led_off(int id) {
  if (id == 0) {
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED0_PIN, GPIO_PIN_SET);
  } else if (id == 1) {
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
  }
}

static inline void led_all_on() {
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED0_PIN | LED1_PIN, GPIO_PIN_RESET);
}

static inline void led_all_off() {
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED0_PIN | LED1_PIN, GPIO_PIN_SET);
}

static inline void led_toggle(int id) {
  if (id == 0) {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED0_PIN);
  } else if (id == 1) {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED1_PIN);
  }
}

#endif // STM32_LIB_LED_H