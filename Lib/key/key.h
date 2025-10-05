#ifndef STM32_LIB_KEY_H
#define STM32_LIB_KEY_H

#include "sys/sys.h"

#define KEY0_GPIO_PORT GPIOH
#define KEY0_GPIO_PIN GPIO_PIN_3
#define KEY0_IRQn EXTI3_IRQn
#define KEY0_IRQHandler EXTI3_IRQHandler

#define KEY1_GPIO_PORT GPIOH
#define KEY1_GPIO_PIN GPIO_PIN_2
#define KEY1_IRQn EXTI2_IRQn
#define KEY1_IRQHandler EXTI2_IRQHandler

#define KEY2_GPIO_PORT GPIOC
#define KEY2_GPIO_PIN GPIO_PIN_13
#define KEY2_IRQn EXTI15_10_IRQn
#define KEY2_IRQHandler EXTI15_10_IRQHandler

#define WKUP_GPIO_PORT GPIOA
#define WKUP_GPIO_PIN GPIO_PIN_0
#define WKUP_IRQn EXTI0_IRQn
#define WKUP_IRQHandler EXTI0_IRQHandler

#define KEY0 HAL_GPIO_ReadPin(KEY0_GPIO_PORT, KEY0_GPIO_PIN)  /* Read KEY0 */
#define KEY1 HAL_GPIO_ReadPin(KEY1_GPIO_PORT, KEY1_GPIO_PIN)  /* Read KEY1 */
#define KEY2 HAL_GPIO_ReadPin(KEY2_GPIO_PORT, KEY2_GPIO_PIN)  /* Read KEY2 */
#define WK_UP HAL_GPIO_ReadPin(WKUP_GPIO_PORT, WKUP_GPIO_PIN) /* Read WKUP */

#define KEY0_PRESS 1 /* Press KEY0 */
#define KEY1_PRESS 2 /* Press KEY1 */
#define KEY2_PRESS 3 /* Press KEY2 */
#define WKUP_PRESS 4 /* Press KEY_UP (WK_UP) */

void KEY_Init(void);

#endif // STM32_LIB_KEY_H