#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>

extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;

//
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);
void delay_us(int x);

// LED PWM defines
#define PWM_FREQ 100
#define PWM_RES 256
#define PWM_SCALE ((12000000 / PWM_RES / PWM_FREQ) - 1)

// GPIO Aliases
#define SIN_0_Pin GPIO_PIN_0
#define SIN_0_GPIO_Port GPIOA
#define SIN_1_Pin GPIO_PIN_1
#define SIN_1_GPIO_Port GPIOA
#define SIN_2_Pin GPIO_PIN_2
#define SIN_2_GPIO_Port GPIOA
#define SIN_3_Pin GPIO_PIN_3
#define SIN_3_GPIO_Port GPIOA
#define SIN_4_Pin GPIO_PIN_4
#define SIN_4_GPIO_Port GPIOA
#define SIN_5_Pin GPIO_PIN_5
#define SIN_5_GPIO_Port GPIOA
#define SIN_6_Pin GPIO_PIN_6
#define SIN_6_GPIO_Port GPIOA
#define SIN_7_Pin GPIO_PIN_7
#define SIN_7_GPIO_Port GPIOA
#define LED_CAPS_Pin GPIO_PIN_0
#define LED_CAPS_GPIO_Port GPIOB
#define PWM_LED_Pin GPIO_PIN_10
#define PWM_LED_GPIO_Port GPIOB
#define PWM_ARR_Pin GPIO_PIN_11
#define PWM_ARR_GPIO_Port GPIOB
#define A0_Pin GPIO_PIN_12
#define A0_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_13
#define A1_GPIO_Port GPIOB
#define A2_Pin GPIO_PIN_14
#define A2_GPIO_Port GPIOB
#define A3_Pin GPIO_PIN_15
#define A3_GPIO_Port GPIOB
#define CHG_Pin GPIO_PIN_8
#define CHG_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_3
#define SW3_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_4
#define SW2_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_5
#define SW1_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
