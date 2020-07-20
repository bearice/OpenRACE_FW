#include "main.h"
#include "led.h"

void led_set_brightness(uint16_t value) {
  value = value % (PWM_RES * 2);
  if (value > PWM_RES)
    value = (PWM_RES * 2) - value;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, value);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_RES - value);
}

void led_set_caps_lock(uint16_t v) {
  HAL_GPIO_WritePin(LED_CAPS_GPIO_Port, LED_CAPS_Pin, !!v);
}

void led_set_num_lock(uint16_t v) {
  UNUSED(v);
  //  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, v);
}

void led_set_scroll_lock(uint16_t v) {
  UNUSED(v);
  //  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, v);
}
