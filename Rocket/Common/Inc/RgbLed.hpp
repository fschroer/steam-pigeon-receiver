#pragma once

extern "C" {
#include "gpio.h"
}

enum class RgbColor : uint8_t {
	White = 0,
	Red,
	Green,
	Blue,
	Cyan,
	Magenta,
	Yellow
};

enum class LedState : uint8_t {
	Off = 0,
	On,
};

inline void RgbLed(RgbColor color, LedState led_state) {
	switch (color) {
		case RgbColor::White :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			break;
		case RgbColor::Red :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, GPIO_PIN_SET);
			break;
		case RgbColor::Green :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, GPIO_PIN_SET);
			break;
		case RgbColor::Blue :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			break;
		case RgbColor::Cyan :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			break;
		case RgbColor::Magenta :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			break;
		case RgbColor::Yellow :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, GPIO_PIN_SET);
			break;
	}
}

inline void SoftLed(uint8_t led, LedState led_state) {
	switch (led) {
		case 1 :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			break;
		case 2 :
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			break;
		case 3 :
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, led_state == LedState::On ? GPIO_PIN_RESET : GPIO_PIN_SET);
			break;
		case 4 :
			HAL_GPIO_WritePin(SOFT_LED4_GPIO_Port, SOFT_LED4_Pin, led_state == LedState::On ? GPIO_PIN_SET : GPIO_PIN_RESET);
			break;
		case 5 :
			HAL_GPIO_WritePin(SOFT_LED5_GPIO_Port, SOFT_LED5_Pin, led_state == LedState::On ? GPIO_PIN_SET : GPIO_PIN_RESET);
			break;
		default :
			break;
	}
}
