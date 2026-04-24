
extern "C" {
#include "adc.h"
//#include <stdio.h>
//#include "stm32wlxx_ll_usart.h"
//#include "stm32wlxx_ll_gpio.h"
#include "spi.h"
}

#include <Factory.hpp>
#include <PowerManagement.hpp>
#include "StRadioAdapter.hpp"
#include "RgbLed.hpp"
//#include "UsartWrite.hpp"

constexpr bool test_mode = false;

Factory::Factory(UART_HandleTypeDef& huart1,
		UART_HandleTypeDef& huart2,
		SPI_HandleTypeDef& hspi2,
		ADC_HandleTypeDef& hadc)
			: huart1_(huart1),
			huart2_(huart2),
			hspi2_(hspi2),
			hadc_(hadc),
			comm_(archive_, power_, huart1_, huart2_),
			flash_(&hspi2_, CSB_MEM_GPIO_Port, CSB_MEM_Pin),
			archive_(flash_),
			power_(&hadc) {
}

void Factory::Init(const Radio_s* radio) {
//  usart_write_bind(USART2);
  radio_adapter_ = new StRadioAdapter(radio);
  comm_.Init(*radio_adapter_);
  RgbLed(RgbColor::White, LedState::Off);
  SoftLed(4, LedState::Off);
  SoftLed(5, LedState::Off);
}

void Factory::Service() {
	comm_.UpdateStatusLeds();
}

void Factory::ForwardToBluetooth(const uint8_t* data, std::size_t len) {
	HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 100);
}

void Factory::OnRadioTxDone() {
	comm_.OnRadioTxDone();
}

void Factory::OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo) {
	comm_.OnRadioRxDone(payload, size, rssi, LoraSnr_FskCfo);
}

void Factory::OnUART1Char(uint8_t uart_char)
{
    // Optional: if the phone app sends commands back
    // You can forward them to the locator or parse them here.
//    HAL_UART_Transmit(&huart2_, &uart_char, 1, 100);
	comm_.OnUART1Char(uart_char);
}

void Factory::OnUART2Char(uint8_t uart_char) {
//  config_.ProcessChar(uart_char, device_state_);
    HAL_UART_Transmit(&huart2_, &uart_char, 1, 100);
}
