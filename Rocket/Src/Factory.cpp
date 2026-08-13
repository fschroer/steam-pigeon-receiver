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
#include "StaticStringWriter.hpp"
#include "RgbLed.hpp"
//#include "UsartWrite.hpp"

constexpr bool test_mode = false;

Factory::Factory(UART_HandleTypeDef &huart1, UART_HandleTypeDef &huart2, SPI_HandleTypeDef &hspi2,
		ADC_HandleTypeDef &hadc) :
		huart1_(huart1), huart2_(huart2), hspi2_(hspi2), hadc_(hadc), comm_(deviceUID_, archive_, power_, huart1_,
				huart2_), flash_(&hspi2_, CSB_MEM_GPIO_Port, CSB_MEM_Pin), archive_(deviceUID_, flash_),
				console_baud_(huart2_), config_(comm_, archive_, huart2_, console_baud_), power_(&hadc) {
}

void Factory::Init(const Radio_s *radio) {
//  usart_write_bind(USART2);
	// Before the flash and radio come up, so an operator whose console is
	// unreadable does not have to race the rest of init to get a sync run in.
	console_baud_.Begin();
	radio_adapter_ = new StRadioAdapter(radio);
	archive_.Init(); // Initialize before comm to pull name and channel
	// First point at which the stored rate is readable — it lives on the external
	// SPI flash.  A detection that already fired during boot outranks it.
	console_baud_.ApplyStoredRate(archive_.GetConsoleBaud());
	comm_.Init(*radio_adapter_);
	RgbLed(RgbColor::White, LedState::Off);
	SoftLed(4, LedState::Off);
	SoftLed(5, LedState::Off);
}

void Factory::Service() {
	ServiceConsole();
	comm_.UpdateStatusLeds();
	comm_.ServicePendingTx();
	comm_.ServiceNoiseFloor();
	comm_.ServiceChannelSurvey();
	comm_.ServiceBadFrameTrace();
	comm_.ServiceBleNameUpdate();
	comm_.ServiceReceiverInfoResponse();
}

void Factory::OnRadioTxDone() {
	comm_.OnRadioTxDone();
}

void Factory::OnRadioRxError() {
	comm_.OnRadioRxError();
}

void Factory::ProcessRadioRx() {
	comm_.ProcessRadioRx();
}

void Factory::OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo) {
	comm_.OnRadioRxDone(payload, size, rssi, LoraSnr_FskCfo);
}

void Factory::OnUART1Char(uint8_t uart_char) {
//    HAL_UART_Transmit(&huart1_, &uart_char, 1, 100);
	comm_.OnUART1Char(uart_char);
}

void Factory::OnUART2Char(uint8_t uart_char) {
	// ISR context — queue only.  Handling runs in Service(), because adopting a
	// detected baud rate re-inits USART2 and that cannot be done from inside
	// USART2's own interrupt.
	const uint16_t next = (uart2_rx_head_ + 1u) & (kUart2RxBufSize - 1u);
	if (next != uart2_rx_tail_) {        // drop on overflow rather than block in the ISR
		uart2_rx_buf_[uart2_rx_head_] = uart_char;
		uart2_rx_head_ = next;
	}
}

void Factory::ServiceConsoleBaud() {
	console_baud_.Poll(HAL_GetTick());

	uint32_t detected_rate = 0;
	if (!console_baud_.TakeCommittedRate(detected_rate))
		return;
	const bool saved = archive_.SetConsoleBaud(detected_rate);
	// Emitted at the rate just adopted, so the line is its own proof the link
	// works — if the operator can read this, the recovery succeeded.
	// Charset reset first: this line follows a stretch of garbage by definition,
	// and that garbage may have left the terminal in DEC Special Graphics.
	StaticStringWriter<96> line(&huart2_);
	line.WriteMany(ConsoleBaud::kAsciiCharsetReset, "\r\nDIAG|BAUD: detected ", detected_rate,
			saved ? " - saved\r\n" : " - SAVE FAILED\r\n");
}

void Factory::ServiceConsole() {
	ServiceConsoleBaud();
	while (uart2_rx_tail_ != uart2_rx_head_) {
		const uint8_t uart_char = uart2_rx_buf_[uart2_rx_tail_];
		uart2_rx_tail_ = (uart2_rx_tail_ + 1u) & (kUart2RxBufSize - 1u);
		// The sync machinery gets first look.  It claims a byte only while a
		// verified sync run is actually in progress; at a matched rate it never
		// arms and every byte passes straight through.
		if (console_baud_.OnByte(uart_char))
			continue;
		config_.ProcessChar(uart_char, device_state_);
	}
	// The live rate changed somewhere in the pass above.  Everything still queued
	// was sampled at the OLD rate and is noise now, so it is dropped rather than
	// handed to the console — otherwise a rate change that worked perfectly still
	// paints a screen of garbage and reads as a failure.
	if (console_baud_.TakeRateChanged())
		uart2_rx_tail_ = uart2_rx_head_;
}
