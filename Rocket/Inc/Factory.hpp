#pragma once
extern "C" {
#include "radio.h"
//#include "usart.h"
//#include "subghz_phy_app.h"
//#include "stm32wlxx_hal_rtc.h"
#include "tim.h"
}

#include "Communication.hpp"
#include "Archive.hpp"
#include "UserInteraction.hpp"
#include "FlashDriver.hpp"
#include "MX25L4006E.hpp"
#include "PowerManagement.hpp"
#include "StRadioAdapter.hpp"
#include "ConsoleBaud.hpp"

enum FlightProfileState {
	kIdle = 0, kMetadataRequested = 1
};

struct Radio_s;
// forward declaration from C

class Factory {
public:
	Factory(UART_HandleTypeDef &huart1, UART_HandleTypeDef &huart2, SPI_HandleTypeDef &hspi2, ADC_HandleTypeDef &hadc);
	void Init(const Radio_s *radio);
	void Service();
	void ForwardToBluetooth(const uint8_t *data, std::size_t len);
	void OnRadioTxDone();
	void OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);
	void OnRadioRxError();
	void ProcessRadioRx();
	void OnUART1Char(uint8_t uart_char);
	void OnUART2Char(uint8_t uart_char);   // ISR context: enqueue only
private:
	// Console (UART2) input is queued by the RX callback and handled from
	// Service(), not in ISR context.  Adopting a detected baud rate re-inits the
	// USART, which cannot be done from inside that peripheral's own interrupt
	// without dropping the byte being serviced.
	void ServiceConsole();
	// Runs the sync-byte verify timeout and persists a detection once it commits.
	void ServiceConsoleBaud();

	static constexpr uint16_t kUart2RxBufSize = 128;  // power of two
	volatile uint8_t uart2_rx_buf_[kUart2RxBufSize] = { };
	volatile uint16_t uart2_rx_head_ = 0;  // producer: UART2 RX callback
	volatile uint16_t uart2_rx_tail_ = 0;  // consumer: Service()

	UART_HandleTypeDef &huart1_;
	UART_HandleTypeDef &huart2_;
	SPI_HandleTypeDef &hspi2_;
	ADC_HandleTypeDef &hadc_;
	const Radio_s *radio_ = nullptr;

	DeviceUID deviceUID_;
	Communication::Communication comm_;
	MX25L4006E flash_;
	Archive archive_;
	// Declared ahead of config_ so it is constructed first: UserInteraction takes
	// a reference to it.
	ConsoleBaud console_baud_;
	UserInteraction config_;
	PowerManagement power_;
	StRadioAdapter *radio_adapter_ = nullptr;

	const char *lora_startup_message_ = "Rocket Receiver v1.0.1\r\n\0";
	const char *usb_connected_ = "Disconnect USB cable before arming locator\r\n\0";
	const char *bad_gps_data_ = "Bad GPS Data\r\n\0";

	DeviceState device_state_ = DeviceState::Receive;
};
