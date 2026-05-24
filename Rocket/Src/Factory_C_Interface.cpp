#include <Factory.hpp>
#include <Factory_C_Interface.h>

// HAL handles provided by CubeMX (C symbols)
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc;

// Single Factory instance
static Factory g_factory(huart1, huart2, hspi2, hadc);

extern "C" void ReceiverFactory_Init(const struct Radio_s* radio) {
    g_factory.Init(radio);
}

extern "C" void ReceiverFactory_Service() {
    g_factory.Service();
}

extern "C" void ReceiverFactory_OnRadioTxDone() {
	g_factory.OnRadioTxDone();
}

extern "C" void ReceiverFactory_OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo) {
	g_factory.OnRadioRxDone(payload, size, rssi, LoraSnr_FskCfo);
}

extern "C" void ReceiverFactory_ProcessRadioRx() {
	g_factory.ProcessRadioRx();
}

extern "C" void ReceiverFactory_OnUART1Char(uint8_t uart_char) {
	g_factory.OnUART1Char(uart_char);
}

extern "C" void ReceiverFactory_OnUART2Char(uint8_t uart_char) {
	g_factory.OnUART2Char(uart_char);
}
