#ifdef __cplusplus
extern "C" {
#endif

//#include "stdint.h"
//
struct Radio_s;   // forward declaration only

// Called once from main.c after HAL init
void ReceiverFactory_Init(const struct Radio_s* radio);
// Periodic service routine to update status, LEDs, etc.
void ReceiverFactory_Service();
// Called from main.c when the radio has transmitted a packet
void ReceiverFactory_OnRadioTxDone();
// Called from main.c when the radio driver indicates a packet is ready
void ReceiverFactory_OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);
void ReceiverFactory_OnRadioRxError();
void ReceiverFactory_ProcessRadioRx();
// Process UART input from the phone app
void ReceiverFactory_OnUART1Char(uint8_t uart_char);
// Process UART input from user
void ReceiverFactory_OnUART2Char(uint8_t uart_char);

#ifdef __cplusplus
}
#endif
