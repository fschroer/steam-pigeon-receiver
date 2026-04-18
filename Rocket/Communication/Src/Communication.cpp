extern void *__libc_init_array;

extern "C" {
#include <stdio.h>
}

#include <cstring>
#include "Communication.hpp"
#include "StRadioAdapter.hpp"
#include "Format.hpp"
#include "Units.hpp"
#include "RgbLed.hpp"

namespace Communication {

using Header  = PacketHeader;

Communication::Communication(Archive& archive, PowerManagement& power, UART_HandleTypeDef& huart1)
    : archive_(archive),
	  power_(power),
	  huart1_(huart1){
}

void Communication::Init(IRadio& radio) { // To do: archive channel
	radio_ = &radio;
	radio_->SetChannel(902300000);
//	radio_->SetChannel(902300000 + archive_.GetLocatorSettings().lora_channel * 200000);
	radio_->Send((uint8_t*)lora_startup_message_, strlen(lora_startup_message_));
}

void Communication::SetChannel(uint8_t channel) {
	radio_->SetChannel(902300000 + channel * 200000);
}

void Communication::OnRadioTxDone()
{
    radio_busy_ = false;
    SoftLed(4, LedState::On);
    last_tx_end_ms_ = HAL_GetTick();
	tx_led_status_serviced_ = false;
}

void Communication::OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo) {
	ParsedMessage parsed{};
	if (ParseLoraFrame(payload, size, system_id, parsed) == ParseResult::Ok) {
		switch (parsed.type) {
			case MsgType::PreLaunchData: {
				PreLaunchMessageExtended ext{};
				// Copy the original message
				std::memcpy(&ext.base, &parsed.prelaunch, sizeof(PreLaunchData));
				// Append receiver metadata
				ext.receiver_lora_channel = archive_.GetLocatorSettings().lora_channel;;
				power_.enableDivider();
				HAL_Delay(50);
				ext.receiver_battery_level = power_.readBatteryMillivolts();
				// Recompute CRC over the extended struct
				ext.base.packet_header.crc = ComputeSendMessageCrc(ext);
				ForwardToBluetooth(reinterpret_cast<const uint8_t*>(&ext), sizeof(ext));
				break;
			}
			default: {
				ForwardToBluetooth(payload, size);
				break;
			}
		}
	    SoftLed(5, LedState::On);
	    last_rx_end_ms_ = HAL_GetTick();
		rx_led_status_serviced_ = false;
	}
	else {
		RgbLed(RgbColor::Red, LedState::On);
		return;
    }

//	SoftLed(4, LedState::On);
//	last_rx_end_ms_ = HAL_GetTick();
//	rx_led_status_serviced_ = false;
}

void Communication::ForwardToBluetooth(const uint8_t* buf, std::size_t len) {
    // Blocking UART transmit; you can switch to DMA if you want.
    HAL_UART_Transmit(&huart1_, const_cast<uint8_t*>(buf), static_cast<uint16_t>(len), 100);
}

void Communication::UpdateStatusLeds() {
	uint32_t current_tick = HAL_GetTick();
	if (!tx_led_status_serviced_ && current_tick - last_tx_end_ms_ > 100) {
		SoftLed(4, LedState::Off);
		tx_led_status_serviced_ = true;
	}
	if (!rx_led_status_serviced_ && current_tick - last_rx_end_ms_ > 100) {
        RgbLed(RgbColor::Red, LedState::Off);
		SoftLed(5, LedState::Off);
		rx_led_status_serviced_ = true;
	}
}

ParseResult Communication::ParseLoraFrame(const uint8_t* data,
                           std::size_t   len,
                           uint8_t       expected_system_id,
                           ParsedMessage& out)
{
  if (len < sizeof(PacketHeader)) {
    return ParseResult::TooShort;
  }

  // Extract header
  PacketHeader hdr{};
  std::memcpy(&hdr, data, sizeof(PacketHeader));
  // System ID check
  if (hdr.system_id != expected_system_id) {
    return ParseResult::SystemIdMismatch;
  }

  // CRC check
  if (!ValidateCRC(data, len)) {
    return ParseResult::CrcMismatch;
  }

  // Dispatch by message type
  switch (hdr.msg_type) {

  case MsgType::PreLaunchData:
    if (len != sizeof(PreLaunchData)) {
      return ParseResult::LengthMismatch;
    }
    std::memcpy(&out.prelaunch, data, sizeof(PreLaunchData));
    out.type = MsgType::PreLaunchData;
    return ParseResult::Ok;

  case MsgType::TelemetryData:
    if (len != sizeof(TelemetryData)) {
      return ParseResult::LengthMismatch;
    }
    std::memcpy(&out.telemetry, data, sizeof(TelemetryData));
    out.type = MsgType::TelemetryData;
    return ParseResult::Ok;

  case MsgType::DeploymentTest:
    if (len != sizeof(DeploymentTestCountdownMessage)) {
      return ParseResult::LengthMismatch;
    }
    std::memcpy(&out.deployment_test, data, sizeof(DeploymentTestCountdownMessage));
    out.type = MsgType::DeploymentTest;
    return ParseResult::Ok;

  default:
    return ParseResult::UnknownType;
  }
}

} // namespace Communication
