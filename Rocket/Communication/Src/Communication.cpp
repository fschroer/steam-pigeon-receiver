extern void *__libc_init_array;

extern "C" {
#include <stdio.h>
}

#include <cstring>
#include <cstdint>
#include "Communication.hpp"
#include "StRadioAdapter.hpp"
#include "Format.hpp"
#include "Units.hpp"
#include "RgbLed.hpp"

namespace Communication {

using Header = PacketHeader;

Communication::Communication(DeviceUID &deviceUID, Archive &archive, PowerManagement &power, UART_HandleTypeDef &huart1,
		UART_HandleTypeDef &huart2) :
		deviceUID_(deviceUID), archive_(archive), power_(power), huart1_(huart1), huart2_(huart2) {
}

void Communication::Init(IRadio &radio) {
	RocketPersistentSettings settings = archive_.GetReceiverSettings();
	radio_ = &radio;
	SetChannel(settings.lora_channel);
	radio_->Send((uint8_t*) lora_startup_message_, strlen(lora_startup_message_));

	// Generate BLE name
	char change_spp_name[sizeof(change_ble_name_) - 1 + device_name_length + 2] = { 0 };
	std::memcpy(change_spp_name, change_ble_name_, sizeof(change_ble_name_) - 1);
	uint8_t i = 0;
	for (; i < device_name_length; i++) {
		char device_name_char = settings.device_name[i];
		if (device_name_char == 0)
			break;
		change_spp_name[i + sizeof(change_ble_name_) - 1] = device_name_char;
	}
	change_spp_name[sizeof(change_ble_name_) - 1 + i++] = '\r';
	change_spp_name[sizeof(change_ble_name_) - 1 + i++] = '\n';

	// Generate BLE address
	uint32_t ble_address_lsb32 = deviceUID_.getUID();
	char ble_address_lsb32_text[] = "00000000";
	Uint32ToHex(ble_address_lsb32_text, ble_address_lsb32);
	char set_ble_address[sizeof(set_ble_address_) - 1 + sizeof(ble_msb16_) - 1 + sizeof(ble_address_lsb32_text) - 1 + 2] = { 0 };
	std::memcpy(set_ble_address, set_ble_address_, sizeof(set_ble_address_) - 1);
	std::memcpy(set_ble_address + sizeof(set_ble_address_) - 1, ble_msb16_, sizeof(ble_msb16_) - 1);
	std::memcpy(set_ble_address + sizeof(set_ble_address_) - 1 + sizeof(ble_msb16_) - 1, ble_address_lsb32_text,
			sizeof(ble_address_lsb32_text) - 1);
	set_ble_address[sizeof(set_ble_address_) - 1 + sizeof(ble_msb16_) - 1 + sizeof(ble_address_lsb32_text) - 1] = '\r';
	set_ble_address[sizeof(set_ble_address_) - 1 + sizeof(ble_msb16_) - 1 + sizeof(ble_address_lsb32_text)] = '\n';

	HAL_UART_Transmit(&huart1_, (uint8_t*) command_mode_, (sizeof(command_mode_) - 1), 100);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1_, (uint8_t*) enable_ble_broadcast_, sizeof(enable_ble_broadcast_) - 1, 100);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1_, (uint8_t*) disable_spp_broadcast_, sizeof(disable_spp_broadcast_) - 1, 100);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1_, (uint8_t*) change_spp_name, sizeof(change_spp_name_) - 1 + i, 100);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1_, (uint8_t*) set_ble_address, (sizeof(set_ble_address)), 100);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1_, (uint8_t*) reset_, (sizeof(reset_) - 1), 100);
	HAL_Delay(100);
}

void Communication::SetChannel(uint8_t channel) {
	radio_->SetChannel(902300000 + channel * 200000);
}

void Communication::OnRadioTxDone() {
	radio_busy_ = false;
	RgbLed(RgbColor::Blue, LedState::On);
	last_radio_tx_end_ms_ = HAL_GetTick();
	radio_tx_led_status_serviced_ = false;
}

void Communication::OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo) {
	RgbLed(RgbColor::Green, LedState::On);
	last_radio_rx_end_ms_ = HAL_GetTick();
	radio_rx_led_status_serviced_ = false;
	std::memcpy(rx_payload_, payload, size);
	rx_message_size_ = size;
	rssi_ = rssi;
	LoraSnr_FskCfo_ = LoraSnr_FskCfo;
}

void Communication::ProcessRadioRx() {
	ParsedMessage parsed { };
	if (ParseLoraFrame(rx_payload_, rx_message_size_, system_id, parsed) == ParseResult::Ok) {
		switch (parsed.type) {
		case MsgType::PreLaunchData: {
			PreLaunchMessageExtended ext { };
			// Copy the original message
			std::memcpy(&ext.base, &parsed.prelaunch, sizeof(PreLaunchData));
			// Append receiver metadata
			ext.receiver_lora_channel = archive_.GetReceiverSettings().lora_channel;
			;
			power_.enableDivider();
			HAL_Delay(50);
			ext.receiver_battery_level = power_.readBatteryMillivolts();
			// Recompute CRC over the extended struct
			ext.base.packet_header.crc = ComputeSendMessageCrc(ext);
			ForwardToBluetooth(reinterpret_cast<const uint8_t*>(&ext), sizeof(ext));
			break;
		}
		default: {
			ForwardToBluetooth(rx_payload_, rx_message_size_);
			break;
		}
		}
		RgbLed(RgbColor::Green, LedState::On);
		last_radio_rx_end_ms_ = HAL_GetTick();
		radio_rx_led_status_serviced_ = false;
	} else {
		RgbLed(RgbColor::Red, LedState::On);
		return;
	}
}

void Communication::ForwardToBluetooth(const uint8_t *buf, std::size_t len) {
	// Blocking UART transmit; you can switch to DMA if you want.
	last_bt_tx_end_ms_ = HAL_GetTick();
	bt_tx_led_status_serviced_ = false;
	SoftLed(4, LedState::On);
	HAL_UART_Transmit(&huart1_, const_cast<uint8_t*>(buf), static_cast<uint16_t>(len), 100);
}

void Communication::UpdateStatusLeds() {
	current_tick_ = HAL_GetTick();
	if (!radio_tx_led_status_serviced_ && current_tick_ - last_radio_tx_end_ms_ > 100) {
		RgbLed(RgbColor::Blue, LedState::Off);
		radio_tx_led_status_serviced_ = true;
	}
	if (!radio_rx_led_status_serviced_ && current_tick_ - last_radio_rx_end_ms_ > 100) {
		RgbLed(RgbColor::Green, LedState::Off);
		radio_rx_led_status_serviced_ = true;
	}
	if (!bt_tx_led_status_serviced_ && current_tick_ - last_bt_tx_end_ms_ > 100) {
		SoftLed(4, LedState::Off);
		bt_tx_led_status_serviced_ = true;
	}
	if (!bt_rx_led_status_serviced_ && current_tick_ - last_bt_rx_end_ms_ > 100) {
		SoftLed(5, LedState::Off);
		bt_rx_led_status_serviced_ = true;
	}
}

ParseResult Communication::ParseLoraFrame(const uint8_t *data, std::size_t len, uint8_t expected_system_id,
		ParsedMessage &out) {
	using namespace Communication;

	if (len < sizeof(PacketHeader))
		return ParseResult::TooShort;

	// Extract header
	PacketHeader hdr { };
	std::memcpy(&hdr, data, sizeof(PacketHeader));

	// System ID check
	if (hdr.system_id != expected_system_id)
		return ParseResult::SystemIdMismatch;

	// CRC check
	if (!ValidateCRC(data, len))
		return ParseResult::CrcMismatch;

	// Dispatch by message type
	switch (hdr.msg_type) {
	case MsgType::Startup:
		return decode_message<MsgType::Startup>(data, len, out);

	case MsgType::PreLaunchData:
		return decode_message<MsgType::PreLaunchData>(data, len, out);

	case MsgType::TelemetryData:
		return decode_message<MsgType::TelemetryData>(data, len, out);

	case MsgType::DeploymentTest:
		return decode_message<MsgType::DeploymentTest>(data, len, out);

	case MsgType::FlightMetadata:
		return decode_message<MsgType::FlightMetadata>(data, len, out);

	case MsgType::FlightData:
		return decode_message<MsgType::FlightData>(data, len, out);

	default:
		return ParseResult::UnknownType;
	}
}

void Communication::OnUART1Char(uint8_t uart_char) {
	last_bt_rx_end_ms_ = HAL_GetTick();
	bt_rx_led_status_serviced_ = false;
	SoftLed(5, LedState::On);
	uint32_t current_byte_time = HAL_GetTick();
	if (parse_state_ != ParseState::IDLE && (current_byte_time - last_byte_time_ > MESSAGE_TIMEOUT_MS)) {
		Reset(); // Dump the buffer if the app stops mid-stream
		RgbLed(RgbColor::Red, LedState::On);
	}
	last_byte_time_ = current_byte_time;

	switch (parse_state_) {
	case ParseState::IDLE:
		if (uart_char == system_id) {
			current_msg_.header.system_id = uart_char;
			parse_state_ = ParseState::TYPE;
			cursor_ = 0;
		}
		break;

	case ParseState::TYPE:
		current_msg_.header.msg_type = static_cast<MsgType>(uart_char);
		switch (current_msg_.header.msg_type) {
		case MsgType::LocatorCfgChgRequest:
			message_length_ = sizeof(LocatorRocketSettings) - sizeof(PacketHeader);
			break;
		case MsgType::ReceiverCfgChgRequest:
			message_length_ = sizeof(ReceiverSettings) - sizeof(PacketHeader);
			break;
		case MsgType::ArmRequest:
			message_length_ = 0;
			break;
		case MsgType::DisarmRequest:
			message_length_ = 0;
			break;
		case MsgType::FlightMetadataRequest:
			message_length_ = 0;
			break;
		case MsgType::FlightDataRequest:
			message_length_ = 1;
			break;
		case MsgType::FlightDataAck:
			message_length_ = 0;
			break;
		case MsgType::DeploymentTestRequest:
			message_length_ = 1;
			break;
		}
		if (message_length_ > 64) { // Safety check
			Reset();
		} else {
			parse_state_ = ParseState::COUNT1;
		}
		break;

	case ParseState::COUNT1:
		current_msg_.header.msg_count = uart_char;
		parse_state_ = ParseState::COUNT2;
		break;

	case ParseState::COUNT2:
		current_msg_.header.msg_count = (current_msg_.header.msg_count & 0xff) | ((uint16_t) uart_char << 8);
		parse_state_ = ParseState::CRC1;
		break;

	case ParseState::CRC1:
		current_msg_.header.crc = uart_char;
		parse_state_ = ParseState::CRC2;
		break;

	case ParseState::CRC2:
		current_msg_.header.crc = (current_msg_.header.crc & 0xff) | ((uint16_t) uart_char << 8);
		if (cursor_ >= message_length_) {
			if (ValidateCRC(reinterpret_cast<const uint8_t*>(&current_msg_), sizeof(PacketHeader) + message_length_)) {
				radio_->Send(reinterpret_cast<const uint8_t*>(&current_msg_), sizeof(PacketHeader) + message_length_);
			}
			Reset();
		} else {
			parse_state_ = ParseState::DATA;
		}
		break;

	case ParseState::DATA:
		current_msg_.payload[cursor_++] = uart_char;
		if (cursor_ >= message_length_) {
			if (current_msg_.header.msg_type == MsgType::ReceiverCfgChgRequest) {
				RocketPersistentSettings &receiver_settings = archive_.GetReceiverSettings();
				receiver_settings.lora_channel = current_msg_.payload[0];
//				for (uint8_t i = 0; i < device_name_length; i++)
//					receiver_settings.device_name[i] = device_name_[i];
				archive_.SaveReceiverSettings(receiver_settings);
				SetChannel(receiver_settings.lora_channel);
			} else {
				if (ValidateCRC(reinterpret_cast<const uint8_t*>(&current_msg_),
						sizeof(PacketHeader) + message_length_)) {
					radio_->Send(reinterpret_cast<const uint8_t*>(&current_msg_),
							sizeof(PacketHeader) + message_length_);
				}
			}
			Reset();
		}
		break;
	}
}

} // namespace Communication
