extern void *__libc_init_array;

extern "C" {
#include <stdio.h>
}

#include <cstring>
#include <cstdint>
#include "Communication.hpp"
#include "version.h"
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
//	RgbLed(RgbColor::Green, LedState::On);
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
			last_locator_periodic_rx_ms_ = HAL_GetTick();
			locator_periodic_ever_rx_    = true;
			locator_in_profile_mode_         = false;  // locator returned to Disarmed
			PreLaunchMessageExtended ext { };
			// Copy the original message
			std::memcpy(&ext.base, &parsed.prelaunch, sizeof(PreLaunchData));
			// Append receiver metadata
			const RocketPersistentSettings& recv_settings = archive_.GetReceiverSettings();
			ext.receiver_lora_channel = recv_settings.lora_channel;
			std::memcpy(ext.receiver_name, recv_settings.device_name, device_name_length);
			power_.enableDivider();
			HAL_Delay(50);
			ext.receiver_battery_level = power_.readBatteryMillivolts();
			ext.rssi = rssi_;
			// Recompute CRC over the extended struct
			ext.base.packet_header.crc = ComputeSendMessageCrc(ext);
			ForwardToBluetooth(reinterpret_cast<const uint8_t*>(&ext), sizeof(ext));
			break;
		}
		case MsgType::TelemetryData: {
			last_locator_periodic_rx_ms_ = HAL_GetTick();
			locator_periodic_ever_rx_    = true;
			TelemetryMessageExtended ext { };
			std::memcpy(&ext.base, &parsed.telemetry, sizeof(TelemetryData));
			ext.rssi = rssi_;
			ext.base.packet_header.crc = ComputeSendMessageCrc(ext);
			ForwardToBluetooth(reinterpret_cast<const uint8_t*>(&ext), sizeof(ext));
			break;
		}
		case MsgType::VersionInfo: {
			// Locator responded with its firmware version.  Append the receiver's
			// version and forward the extended message to the app via Bluetooth.
			VersionInfoExtended ext { };
			std::memcpy(&ext.base, &parsed.version_info, sizeof(VersionInfoMessage));
			std::memcpy(ext.receiver_version, GIT_VERSION,
					std::min(sizeof(ext.receiver_version), sizeof(GIT_VERSION)));
			ext.base.packet_header.crc = ComputeSendMessageCrc(ext);
			ForwardToBluetooth(reinterpret_cast<const uint8_t*>(&ext), sizeof(ext));
			break;
		}
		default: {
			if (parsed.type == MsgType::FlightData || parsed.type == MsgType::FlightDataParity) {
				// Locator is in DataRequested — record burst timing and arm deferred-ACK.
				locator_in_profile_mode_    = true;
				last_flight_data_rx_ms_ = HAL_GetTick();
			} else if (parsed.type == MsgType::FlightMetadata) {
				// Locator is in flight-profile mode (MetadataRequested): it has
				// gone quiet and is listening, NOT running its ~1 s PreLaunchData
				// TX cycle.  Mark it so app→locator commands (notably the
				// FlightDataRequest) forward immediately instead of waiting for a
				// PreLaunchData window that never opens while the locator is quiet
				// — that wait was the ~24 s first-tap delay.
				locator_in_profile_mode_ = true;
			}
			ForwardToBluetooth(rx_payload_, rx_message_size_);
			break;
		}
		}
		RgbLed(RgbColor::Green, LedState::On);
	} else {
		// Suppress the red LED for a brief window after our own TX ends:
		// the TX→RX radio transition can produce a spurious OnRxDone with
		// a payload that fails our CRC, which is not a real receive error.
		if (HAL_GetTick() - last_radio_tx_end_ms_ >= kPostTxRxGuardMs)
			RgbLed(RgbColor::Red, LedState::On);
	}
	last_radio_rx_end_ms_ = HAL_GetTick();
	radio_rx_led_status_serviced_ = false;
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

	case MsgType::VersionInfo:
		return decode_message<MsgType::VersionInfo>(data, len, out);

	case MsgType::FlightMetadata:
		return decode_message<MsgType::FlightMetadata>(data, len, out);

	case MsgType::FlightData:
	case MsgType::FlightDataParity:
		// Variable-length packets: accept any size down to the fixed header fields.
		// The receiver only forwards these raw; it does not decode the payload.
		if (len < sizeof(PacketHeader) + 2 + 2 + 2 + 4)
			return ParseResult::TooShort;
		out.type = hdr.msg_type;
		return ParseResult::Ok;

	default:
		return ParseResult::UnknownType;
	}
}

void Communication::ServicePendingTx() {
	// Apply a deferred receiver channel switch once the forwarded locator config
	// change has finished transmitting.  Done here (main loop), not right after
	// radio_->Send(), so we never change RF frequency mid-transmit and corrupt the
	// packet the locator needs.  Waits for a TxDone recorded after we armed, plus
	// the standard post-TX settle guard.
	if (pending_locator_channel_switch_ &&
			(int32_t)(last_radio_tx_end_ms_ - locator_channel_switch_armed_ms_) >= 0 &&
			HAL_GetTick() - last_radio_tx_end_ms_ >= kPostTxRxGuardMs) {
		pending_locator_channel_switch_ = false;
		RocketPersistentSettings &receiver_settings = archive_.GetReceiverSettings();
		if (pending_locator_channel_ != receiver_settings.lora_channel) {
			receiver_settings.lora_channel = pending_locator_channel_;
			archive_.SaveReceiverSettings(receiver_settings);
			SetChannel(pending_locator_channel_);
		}
	}

	if (!pending_tx_.ready)
		return;

	if (locator_in_profile_mode_ &&
			pending_tx_.msg.header.msg_type == MsgType::FlightDataAck) {
		// Deferred-ACK strategy: hold the ACK until the locator's burst has
		// ended (no FlightData packet received for kAckDeferMs).  This ensures
		// the locator's radio is in RX — not mid-TX on the next burst packet —
		// when the ACK arrives, preventing the half-duplex collision that
		// caused the transfer to stall after the first packet.
		if (HAL_GetTick() - last_flight_data_rx_ms_ < kAckDeferMs)
			return;
	} else if (!locator_in_profile_mode_) {
		// PreLaunchData collision-avoidance: only forward to the locator during
		// the safe window [kPostPrelaunchMinMs, kPostPrelaunchMaxMs) after the
		// last received PreLaunchData.  This ensures the locator's radio is in
		// RX (not mid-TX on its ~1 s periodic PreLaunchData burst) when our
		// message arrives.  If no PreLaunchData has been seen yet, block
		// entirely — we have no timing reference and should not transmit blind.
		if (!locator_periodic_ever_rx_)
			return;
		const uint32_t elapsed = HAL_GetTick() - last_locator_periodic_rx_ms_;
		if (elapsed < kPostPrelaunchMinMs || elapsed >= kPostPrelaunchMaxMs)
			return;
	}

	// A LocatorCfgChgRequest may carry a new LoRa channel.  Capture it *before* the
	// send so we can arm a deferred switch: the receiver must follow the locator
	// onto the new channel, but only after this forward has fully transmitted (see
	// the deferred-apply block at the top of this function).
	bool arm_channel_switch = false;
	uint8_t forwarded_channel = 0;
	if (pending_tx_.msg.header.msg_type == MsgType::LocatorCfgChgRequest) {
		// payload holds the raw LocatorRocketSettings body (after the header).
		constexpr size_t kChanOffset =
				offsetof(LocatorRocketSettings, lora_channel) - sizeof(PacketHeader);
		forwarded_channel = pending_tx_.msg.payload[kChanOffset];
		arm_channel_switch = (forwarded_channel != archive_.GetReceiverSettings().lora_channel);
	}

	radio_->Send(reinterpret_cast<const uint8_t*>(&pending_tx_.msg),
			sizeof(PacketHeader) + pending_tx_.len);
	pending_tx_.ready = false;

	if (arm_channel_switch) {
		pending_locator_channel_ = forwarded_channel;
		locator_channel_switch_armed_ms_ = HAL_GetTick();
		pending_locator_channel_switch_ = true;
	}
}

void Communication::QueueBleNameUpdate(const char* name) {
	if (name != nullptr && name[0] != 0) {
		std::memcpy(pending_ble_name_, name, device_name_length);
		ble_name_update_pending_ = true;
	}
}

void Communication::ServiceBleNameUpdate() {
	if (!ble_name_update_pending_) return;
	ble_name_update_pending_ = false;

	// Build: AT+LENA<name>\r\n
	char cmd[sizeof(change_ble_name_) - 1 + device_name_length + 2] = { 0 };
	std::memcpy(cmd, change_ble_name_, sizeof(change_ble_name_) - 1);
	uint8_t i = 0;
	for (; i < device_name_length; i++) {
		if (pending_ble_name_[i] == 0) break;
		cmd[sizeof(change_ble_name_) - 1 + i] = pending_ble_name_[i];
	}
	cmd[sizeof(change_ble_name_) - 1 + i++] = '\r';
	cmd[sizeof(change_ble_name_) - 1 + i++] = '\n';

	// Enter command mode, update name, reset module.
	// The BLE reset drops the active connection; the app reconnects by MAC address.
	HAL_UART_Transmit(&huart1_, (uint8_t*) command_mode_, sizeof(command_mode_) - 1, 100);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1_, (uint8_t*) cmd, sizeof(change_ble_name_) - 1 + i, 100);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1_, (uint8_t*) reset_, sizeof(reset_) - 1, 100);
	HAL_Delay(100);
}

void Communication::ServiceReceiverInfoResponse() {
	if (!receiver_info_response_pending_) return;
	receiver_info_response_pending_ = false;

	ReceiverInfoMessage msg { };
	msg.header.system_id = system_id;
	msg.header.msg_type  = MsgType::ReceiverInfo;
	msg.header.msg_count = 0;
	msg.header.crc       = 0;

	const RocketPersistentSettings& settings = archive_.GetReceiverSettings();
	msg.lora_channel = settings.lora_channel;
	std::memcpy(msg.device_name, settings.device_name, device_name_length);

	msg.header.crc = ComputeMessageCrc(msg);
	ForwardToBluetooth(reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
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
			message_length_ = sizeof(FlightDataAck) - sizeof(PacketHeader);
			break;
		case MsgType::DeploymentTestRequest:
			message_length_ = 1;
			break;
		case MsgType::ReceiverInfoRequest:
			message_length_ = 0;
			break;
		case MsgType::VersionRequest:
			message_length_ = 0;
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
				if (current_msg_.header.msg_type == MsgType::ReceiverInfoRequest) {
					// Handled locally — queue a BLE response, never forward to locator.
					receiver_info_response_pending_ = true;
				} else {
					// Zero-payload message (e.g. ArmRequest, DisarmRequest): queue for
					// timed forwarding to the locator.
					pending_tx_.msg   = current_msg_;
					pending_tx_.len   = message_length_;
					pending_tx_.ready = true;
				}
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
				// Receiver config is handled locally; never forwarded to the locator.
				RocketPersistentSettings &receiver_settings = archive_.GetReceiverSettings();
				// Capture old name before overwriting so we can skip the BLE module reset
				// when only the channel changed (reset drops the active BLE connection).
				char old_name[device_name_length];
				std::memcpy(old_name, receiver_settings.device_name, device_name_length);
				receiver_settings.lora_channel = current_msg_.payload[0];
				for (uint8_t i = 0; i < device_name_length; i++)
					receiver_settings.device_name[i] = current_msg_.payload[1 + i];
				archive_.SaveReceiverSettings(receiver_settings);
				SetChannel(receiver_settings.lora_channel);
				// Only reset the BLE module when the advertised name actually changed;
				// the reset drops the active connection and delays channel confirmation.
				if (std::memcmp(old_name, receiver_settings.device_name, device_name_length) != 0)
					QueueBleNameUpdate(receiver_settings.device_name);
				// Automatically send ReceiverInfo so the app gets confirmation without
				// waiting for it to poll with a ReceiverInfoRequest.
				receiver_info_response_pending_ = true;
			} else {
				// All other payloaded messages: validate CRC and queue for timed forwarding.
				if (ValidateCRC(reinterpret_cast<const uint8_t*>(&current_msg_),
						sizeof(PacketHeader) + message_length_)) {
					pending_tx_.msg   = current_msg_;
					pending_tx_.len   = message_length_;
					pending_tx_.ready = true;
				}
			}
			Reset();
		}
		break;
	}
}

} // namespace Communication
