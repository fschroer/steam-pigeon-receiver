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
#include "StaticStringWriter.hpp"
#include "UserInteraction.hpp"   // UART_LINE_MAX_LENGTH

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

void Communication::OnRadioRxError() {
	// A frame was demodulated and failed the radio's own CRC: the direct signature
	// of a collision, and until now discarded entirely one layer below us.
	// Suppressed for the same window as the red LED, since the TX->RX transition
	// manufactures one of these after every transmission, and during a survey or a
	// locator search, where the radio is parked on channels whose failures say
	// nothing about home.
	if (survey_active_ || search_active_)
		return;
	if (HAL_GetTick() - last_radio_tx_end_ms_ < kPostTxRxGuardMs)
		return;
	if (bad_frame_count_ < UINT8_MAX)
		bad_frame_count_++;
	bad_frames_total_++;
	RgbLed(RgbColor::Red, LedState::On);
	last_radio_rx_end_ms_ = HAL_GetTick();
	radio_rx_led_status_serviced_ = false;
}

void Communication::ProcessRadioRx() {
	ParsedMessage parsed { };
	if (ParseLoraFrame(rx_payload_, rx_message_size_, system_id, parsed) == ParseResult::Ok) {
		// A frame that decodes while a confirm dwell is in progress was transmitted
		// ON the channel being dwelt — off-channel bleed does not survive the
		// demodulator, however loud it is.  That makes this the one unambiguous
		// answer to "is another locator using this channel", which RSSI cannot give
		// (#33): a locator a few feet away raises the level on every channel at
		// once, so power says "busy" everywhere and distinguishes nothing.
		//
		// Counted and then DROPPED, not relayed.  During a sweep these are other
		// people's broadcasts on channels we are only visiting; forwarding them put
		// a stranger's PreLaunchData in front of the app mid-scan and raised a
		// conflict banner for a locator it is not sharing a channel with at all.
		if (survey_active_ || search_active_) {
			// Who sent it, as the frame CLAIMS.  Cleartext locator_id, straight off
			// the wire, and only PreLaunchData and TelemetryData carry one — the
			// receiver has no password and cannot check auth_tag, so this is
			// identity to label a channel with, never identity to trust.
			uint32_t sender_id = 0;
			uint8_t  sender_armed = 0;
			const char* sender_name = nullptr;
			if (parsed.type == MsgType::PreLaunchData) {
				sender_id    = parsed.prelaunch.locator_id;
				sender_armed = parsed.prelaunch.armed;
				sender_name  = parsed.prelaunch.device_name;
			} else if (parsed.type == MsgType::TelemetryData) {
				sender_id    = parsed.telemetry.locator_id;
				sender_armed = parsed.telemetry.armed;
				// TelemetryData carries no device_name; the id is all there is.
			}
			if (survey_active_ && survey_phase_ == SurveyPhase::Confirm &&
					survey_confirm_index_ < kSurveyConfirmCount) {
				if (survey_confirm_frames_[survey_confirm_index_] < UINT8_MAX)
					survey_confirm_frames_[survey_confirm_index_]++;
				// First id wins.  A channel with two locators on it is occupied
				// either way, and the count already says how busy it is.
				if (survey_confirm_locator_id_[survey_confirm_index_] == 0)
					survey_confirm_locator_id_[survey_confirm_index_] = sender_id;
			}
			if (search_active_ && !search_hit_) {
				search_hit_       = true;
				search_hit_id_    = sender_id;
				search_hit_rssi_  = rssi_;
				search_hit_snr_   = LoraSnr_FskCfo_;
				search_hit_armed_ = sender_armed;
				if (sender_name != nullptr)
					std::memcpy(search_hit_name_, sender_name, device_name_length);
				else
					std::memset(search_hit_name_, 0, device_name_length);
			}
			RgbLed(RgbColor::Green, LedState::On);
			last_radio_rx_end_ms_ = HAL_GetTick();
			radio_rx_led_status_serviced_ = false;
			return;
		}
		switch (parsed.type) {
		case MsgType::PreLaunchData: {
			last_locator_periodic_rx_ms_ = HAL_GetTick();
			locator_periodic_ever_rx_    = true;
			locator_in_profile_mode_         = false;  // locator returned to Disarmed
			locator_armed_                   = parsed.prelaunch.armed != 0;
			// PreLaunchData carries no flight_state to read.  It is the on-pad
			// message by construction — a locator that has left the pad sends
			// TelemetryData, armed or not (#36) — so there is no in-flight case
			// to miss here.
			locator_in_flight_               = false;
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
			ext.snr = LoraSnr_FskCfo_;
			ext.noise_floor = TakeNoiseFloor();
			ext.bad_frames = TakeBadFrameCount();
			// Recompute CRC over the extended struct
			ext.base.packet_header.crc = ComputeSendMessageCrc(ext);
			ForwardToBluetooth(reinterpret_cast<const uint8_t*>(&ext), sizeof(ext));
			break;
		}
		case MsgType::TelemetryData: {
			last_locator_periodic_rx_ms_ = HAL_GetTick();
			locator_periodic_ever_rx_    = true;
			// Read, not inferred (ADR-0021 Decision 3, #35).  These are two
			// different questions and #36 splits them: a DISARMED locator
			// broadcasts telemetry in flight, so "sent TelemetryData" stops
			// meaning "armed" and the survey gate below needs both.
			locator_armed_               = parsed.telemetry.armed != 0;
			locator_in_flight_           = parsed.telemetry.flight_state != FlightStates::WaitingLaunch
			                            && parsed.telemetry.flight_state != FlightStates::Landed;
			TelemetryMessageExtended ext { };
			std::memcpy(&ext.base, &parsed.telemetry, sizeof(TelemetryData));
			ext.rssi = rssi_;
			ext.snr = LoraSnr_FskCfo_;
			ext.noise_floor = TakeNoiseFloor();
			ext.bad_frames = TakeBadFrameCount();
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
		case MsgType::DeploymentTest: {
			// The countdown IS the locator's periodic transmission while a
			// deployment test runs.  In DeviceState::Test it sends this and
			// nothing else — PreLaunchData and TelemetryData both stop, because
			// both live in the Disarmed/Armed branch of its per-state switch.
			//
			// Without this the safe-window reference below freezes at the last
			// PreLaunchData heard BEFORE the test began, the window shuts ~700 ms
			// in and never reopens, and every app→locator command queued for the
			// rest of the test sits in pending_tx_ undelivered.  The one that
			// matters is the CANCEL: the operator presses stop, the receiver
			// silently holds the frame, and the charge fires on schedule.
			//
			// This is the same failure the flight-profile branch below already
			// fixes for metadata and data bursts ("a PreLaunchData window that
			// never opens while the locator is quiet").  The deployment test is
			// the third mode where the locator stops sending PreLaunchData, and
			// it was the one still unhandled.
			//
			// Treated as a timing reference rather than by setting
			// locator_in_profile_mode_: this message keeps the same ~1 Hz cadence
			// out of the same super-loop as PreLaunchData, so it is an equally
			// good collision-avoidance reference and the existing window offsets
			// apply unchanged.  Profile mode means the locator has gone QUIET and
			// forwarding is unconstrained — which is not true here, and would put
			// our transmission on top of the next countdown.
			last_locator_periodic_rx_ms_ = HAL_GetTick();
			locator_periodic_ever_rx_    = true;
			ForwardToBluetooth(rx_payload_, rx_message_size_);
			break;
		}
		default: {
			if (parsed.type == MsgType::FlightData || parsed.type == MsgType::FlightDataParity) {
				// Locator is in DataRequested — record burst timing and arm deferred-ACK.
				locator_in_profile_mode_    = true;
				last_flight_data_rx_ms_ = HAL_GetTick();
			} else if (parsed.type == MsgType::FlightMetadata || parsed.type == MsgType::FlightEvents) {
				// Locator is in flight-profile mode (MetadataRequested, or the
				// event summary that precedes a data burst): it has
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
		if (HAL_GetTick() - last_radio_tx_end_ms_ >= kPostTxRxGuardMs) {
			RgbLed(RgbColor::Red, LedState::On);
			bad_frames_total_++;
			// A frame arrived and did not survive (ADR-0019).  This is the most
			// direct evidence of collision the system has, and it was previously
			// lit on an LED and discarded.  It beats inferring from a gap: a gap
			// might be a locator switched off, whereas a corrupted frame proves
			// something transmitted and was destroyed.  Reaches us because the
			// driver no longer drops hardware CRC mismatches (#16), so collision
			// garbage lands here instead of vanishing inside radio_driver.c.
			//
			// Not counted during a survey or a search: the radio is parked on other
			// channels then, and failures there say nothing about the home channel.
			if (!survey_active_ && !search_active_ && bad_frame_count_ < UINT8_MAX)
				bad_frame_count_++;
		}
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

	case MsgType::FlightEvents:
		return decode_message<MsgType::FlightEvents>(data, len, out);

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

uint8_t Communication::TakeBadFrameCount() {
	const uint8_t n = bad_frame_count_;
	bad_frame_count_ = 0;
	return n;
}

int16_t Communication::TakeNoiseFloor() {
	const int16_t peak = noise_floor_peak_;
	noise_floor_peak_ = kNoiseFloorUnknown;
	return peak;
}

void Communication::ServiceNoiseFloor() {
	if (radio_ == nullptr || radio_busy_)
		return;
	// A survey or a search parks the radio on other channels; anything sampled then
	// belongs to a different frequency and would poison the home channel's floor.
	if (survey_active_ || search_active_)
		return;
	const uint32_t now = HAL_GetTick();
	// Never read RSSI while our own transmit is still settling.
	if (now - last_radio_tx_end_ms_ < kPostTxRxGuardMs)
		return;
	// Sample only inside the ADR-0009 safe window, for two reasons.  First, that
	// is the interval in which the locator is known to be listening rather than
	// transmitting, so we measure the channel and not the locator's own carrier —
	// sampling outside it would report our own link as interference.  Second, the
	// window opens just after an RxDone, and the SubGHz PHY re-arms Rx() on every
	// event, so the radio is reliably in RX (Rssi() is meaningless in standby).
	//
	// In flight-profile mode the locator bursts on no fixed schedule, so no safe
	// window exists and sampling stops.  That is also when the user is on the
	// ground pulling data and has no use for it.
	if (locator_in_profile_mode_)
		return;
	const uint32_t elapsed = now - last_locator_periodic_rx_ms_;
	const bool in_safe_window =
			(locator_periodic_ever_rx_ &&
			 elapsed >= kPostPrelaunchMinMs && elapsed < kPostPrelaunchMaxMs);
	// Once a broadcast is overdue we are ALREADY missing packets — and explaining
	// why is the entire point of this measurement.  Restricting sampling to the
	// safe window meant the floor stopped being measured exactly when the channel
	// was at its worst, so a co-channel interferer that was destroying broadcasts
	// reported a clean floor and no alert.
	//
	// Sampling here cannot produce a false positive: the only thing that could
	// contaminate it is the locator's own carrier, and if its carrier were audible
	// and uncollided we would have received the packet and not be overdue.  Hearing
	// energy here while missing the broadcast IS the collision we are looking for.
	// A locator that is simply switched off leaves a quiet channel and raises nothing.
	const bool overdue = (elapsed >= kPeriodicOverdueMs);
	if (!in_safe_window && !overdue)
		return;
	if (now - last_noise_sample_ms_ < kNoiseSampleIntervalMs)
		return;
	last_noise_sample_ms_ = now;

	const int16_t rssi = radio_->Rssi();
	if (noise_floor_peak_ == kNoiseFloorUnknown || rssi > noise_floor_peak_)
		noise_floor_peak_ = rssi;
}

void Communication::SurveyTraceLine(const char* tag, int32_t a, int32_t b) {
	StaticStringWriter<UART_LINE_MAX_LENGTH> line(&huart2_);
	line.WriteMany("[survey] ", tag, " ", a, " ", b, "\r\n");
}

void Communication::SurveyTraceCoarseTable() {
	// Eight channels per line, so the whole band is legible in eight lines and the
	// blocking UART cost lands between phases rather than inside a dwell.
	for (uint8_t base = 0; base < kSurveyChannelCount; base += 8) {
		StaticStringWriter<UART_LINE_MAX_LENGTH> line(&huart2_);
		line.Clear();
		line.Buffer().AppendMany("[survey] c", static_cast<uint32_t>(base), ":");
		for (uint8_t i = 0; i < 8 && (base + i) < kSurveyChannelCount; i++) {
			line.Buffer().Append(' ');
			line.Buffer().AppendPadded(static_cast<int32_t>(survey_level_[base + i]), 5);
		}
		line.Buffer().Append("\r\n");
		line.Flush();
	}
}

void Communication::BeginChannelSurvey() {
	// Every path out of here must queue a response.  Returning silently leaves the
	// app waiting on a reply that will never come, which is indistinguishable from
	// a receiver whose firmware does not support the survey at all.
	if (radio_ == nullptr) {
		survey_status_ = ChannelSurveyStatus::RefusedBusy;
		survey_response_pending_ = true;
		return;
	}
	if (survey_active_ || search_active_) {
		// A sweep or a search is already running; report rather than ignoring the
		// request.  The running one will answer separately, and a duplicate response
		// is harmless because the app matches on arrival, not on request.
		survey_status_ = ChannelSurveyStatus::RefusedBusy;
		survey_response_pending_ = true;
		return;
	}
	// Same guard as BeginLocatorSearch, for the same reason -- see the long note
	// there.  A survey started on top of a queued command is cancelled by
	// ServicePendingTx on its first pass, which reads to the user as a scan that
	// refused itself.
	if (pending_tx_.ready && IsOperatorCommand(pending_tx_.msg.header.msg_type)) {
		survey_status_ = ChannelSurveyStatus::RefusedBusy;
		survey_response_pending_ = true;
		SurveyTraceLine("refused pending command", 0, 0);
		return;
	}
	// Enforce the refusals here rather than trusting the app's gate, which is
	// soft (ADR-0006).  A sweep is deaf to the locator for ~1 s: harmless on the
	// ground, lost telemetry in flight — and the channel cannot be changed in
	// flight anyway, so the result would be unusable even if we took it.
	if (locator_armed_ || locator_in_flight_) {
		survey_status_ = ChannelSurveyStatus::RefusedArmed;
		survey_response_pending_ = true;
		SurveyTraceLine("refused armed", 0, 0);
		return;
	}
	if (locator_in_profile_mode_) {
		survey_status_ = ChannelSurveyStatus::RefusedBusy;
		survey_response_pending_ = true;
		SurveyTraceLine("refused profile-mode", 0, 0);
		return;
	}

	survey_home_channel_ = archive_.GetReceiverSettings().lora_channel;
	survey_status_  = ChannelSurveyStatus::Ok;
	survey_active_  = true;
	survey_phase_   = SurveyPhase::Coarse;
	survey_visit_   = 0;
	survey_channel_ = 0;
	survey_channel_peak_ = kNoiseFloorUnknown;
	survey_confirm_count_ = 0;
	survey_confirm_index_ = 0;
	for (uint8_t i = 0; i < kSurveyConfirmCount; i++) {
		survey_confirm_frames_[i] = 0;
		survey_confirm_locator_id_[i] = 0;
	}
	for (uint8_t i = 0; i < kSurveyChannelCount; i++)
		survey_level_[i] = 0;
	SetChannel(survey_channel_);
	survey_channel_start_ms_ = HAL_GetTick();
	survey_start_ms_ = survey_channel_start_ms_;
	survey_last_sample_ms_ = survey_channel_start_ms_;
	SurveyTraceLine("start home", static_cast<int32_t>(survey_home_channel_), 0);
}

void Communication::BeginSurveyConfirmPhase() {
	SurveyTraceLine("coarse done ms", static_cast<int32_t>(HAL_GetTick() - survey_start_ms_), 0);
	SurveyTraceCoarseTable();
	// Shortlist the quietest coarse candidates.  Selection rather than a sort: we
	// only need the few we might recommend, and this runs on the main loop.
	bool taken[kSurveyChannelCount] = { };
	survey_confirm_count_ = 0;
	// Slot 0 is always the home channel.
	//
	// It is the channel the user is actually ON, and it was previously confirmed
	// only by luck: in one bench sweep it tied with sixteen others at the noise
	// floor — its own locator's burst having been missed by the 12 ms coarse dwell —
	// and was never evaluated. "Is my current channel contested?" is the first
	// question a survey should answer and it could not.
	//
	// It also gives the frame counter deterministic coverage. Our own locator
	// transmits here, so every sweep exercises the decode path on a channel known
	// to be occupied, instead of waiting for a coarse miss to coincide with a
	// favorable tie-break.
	if (survey_home_channel_ < kSurveyChannelCount) {
		taken[survey_home_channel_] = true;
		survey_confirm_channel_[survey_confirm_count_++] = survey_home_channel_;
	}
	while (survey_confirm_count_ < kSurveyConfirmCount) {
		// Quietest level still available.
		bool found = false;
		int8_t best_level = 0;
		for (uint8_t ch = 0; ch < kSurveyChannelCount; ch++) {
			if (taken[ch])
				continue;
			if (!found || survey_level_[ch] < best_level) {
				best_level = survey_level_[ch];
				found = true;
			}
		}
		if (!found)
			break;
		// Among the channels AT that level, take the one furthest from those
		// already chosen.
		//
		// At the noise floor a large fraction of the band ties — a real sweep had
		// 17 channels at -115 — and breaking those ties by channel number drew all
		// five candidates from one corner (11, 12, 14, 25, 28) every single run.
		// Two things went wrong with that. The recommendation only ever came from
		// the low end, so most of the band was never evaluated at all; and a
		// channel that ties but is high-numbered could never be confirmed, which
		// is exactly where the frame count would have caught an occupied channel
		// the coarse pass missed. In that same sweep the HOME channel tied at -115
		// (its own locator's burst was missed) and sat eleventh in the tie list.
		//
		// Deterministic, so repeat sweeps still agree: the first pick is the
		// lowest-numbered minimum, and each later pick maximizes its distance from
		// the ones already taken.
		uint8_t best = kSurveyChannelCount;
		uint8_t best_spread = 0;
		for (uint8_t ch = 0; ch < kSurveyChannelCount; ch++) {
			if (taken[ch] || survey_level_[ch] != best_level)
				continue;
			uint8_t spread = kSurveyChannelCount;   // nothing chosen yet
			for (uint8_t i = 0; i < survey_confirm_count_; i++) {
				const uint8_t sel = survey_confirm_channel_[i];
				const uint8_t d = (ch > sel) ? (ch - sel) : (sel - ch);
				if (d < spread)
					spread = d;
			}
			if (best == kSurveyChannelCount || spread > best_spread) {
				best = ch;
				best_spread = spread;
			}
		}
		if (best == kSurveyChannelCount)
			break;
		taken[best] = true;
		survey_confirm_channel_[survey_confirm_count_++] = best;
	}
	survey_phase_ = SurveyPhase::Confirm;
	survey_confirm_index_ = 0;
	// Discard the coarse reading for each shortlisted channel: it is the value we
	// distrust, and keeping it as a floor would let a lucky quiet sample mask what
	// the long dwell is about to find.
	for (uint8_t i = 0; i < survey_confirm_count_; i++)
		survey_level_[survey_confirm_channel_[i]] = 0;
	survey_channel_peak_ = kNoiseFloorUnknown;
	if (survey_confirm_count_ == 0) {
		FinishChannelSurvey();
		return;
	}
	for (uint8_t i = 0; i < survey_confirm_count_; i++)
		SurveyTraceLine("shortlist", static_cast<int32_t>(i),
				static_cast<int32_t>(survey_confirm_channel_[i]));
	survey_channel_ = survey_confirm_channel_[0];
	SetChannel(survey_channel_);
	survey_channel_start_ms_ = HAL_GetTick();
}

void Communication::FinishChannelSurvey() {
	survey_active_ = false;
	// Full restore, in this order.  Returning only the channel would leave the
	// radio on the home frequency but not listening if the PHY's RX timeout
	// happened to expire mid-sweep — the link would look dead for no visible
	// reason (ADR-0019 Decision 6).
	SetChannel(survey_home_channel_);
	// Same pairing as the search's restore, for the same reason (#40): the survey
	// owns the radio too, and a receiver channel change during one is undone here
	// while the setting keeps it.
	SurveyTraceLine("restored channel / setting",
			static_cast<int32_t>(survey_home_channel_),
			static_cast<int32_t>(archive_.GetReceiverSettings().lora_channel));
	if (radio_ != nullptr)
		radio_->Rx(kRxTimeoutMs);
	// The noise-floor accumulator sampled other channels during the sweep, so its
	// current peak describes the wrong frequency.  Discard it.
	noise_floor_peak_ = kNoiseFloorUnknown;
	survey_response_pending_ = true;
	SurveyTraceLine("done status/ms", static_cast<int32_t>(survey_status_),
			static_cast<int32_t>(HAL_GetTick() - survey_start_ms_));
	SurveyTraceLine("restored channel", static_cast<int32_t>(survey_home_channel_), 0);
}

void Communication::ServiceBadFrameTrace() {
	// One line per second while frames are failing, whether or not anything is
	// getting through.  This is the console counterpart of the red LED: each red
	// flash is one demodulated frame that did not survive, and during a collision
	// epoch that is the only evidence the receiver has.
	if (bad_frames_total_ == bad_frames_traced_)
		return;
	const uint32_t now = HAL_GetTick();
	if (now - last_bad_frame_trace_ms_ < kBadFrameTraceIntervalMs)
		return;
	last_bad_frame_trace_ms_ = now;
	SurveyTraceLine("bad frames since/total",
			static_cast<int32_t>(bad_frames_total_ - bad_frames_traced_),
			static_cast<int32_t>(bad_frames_total_));
	bad_frames_traced_ = bad_frames_total_;
}

void Communication::ServiceChannelSurvey() {
	if (survey_response_pending_) {
		survey_response_pending_ = false;
		ChannelSurveyResponse msg { };
		msg.packet_header.system_id = system_id;
		msg.packet_header.msg_type  = MsgType::ChannelSurvey;
		msg.packet_header.msg_count = 0;
		msg.packet_header.crc       = 0;
		msg.status        = static_cast<uint8_t>(survey_status_);
		msg.channel_count = (survey_status_ == ChannelSurveyStatus::Ok) ? kSurveyChannelCount : 0;
		msg.home_channel  = archive_.GetReceiverSettings().lora_channel;
		if (survey_status_ == ChannelSurveyStatus::Ok) {
			std::memcpy(msg.level, survey_level_, sizeof(msg.level));
			msg.confirmed_count = survey_confirm_count_;
			std::memcpy(msg.confirmed_channel, survey_confirm_channel_,
					sizeof(msg.confirmed_channel));
			std::memcpy(msg.confirmed_frames, survey_confirm_frames_,
					sizeof(msg.confirmed_frames));
			std::memcpy(msg.confirmed_locator_id, survey_confirm_locator_id_,
					sizeof(msg.confirmed_locator_id));
		}
		msg.packet_header.crc = ComputeMessageCrc(msg);
		ForwardToBluetooth(reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
		return;
	}
	if (!survey_active_ || radio_ == nullptr)
		return;

	// Abort rather than push on if the locator arms — or launches — mid-sweep.
	// Arming is a deliberate act by someone standing at the pad; finishing the
	// sweep would keep the receiver deaf through the first seconds of a live
	// flight.  The launch half matters once a disarmed locator can fly (#36):
	// there is no arm event to catch, so flight_state is the only signal.
	if (locator_armed_ || locator_in_flight_) {
		survey_status_ = ChannelSurveyStatus::RefusedArmed;
		FinishChannelSurvey();
		return;
	}

	const uint32_t now = HAL_GetTick();
	// Backstop: whatever goes wrong, restore the radio and answer.  The app has no
	// other way to learn a sweep died, and a silent receiver is the one failure it
	// cannot tell apart from unsupported firmware.
	if (now - survey_start_ms_ >= kSurveyDeadlineMs) {
		SurveyTraceLine("DEADLINE phase/ch", static_cast<int32_t>(survey_phase_),
				static_cast<int32_t>(survey_channel_));
		survey_status_ = ChannelSurveyStatus::RefusedBusy;
		FinishChannelSurvey();
		return;
	}

	const uint32_t dwell =
			(survey_phase_ == SurveyPhase::Coarse) ? kSurveyDwellMs : kSurveyConfirmDwellMs;
	const uint32_t elapsed = now - survey_channel_start_ms_;
	if (elapsed >= kSurveySettleMs && now - survey_last_sample_ms_ >= kSurveySampleIntervalMs) {
		survey_last_sample_ms_ = now;
		const int16_t rssi = radio_->Rssi();
		if (survey_channel_peak_ == kNoiseFloorUnknown || rssi > survey_channel_peak_)
			survey_channel_peak_ = rssi;
	}
	if (elapsed < dwell)
		return;

	survey_level_[survey_channel_] =
			(survey_channel_peak_ == kNoiseFloorUnknown) ? 0 : static_cast<int8_t>(survey_channel_peak_);
	survey_channel_peak_ = kNoiseFloorUnknown;

	if (survey_phase_ == SurveyPhase::Coarse) {
		if (++survey_visit_ >= kSurveyChannelCount) {
			BeginSurveyConfirmPhase();
			return;
		}
		survey_channel_ = static_cast<uint8_t>(
				(static_cast<uint16_t>(survey_visit_) * kSurveyCoarseStride) % kSurveyChannelCount);
	} else {
		SurveyTraceLine("confirm ch/level", static_cast<int32_t>(survey_channel_),
				static_cast<int32_t>(survey_level_[survey_channel_]));
		// Frames decoded here is the load-bearing number, not the level.
		SurveyTraceLine("confirm ch/frames", static_cast<int32_t>(survey_channel_),
				static_cast<int32_t>(survey_confirm_frames_[survey_confirm_index_]));
		// Who, not just how many — the number that turns "this channel is busy"
		// into "your Redline is sitting on it".
		SurveyTraceLine("confirm ch/id", static_cast<int32_t>(survey_channel_),
				static_cast<int32_t>(survey_confirm_locator_id_[survey_confirm_index_]));
		if (++survey_confirm_index_ >= survey_confirm_count_) {
			FinishChannelSurvey();
			return;
		}
		survey_channel_ = survey_confirm_channel_[survey_confirm_index_];
	}
	SetChannel(survey_channel_);
	survey_channel_start_ms_ = HAL_GetTick();
}

void Communication::SearchTraceLine(const char* tag, int32_t a, int32_t b) {
	StaticStringWriter<UART_LINE_MAX_LENGTH> line(&huart2_);
	line.WriteMany("[search] ", tag, " ", a, " ", b, "\r\n");
}

uint8_t Communication::SearchChannelAt(uint8_t index) const {
	// The whole-band run walks 0..63 in order rather than copying 64 numbers into
	// a 16-slot list.  Same walk either way, so everything downstream is unaware
	// of which kind of run it is servicing.
	return search_whole_band_ ? index : search_channel_[index];
}

void Communication::BeginLocatorSearch(const LocatorSearchRequest& req) {
	// Cancel rides on the same message, because it only means anything while a run
	// is in progress and the app already knows how to send this one.  Answered
	// even when nothing is running: silence is the one reply the app cannot tell
	// apart from firmware that has never heard of a search.
	if (req.flags & kSearchFlagCancel) {
		if (search_active_)
			FinishLocatorSearch(LocatorSearchStatus::Cancelled);
		else
			RefuseLocatorSearch(LocatorSearchStatus::Cancelled);
		return;
	}
	if (radio_ == nullptr) {
		RefuseLocatorSearch(LocatorSearchStatus::RefusedBusy);
		return;
	}
	if (search_active_) {
		// A run is already streaming, and that stream IS the answer to this request.
		// Queuing a refusal here instead would put a terminator in front of the app
		// mid-run — it would mark the search finished and then go on receiving
		// per-channel results for a run it believes has ended.  This is where the
		// survey's rule ("always answer, a duplicate response is harmless") stops
		// applying: a duplicate is harmless only for a single-shot response.
		return;
	}
	if (survey_active_) {
		RefuseLocatorSearch(LocatorSearchStatus::RefusedBusy);
		return;
	}
	// A COMMAND FOR THE LOCATOR IS ALREADY WAITING for its forwarding window, so a
	// sweep must not start on top of it.  ServicePendingTx ends a sweep the moment
	// anything is queued (the ADR-0029 decision-7 abort), and that rule reads a
	// message queued BEFORE the run as though the operator had just sent it -- so
	// the run is cancelled on its very first service pass, before one channel is
	// dwelt.  Reported 2026-08-30 as a search that "stops" if it is started the
	// instant the previous one finishes.
	//
	// The window is wider than it looks, and this is why the symptom appears
	// straight after a search rather than at random.  Forwarding is gated on the
	// safe interval after the last PreLaunchData, and ProcessRadioRx counts and
	// DROPS broadcasts during a sweep -- so last_locator_periodic_rx_ms_ is stale
	// when a sweep ends, and anything queued stays latched until the locator's next
	// broadcast, up to a full second later.  Waiting "a beat" is waiting for that
	// broadcast.
	//
	// Refusing rather than deferring keeps the operator's command first, which is
	// the whole point of the abort: a queued Arm goes out at the next window instead
	// of waiting out a run that could be 90 s long.  With this guard in place, any
	// pending_tx_ that ServicePendingTx sees during a run MUST have arrived during
	// it, which is exactly the condition that rule was written for.
	if (pending_tx_.ready && IsOperatorCommand(pending_tx_.msg.header.msg_type)) {
		RefuseLocatorSearch(LocatorSearchStatus::RefusedBusy);
		SearchTraceLine("refused pending command", 0, 0);
		return;
	}
	// The same enforcement the survey has, against a worse version of the same
	// hazard.  A survey is ~8 s of deafness; a whole-band search is up to ~90 s, so one
	// running over a live flight would lose the entire descent.  App-side gating is
	// soft (ADR-0006); this is the gate that actually holds.
	if (locator_armed_ || locator_in_flight_) {
		RefuseLocatorSearch(LocatorSearchStatus::RefusedArmed);
		SearchTraceLine("refused armed", 0, 0);
		return;
	}
	if (locator_in_profile_mode_) {
		RefuseLocatorSearch(LocatorSearchStatus::RefusedBusy);
		SearchTraceLine("refused profile-mode", 0, 0);
		return;
	}

	search_count_ = 0;
	search_whole_band_ = (req.channel_count == 0);
	if (!search_whole_band_) {
		const uint8_t requested = (req.channel_count > kSearchMaxChannels)
				? kSearchMaxChannels : req.channel_count;
		// Copy only channels that exist, and only once each.  The app builds this
		// list from several sources that legitimately overlap — a locator's last
		// known channel is often the receiver's current one — and a duplicate would
		// cost a full 1.2 s dwell to learn the same thing twice.
		for (uint8_t i = 0; i < requested; i++) {
			const uint8_t ch = req.channel[i];
			if (ch >= kSurveyChannelCount)
				continue;
			bool already = false;
			for (uint8_t j = 0; j < search_count_; j++)
				already = already || (search_channel_[j] == ch);
			if (!already)
				search_channel_[search_count_++] = ch;
		}
		if (search_count_ == 0) {
			// A list that was entirely out of range is a caller error, not an empty
			// band.  Refusing says so; falling through to a whole-band sweep would
			// turn a typo into 77 s of deafness.
			RefuseLocatorSearch(LocatorSearchStatus::RefusedBusy);
			SearchTraceLine("refused empty list", 0, 0);
			return;
		}
	}

	search_active_       = true;
	search_target_id_    = req.target_locator_id;
	search_index_        = 0;
	search_hit_          = false;
	search_hit_id_       = 0;
	search_hit_rssi_     = 0;
	search_hit_snr_      = 0;
	search_hit_armed_    = 0;
	std::memset(search_hit_name_, 0, device_name_length);
	search_home_channel_ = archive_.GetReceiverSettings().lora_channel;
	SetChannel(SearchChannelAt(0));
	search_start_ms_         = HAL_GetTick();
	search_channel_start_ms_ = search_start_ms_;
	SearchTraceLine("start channels/target",
			static_cast<int32_t>(search_whole_band_ ? kSurveyChannelCount : search_count_),
			static_cast<int32_t>(search_target_id_));
}

void Communication::RefuseLocatorSearch(LocatorSearchStatus status) {
	// Nothing was retuned, so there is nothing to restore — but the app still gets
	// an explicit end, out of the same queue every other ending uses.
	//
	// The run description is cleared ONLY when no run owns it.  The commonest
	// refusal is a second request arriving while the first is still sweeping, and
	// zeroing search_count_ there would cut the live run short on its next slice:
	// its total would read 0, and the very next channel boundary would satisfy
	// `search_index_ >= total` and end it as though it had finished.
	if (!search_active_) {
		search_count_      = 0;
		search_whole_band_ = false;
	}
	search_status_     = status;
	search_terminator_pending_ = true;
}

void Communication::FinishLocatorSearch(LocatorSearchStatus status) {
	search_active_ = false;
	// Full restore, in the survey's order and for the survey's reason (ADR-0019
	// Decision 6): channel first, then re-arm RX, or the radio sits on the right
	// frequency without listening and the link looks dead for no visible reason.
	SetChannel(search_home_channel_);
	if (radio_ != nullptr)
		radio_->Rx(kRxTimeoutMs);
	// Sampled on other channels throughout, so the accumulated peak describes the
	// wrong frequency.
	noise_floor_peak_ = kNoiseFloorUnknown;
	search_status_ = status;
	search_terminator_pending_ = true;
	SearchTraceLine("done status/ms", static_cast<int32_t>(status),
			static_cast<int32_t>(HAL_GetTick() - search_start_ms_));
	// BOTH values, because a mismatch between them is the whole of #40: a receiver
	// channel change applied while this run owned the radio is overwritten by the
	// next dwell and then undone here, while the persisted setting keeps the new
	// value — leaving the radio and the settings on different channels, invisibly,
	// since everything the app reads (ReceiverInfo, and the channel stamped on each
	// relayed frame) comes from the settings.  Printing one of the two could not
	// show that, which is why the split had to be reasoned about rather than seen.
	SearchTraceLine("restored channel / setting",
			static_cast<int32_t>(search_home_channel_),
			static_cast<int32_t>(archive_.GetReceiverSettings().lora_channel));
}

void Communication::SendSearchResult(LocatorSearchStatus status, uint8_t channel,
		uint8_t searched) {
	LocatorSearchResult msg { };
	msg.packet_header.system_id = system_id;
	msg.packet_header.msg_type  = MsgType::LocatorSearchResult;
	msg.packet_header.msg_count = 0;
	msg.packet_header.crc       = 0;
	msg.status   = static_cast<uint8_t>(status);
	msg.channel  = channel;
	msg.searched = searched;
	msg.total    = search_whole_band_ ? kSurveyChannelCount : search_count_;
	// Only a per-channel result carries a hit.  A terminator reports the ending and
	// nothing else, so the app never has to work out whether the fields on it mean
	// anything.
	if (status == LocatorSearchStatus::Progress && search_hit_) {
		msg.found      = 1;
		msg.armed      = search_hit_armed_;
		msg.rssi       = search_hit_rssi_;
		msg.snr        = search_hit_snr_;
		msg.locator_id = search_hit_id_;
		std::memcpy(msg.device_name, search_hit_name_, device_name_length);
	}
	msg.packet_header.crc = ComputeMessageCrc(msg);
	ForwardToBluetooth(reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void Communication::ServiceLocatorSearch() {
	if (search_terminator_pending_) {
		search_terminator_pending_ = false;
		SendSearchResult(search_status_, 0, 0);
		return;
	}
	if (!search_active_ || radio_ == nullptr)
		return;

	// Abort if the locator arms or launches mid-run, exactly as the survey does.
	// The window is ten times wider here, so this is the check that earns its keep:
	// someone can walk to the pad and arm while a whole-band run is still going.
	if (locator_armed_ || locator_in_flight_) {
		FinishLocatorSearch(LocatorSearchStatus::RefusedArmed);
		return;
	}

	const uint32_t now = HAL_GetTick();
	// Backstop: whatever goes wrong, restore the radio and terminate the stream.  A
	// search that dies silently leaves the receiver deaf on some other channel,
	// which is the worst state this feature can produce.
	if (now - search_start_ms_ >= kSearchDeadlineMs) {
		SearchTraceLine("DEADLINE ch/index",
				static_cast<int32_t>(SearchChannelAt(search_index_)),
				static_cast<int32_t>(search_index_));
		FinishLocatorSearch(LocatorSearchStatus::RefusedBusy);
		return;
	}

	// End the dwell early once the channel has answered.  There is ONE hit slot per
	// channel and the first frame fills it, so the rest of the dwell cannot learn
	// anything further about this channel -- it is time spent deaf for nothing.
	// This is what pays for the longer dwell (kSurveyConfirmDwellMs): a band with
	// locators on it now finishes FASTER than it did at 1200 ms, and only channels
	// with nothing on them wait out the full 1.4 s.
	if (!search_hit_ && now - search_channel_start_ms_ < kSearchDwellMs)
		return;

	const uint8_t total = search_whole_band_ ? kSurveyChannelCount : search_count_;
	const uint8_t channel = SearchChannelAt(search_index_);
	SearchTraceLine("channel/id", static_cast<int32_t>(channel),
			static_cast<int32_t>(search_hit_ ? search_hit_id_ : 0));
	// Only for a hit, and only on the bench path: this is the pair that tells a real
	// occupant from a near-field artifact, which cost a relocation to diagnose once.
	if (search_hit_)
		SearchTraceLine("channel rssi/snr", static_cast<int32_t>(search_hit_rssi_),
				static_cast<int32_t>(search_hit_snr_));
	SendSearchResult(LocatorSearchStatus::Progress, channel,
			static_cast<uint8_t>(search_index_ + 1));

	// Stop early only for a locator the app named.  With no target the run is a
	// census — "who is out there" — and stopping at the first hit would hide the
	// second rocket, which is one of the cases this exists for.
	const bool target_found = search_hit_ && search_target_id_ != 0
			&& search_hit_id_ == search_target_id_;
	search_hit_       = false;
	search_hit_id_    = 0;
	search_hit_rssi_  = 0;
	search_hit_snr_   = 0;
	search_hit_armed_ = 0;
	std::memset(search_hit_name_, 0, device_name_length);

	if (target_found || ++search_index_ >= total) {
		FinishLocatorSearch(LocatorSearchStatus::Done);
		return;
	}
	SetChannel(SearchChannelAt(search_index_));
	search_channel_start_ms_ = HAL_GetTick();
}

void Communication::ServicePendingTx() {
	// A forward that has waited past kPendingTxStaleMs is dropped: the flow that
	// queued it has given up, and delivering it now is worse than not delivering
	// it.  ADR-0011 found the shape of that harm — an undelivered
	// LocatorCfgChgRequest sat here indefinitely and would fire whenever a locator
	// was next heard, minutes later and against whatever channel the receiver had
	// since been pointed at, out of the flow that queued it.
	//
	// Checked FIRST, ahead of the sweep-cancel below, so a message nobody is
	// waiting on any more cannot end a scan on its way to being discarded.
	//
	// The FlightDataAck path cannot reach this limit legitimately: in profile mode
	// it waits only kAckDeferMs (600 ms) past the end of the locator's burst.
	if (pending_tx_.ready &&
			HAL_GetTick() - pending_tx_.queued_ms >= kPendingTxStaleMs)
		pending_tx_.ready = false;

	// Never transmit while a survey or a search has the radio parked on another
	// channel: the burst would go out on that frequency, so the locator would not
	// hear it and whatever is on that channel would.
	//
	// A QUEUED COMMAND ENDS THE SWEEP RATHER THAN WAITING BEHIND IT.  The previous
	// rule — let it wait, a sweep is soon over — rested on two claims and both were
	// wrong.  "It aborts the moment a locator arms" cannot happen: the armed flags
	// are only assigned in ProcessRadioRx's PreLaunchData/TelemetryData cases, which
	// a sweep returns before reaching, so they are frozen for its duration.  And "a
	// command queued against a locator we cannot hear had nowhere to go anyway" is
	// false for the one message where it matters: an ArmRequest goes out the instant
	// the sweep ends.
	//
	// Measured on the bench 2026-08-28: Arm pressed during a whole-band search did
	// nothing visible, and the locator armed when the sweep finished — up to 77 s
	// later, by which time the operator has read it as a failed arm and may be at
	// the pad.  A late pyro arming is a different class of problem from lost
	// telemetry.
	//
	// So the operator's command wins.  Ending the sweep also restores the radio to
	// the home channel, which is what makes the command deliverable at all — and it
	// is the abort ADR-0029 decision 7 wanted, reached by the path that actually
	// exists: the receiver cannot HEAR a locator arm, but it does see the app's
	// ArmRequest pass through it.  Sent on the next poll rather than here, so the
	// retune settles and every timing guard below still applies.
	if (survey_active_ || search_active_) {
		// Only an OPERATOR command ends a sweep -- see IsOperatorCommand.  The app's
		// own version poll was cancelling scans, and blaming the user for it.
		if (pending_tx_.ready &&
				IsOperatorCommand(pending_tx_.msg.header.msg_type)) {
			if (search_active_)
				FinishLocatorSearch(LocatorSearchStatus::Cancelled);
			if (survey_active_) {
				survey_status_ = ChannelSurveyStatus::Cancelled;
				FinishChannelSurvey();
			}
		}
		return;
	}
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
		// Announce the move we made on our OWN initiative.  A ReceiverCfgChgRequest
		// has always been answered with a ReceiverInfo; this follow was silent, and
		// the app is the one thing that needed to hear about it.  Reaching here is
		// proof the forward TRANSMITTED — the switch is armed only after Send() and
		// applied only after TxDone plus the settle guard — so this message is the
		// app's receipt that the channel change went on air, delivered over BLE
		// where the noisy channel that motivated the move cannot eat it.  ADR-0011
		// invariant 4 starts its confirm window from this, not from the BLE write:
		// on a lossy channel the wait for a forwarding window used to spend the
		// whole window before the command was even transmitted.
		receiver_info_response_pending_ = true;
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

	// Drain the same accumulators the extended broadcasts drain, with the same
	// "peak/count since the last report" meaning.  Sharing them is deliberate: a
	// sample must be reported exactly once, whichever message happens to carry it,
	// or a quiet interval could be described twice and a busy one averaged away.
	//
	// The cost of that sharing is that every reader SHORTENS the window the next
	// reader reports on, and a peak over a shorter window is never higher than one
	// over a longer window.  So a poll competing with live broadcasts would bias
	// the whole measurement downward, and the app's quietest-floor baseline keeps
	// the minimum of what it sees — it would drift down and make the relative
	// interference test creep toward firing on its own.
	//
	// That is why the app holds this poll back until the locator has been unheard
	// for several seconds (RocketViewModel.CHANNEL_WATCH_SILENCE_MS), rather than
	// starting at the first missed broadcast: a distant rocket routinely drops one
	// or two, and while it is still transmitting the surviving packets carry the
	// floor anyway.  By the time this poll runs, nothing is competing to drain.
	msg.noise_floor = TakeNoiseFloor();
	msg.bad_frames  = TakeBadFrameCount();

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
		// These four are addressed but otherwise payload-free (ADR-0020).
		case MsgType::ArmRequest:
		case MsgType::DisarmRequest:
		case MsgType::FlightMetadataRequest:
			message_length_ = sizeof(TargetedRequest) - sizeof(PacketHeader);
			break;
		case MsgType::FlightDataRequest:
			message_length_ = sizeof(TargetedRequest) - sizeof(PacketHeader) + 1;  // + record
			break;
		case MsgType::FlightDataAck:
			message_length_ = sizeof(FlightDataAck) - sizeof(PacketHeader);
			break;
		case MsgType::DeploymentTestRequest:
			message_length_ = sizeof(TargetedRequest) - sizeof(PacketHeader) + 1;  // + channel
			break;
		case MsgType::PadAlertSnoozeRequest:
			message_length_ = sizeof(TargetedRequest) - sizeof(PacketHeader) + 1;  // + minutes (#37)
			break;
		case MsgType::ReceiverInfoRequest:
			message_length_ = 0;
			break;
		case MsgType::VersionRequest:
			message_length_ = sizeof(TargetedRequest) - sizeof(PacketHeader);
			break;
		case MsgType::ChannelSurveyRequest:
			message_length_ = 0;
			break;
		case MsgType::LocatorSearchRequest:
			message_length_ = sizeof(LocatorSearchRequest) - sizeof(PacketHeader);
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
				} else if (current_msg_.header.msg_type == MsgType::ChannelSurveyRequest) {
					// Also receiver-local: the locator plays no part in a survey and
					// must never see this message.
					BeginChannelSurvey();
				} else {
					// Zero-payload message (e.g. ArmRequest, DisarmRequest): queue for
					// timed forwarding to the locator.
					pending_tx_.msg   = current_msg_;
					pending_tx_.len   = message_length_;
					pending_tx_.queued_ms = HAL_GetTick();
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
				// A SCAN OWNS THE RADIO, AND THE OPERATOR'S COMMAND ENDS IT (#40).
				//
				// Applied underneath a running scan this retune is overwritten by the
				// next dwell and then undone by the scan's home-restore, while
				// SaveReceiverSettings below keeps the new value — radio and settings
				// left on different channels.  Invisibly: both ReceiverInfo and the
				// receiver_lora_channel stamped on every relayed frame are read from
				// the settings, so the app is confidently wrong about where its own
				// receiver points and nothing in the protocol can contradict it.
				//
				// This is not a new rule.  ServicePendingTx already ends a sweep for a
				// queued OPERATOR COMMAND rather than letting it wait, because the
				// operator's intent wins and because ending the sweep is what restores
				// the radio and makes the command deliverable at all (ADR-0029). A
				// receiver channel change is an operator command by every test that
				// rule applies; it simply arrives receiver-local over BLE instead of
				// through pending_tx_, which is how it missed the guard.
				//
				// Deferring instead was rejected: a whole-band run is up to ~90 s, and
				// a tap that silently does nothing for that long is the failure the
				// Communication screen was reorganised to eliminate.
				//
				// Cancel FIRST, then apply.  Each Finish* restores its own home channel
				// and re-arms RX, so the assignment below lands last and the final state
				// is identical to an ordinary channel change — which is the known-good
				// path.  Both scans, because the survey has the identical hole.
				if (search_active_)
					FinishLocatorSearch(LocatorSearchStatus::Cancelled);
				if (survey_active_) {
					survey_status_ = ChannelSurveyStatus::Cancelled;
					FinishChannelSurvey();
				}
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
			} else if (current_msg_.header.msg_type == MsgType::LocatorSearchRequest) {
				// Receiver-local, like the survey: the locator plays no part in a
				// search and must never see this message.  Forwarding it would put an
				// unknown MsgType on the air for every locator in earshot to parse.
				if (ValidateCRC(reinterpret_cast<const uint8_t*>(&current_msg_),
						sizeof(PacketHeader) + message_length_)) {
					LocatorSearchRequest req { };
					req.packet_header = current_msg_.header;
					std::memcpy(reinterpret_cast<uint8_t*>(&req) + sizeof(PacketHeader),
							current_msg_.payload, message_length_);
					BeginLocatorSearch(req);
				}
			} else {
				// All other payloaded messages: validate CRC and queue for timed forwarding.
				if (ValidateCRC(reinterpret_cast<const uint8_t*>(&current_msg_),
						sizeof(PacketHeader) + message_length_)) {
					pending_tx_.msg   = current_msg_;
					pending_tx_.len   = message_length_;
					pending_tx_.queued_ms = HAL_GetTick();
					pending_tx_.ready = true;
				}
			}
			Reset();
		}
		break;
	}
}

} // namespace Communication
