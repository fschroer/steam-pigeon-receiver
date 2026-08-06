#pragma once

extern "C" {
#include "usart.h"
}

#include <type_traits>
#include "PowerManagement.hpp"
#include "MessageProtocol.hpp"
#include "DeviceUID.hpp"

namespace Communication {

enum class ParseResult {
    Ok,
    TooShort,
    SystemIdMismatch,
    CrcMismatch,
    LengthMismatch,
    UnknownType
};

template<typename T>
ParseResult decode_into(const uint8_t* data, std::size_t len, T& out)
{
    static_assert(std::is_trivially_copyable_v<T>,
                  "Message type must be trivially copyable");

    if (len != sizeof(T))
        return ParseResult::LengthMismatch;

    std::memcpy(&out, data, sizeof(T));
    return ParseResult::Ok;
}

template<MsgType M> struct MsgTraits;

template<> struct MsgTraits<MsgType::Startup> {
    using type = StartupMessage;
    static constexpr auto field = &ParsedMessage::startup;
};

template<> struct MsgTraits<MsgType::PreLaunchData> {
    using type = PreLaunchData;
    static constexpr auto field = &ParsedMessage::prelaunch;
};

template<> struct MsgTraits<MsgType::TelemetryData> {
    using type = TelemetryData;
    static constexpr auto field = &ParsedMessage::telemetry;
};

template<> struct MsgTraits<MsgType::DeploymentTest> {
    using type = DeploymentTestCountdownMessage;
    static constexpr auto field = &ParsedMessage::deployment_test;
};

template<> struct MsgTraits<MsgType::FlightMetadata> {
    using type = FlightMetadata;
    static constexpr auto field = &ParsedMessage::flight_metadata;
};

template<> struct MsgTraits<MsgType::FlightEvents> {
    using type = FlightEventsMessage;
    static constexpr auto field = &ParsedMessage::flight_events;
};

template<> struct MsgTraits<MsgType::FlightData> {
    using type = FlightDataPacket;
    static constexpr auto field = &ParsedMessage::flight_data_packet;
};

template<> struct MsgTraits<MsgType::FlightDataParity> {
    using type = FlightDataPacket;
    static constexpr auto field = &ParsedMessage::flight_data_packet;
};

template<> struct MsgTraits<MsgType::VersionInfo> {
    using type = VersionInfoMessage;
    static constexpr auto field = &ParsedMessage::version_info;
};

template<MsgType M>
ParseResult decode_message(const uint8_t* data, std::size_t len, ParsedMessage& out)
{
    auto field = MsgTraits<M>::field;

    auto result = decode_into(data, len, out.*field);
    if (result == ParseResult::Ok)
        out.type = M;

    return result;
}

// Simple radio interface so we don't hide globals
class IRadio
{
public:
    virtual ~IRadio() = default;
    virtual void Send(const uint8_t* data, size_t len) = 0;
    virtual void Rx(uint32_t timeout_ms) = 0;
    virtual void SetChannel(uint32_t freq) = 0;
    // Instantaneous RSSI on the current channel.  Read while idle in RX this is
    // the channel's occupancy — noise plus whatever else is transmitting —
    // independent of whether we can decode any of it (ADR-0019).  No default
    // implementation: a silent one would let the sampling path go untested.
    virtual int16_t Rssi() = 0;
};

class Communication{
public:
	Communication(DeviceUID& deviceUID, Archive& archive, PowerManagement& power, UART_HandleTypeDef& huart1, UART_HandleTypeDef& huart2);
	void Init(IRadio& radio);
	void SetChannel(uint8_t channel);
	void OnRadioTxDone();   // called from ISR/callback
	void OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);   // ACK reception handler
	void ProcessRadioRx();
	void ForwardToBluetooth(const uint8_t* buf, std::size_t len);
	void UpdateStatusLeds();
	void OnUART1Char(uint8_t uart_char);
	// Call from the main loop: sends any pending outbound LoRa message once
	// the timing window is safe (not near the next expected PreLaunchData TX).
	void ServicePendingTx();
	// Call from the main loop: takes an idle-channel RSSI sample when the timing
	// is known safe, and accumulates the peak for the next broadcast (ADR-0019).
	void ServiceNoiseFloor();
	// Call from the main loop: advances the channel survey one slice per call
	// (ADR-0019 tier 3).  Never blocks for the whole sweep.
	void ServiceChannelSurvey();
	void ServiceBleNameUpdate();
	void QueueBleNameUpdate(const char* name);
	void ServiceReceiverInfoResponse();

private:
	DeviceUID& deviceUID_;
	Archive& archive_;
	PowerManagement& power_;
	UART_HandleTypeDef& huart1_;
	UART_HandleTypeDef& huart2_;
	IRadio* radio_ = nullptr;

	bool radio_busy_ = false;
	uint32_t current_tick_ = 0;
	uint32_t last_radio_tx_end_ms_ = 0;
	bool radio_tx_led_status_serviced_ = true;

	// How long after our own TX completes to ignore RX parse failures.
	// The STM32WL TX→RX transition can produce a spurious OnRxDone with
	// garbage payload that passes the hardware LoRa CRC but fails our
	// application CRC.  Suppressing the red LED for this window avoids
	// false error indications after every arm/disarm transmission.
	static constexpr uint32_t kPostTxRxGuardMs = 100u;
	uint32_t last_radio_rx_end_ms_ = 0;
	bool radio_rx_led_status_serviced_ = true;
	// Timestamp of the last received periodic locator packet (PreLaunchData or
	// TelemetryData).  Both are sent at ~1 s intervals on rocket_service_count==2.
	// Used by ServicePendingTx() to keep outbound TX in the safe window between
	// locator transmissions.
	uint32_t last_locator_periodic_rx_ms_ = 0;
	bool     locator_periodic_ever_rx_    = false;

	// Safe TX window relative to last received periodic locator packet.
	// PreLaunchData is sent at ~1 s intervals; the locator's radio transitions
	// to RX immediately after its TX completes, which is ~0 ms before the
	// receiver sees the packet end (airtime is symmetric).  We open the window
	// kPostPrelaunchMinMs after receipt to let the locator radio fully settle
	// into RX, and close it at kPostPrelaunchMaxMs to keep a safe margin before
	// the next expected locator TX (~1000 ms after receipt, minus one airtime
	// ~80 ms = ~920 ms).  If the arm/disarm command arrives outside this window
	// it waits for the next PreLaunchData cycle rather than risking a collision.
	static constexpr uint32_t kPostPrelaunchMinMs = 50u;
	static constexpr uint32_t kPostPrelaunchMaxMs = 700u;

	// Idle-channel noise floor (ADR-0019).  Sampled only inside the window above,
	// which is precisely the interval in which the locator is known to be
	// listening rather than transmitting — sampling outside it would measure the
	// locator's own carrier and report our own link as interference.
	//
	// We keep the PEAK rather than a mean: interference is bursty, and a mean over
	// a second buries the 50 ms burst that is actually destroying packets.
	//
	// The value published with broadcast N covers the window after broadcast N-1,
	// so it lags by one ~1 s period.  Irrelevant at the timescale a user reacts on.
	static constexpr int16_t  kNoiseFloorUnknown = INT16_MIN;  // no sample this interval
	static constexpr uint32_t kNoiseSampleIntervalMs = 20u;
	// Broadcasts are ~1 s apart; past this the next one is late, so we are already
	// losing packets and sampling resumes outside the safe window.  See the
	// reasoning in ServiceNoiseFloor for why that cannot produce a false positive.
	static constexpr uint32_t kPeriodicOverdueMs = 1500u;
	int16_t  noise_floor_peak_ = kNoiseFloorUnknown;
	uint32_t last_noise_sample_ms_ = 0;
	// Read the accumulated peak and start a fresh interval.
	int16_t TakeNoiseFloor();

	// ── Channel survey (ADR-0019 tier 3, #33) ─────────────────────────────────
	//
	// Sweeps channels 0..63 sampling the idle level on each, so the app can rank
	// them and recommend a quiet one.  Three properties are load-bearing:
	//
	// 1. **Time-sliced, never blocking.** A whole sweep is ~1 s; running it inside
	//    one Service() call would stall BLE servicing and the forwarding windows
	//    for that entire second.  One slice per call instead.
	// 2. **Stays in LoRa RX the whole way.** Only the channel changes.  The obvious
	//    primitive, Radio.IsChannelFree(), switches the modem to FSK and leaves the
	//    radio in standby — restoring only the channel after that would leave the
	//    link dead in a way that looks like a receiver failure (ADR-0019 Decision 6).
	//    Retuning in place means the restore is just channel + Rx re-arm.
	// 3. **Refused while armed.** The sweep is deaf to the locator for its duration.
	//    On the ground that is a blink; in flight it is lost telemetry, and the
	//    channel cannot be changed in flight anyway.
	//
	// The locator is not involved: it never sees either message.
	// Two-phase sweep.  A locator is on air ~138 ms per second, so a short dwell
	// lands in the gap most of the time and reports an occupied channel as quiet —
	// a bench sweep ranked the channel both locators were using as the quietest in
	// the band.  Detecting a 1 Hz emitter reliably needs a dwell longer than its
	// period; 64 of those is a minute, so instead the coarse pass only shortlists
	// and the expensive dwell is spent confirming the few channels we might
	// actually recommend.
	static constexpr uint32_t kSurveySettleMs = 2u;   // let the PLL settle before believing RSSI
	static constexpr uint32_t kSurveyDwellMs  = 12u;  // coarse, per channel, settle included
	// Must exceed the ~1 s broadcast period so at least one transmission is
	// guaranteed to fall inside it, with margin for cadence jitter.
	static constexpr uint32_t kSurveyConfirmDwellMs = 1200u;
	// Throttle RSSI reads.  Sampling every main-loop pass was defensible at a 15 ms
	// dwell (~1500 reads per channel); at 1200 ms it is ~120000 per channel and
	// ~600000 per sweep, each one a SUBGHZ SPI transaction that polls the radio's
	// BUSY line.  1 ms still gives ~1200 samples across a confirm dwell, far more
	// than needed to catch a 138 ms burst.
	static constexpr uint32_t kSurveySampleIntervalMs = 1u;
	// Hard ceiling on a whole sweep.  Nominally ~7 s (0.8 s coarse + 5 x 1.2 s);
	// this is the backstop that guarantees the app always gets an answer, so a
	// sweep that stalls for any reason cannot leave it waiting forever.
	static constexpr uint32_t kSurveyDeadlineMs = 12000u;
	// Mirrors the PHY's RX_TIMEOUT_VALUE (subghz_phy_app.c).  Used only to re-arm
	// RX after a sweep; if that constant changes, this must follow.
	static constexpr uint32_t kRxTimeoutMs = 3000u;

	enum class SurveyPhase : uint8_t { Coarse, Confirm };

	bool     survey_active_ = false;
	bool     survey_response_pending_ = false;
	ChannelSurveyStatus survey_status_ = ChannelSurveyStatus::Ok;
	SurveyPhase survey_phase_ = SurveyPhase::Coarse;
	uint8_t  survey_channel_ = 0;          // channel currently dwelling
	uint32_t survey_channel_start_ms_ = 0;
	uint8_t  survey_home_channel_ = 0;     // restored when the sweep finishes or aborts
	int8_t   survey_level_[kSurveyChannelCount] = { };
	int16_t  survey_channel_peak_ = kNoiseFloorUnknown;
	uint8_t  survey_confirm_channel_[kSurveyConfirmCount] = { };
	uint8_t  survey_confirm_count_ = 0;    // how many made the shortlist
	uint8_t  survey_confirm_index_ = 0;    // which of them is dwelling now
	uint32_t survey_start_ms_ = 0;         // for kSurveyDeadlineMs
	uint32_t survey_last_sample_ms_ = 0;

	void BeginChannelSurvey();
	void BeginSurveyConfirmPhase();  // shortlists the quietest coarse candidates
	void FinishChannelSurvey();      // restores channel + RX and queues the response

	// Deferred receiver channel switch after forwarding a locator channel change.
	// radio_->Send() only *starts* the forward; changing the RF frequency before
	// the transmit completes (OnRadioTxDone) would corrupt the very packet the
	// locator must receive to switch.  So we arm the switch when the forward is
	// sent and apply it from ServicePendingTx() once the forward TX has completed
	// (last_radio_tx_end_ms_ advances past the arm time) and the radio has settled.
	bool     pending_locator_channel_switch_ = false;
	uint8_t  pending_locator_channel_ = 0;
	uint32_t locator_channel_switch_armed_ms_ = 0;

	// Set when the locator is in flight-profile mode — i.e. a FlightMetadata
	// response or a FlightData/FlightDataParity packet has been received.  In
	// that state the locator has gone quiet (listening, not running its periodic
	// PreLaunchData TX), so the PreLaunchData timing gate is unnecessary:
	// app→locator commands forward immediately, and deferred-ACK logic governs
	// FlightDataAck.  Cleared when PreLaunchData resumes (locator back to Disarmed).
	bool     locator_in_profile_mode_ = false;

	// True when the last periodic packet was TelemetryData rather than PreLaunchData,
	// i.e. the locator is armed.  The app gates the survey too, but that gate is
	// app-side and soft (ADR-0006); this one is the enforcement that actually
	// protects flight telemetry from a sweep.
	bool     locator_armed_ = false;
	// Timestamp of the last received FlightData or FlightDataParity packet.
	// Used to detect when the locator's burst has ended (no new packet for
	// kAckDeferMs ms) so the cumulative ACK can be safely forwarded.
	uint32_t last_flight_data_rx_ms_  = 0;
	// How long to wait after the last burst packet before forwarding the ACK.
	// Must exceed one inter-packet gap (data TX ~380 ms + loop delay ~50 ms)
	// so we do not send the ACK while the next burst packet is still arriving.
	static constexpr uint32_t kAckDeferMs = 600u;

	// Single-slot outbound queue: one validated message waiting for a safe
	// TX window.  Overwritten if a second message arrives before the first
	// is sent (acceptable — ACKs are cumulative, requests are idempotent).
	struct PendingTx {
		AppMessage msg {};
		uint8_t    len  = 0;
		bool       ready = false;
	};
	PendingTx pending_tx_;
	uint32_t last_bt_tx_end_ms_ = 0;
	bool bt_tx_led_status_serviced_ = true;
	uint32_t last_bt_rx_end_ms_ = 0;
	bool bt_rx_led_status_serviced_ = true;

	ParseState parse_state_ = ParseState::IDLE;
	uint32_t last_byte_time_ = 0;
	AppMessage current_msg_;
	uint8_t cursor_ = 0;
	uint8_t message_length_;

	uint8_t rx_payload_[kMaxPayloadBytes];
	uint16_t rx_message_size_;
	int16_t rssi_;
	int8_t LoraSnr_FskCfo_;

	const char* lora_startup_message_ = "Rocket Receiver v1.0.1\r\n\0";
	const char* usb_connected_ = "Disconnect USB cable before arming locator\r\n\0";
	const char* bad_gps_data_ = "Bad GPS Data\r\n\0";
	static constexpr char query_version_[] = "AT+VERS\r\n";
	static constexpr char command_mode_[] = "AT+ENAT\r\n";
	static constexpr char data_mode_[] = "AT+EXAT\r\n";
	static constexpr char enable_ble_broadcast_[] = "AT+LEON\r\n";
	static constexpr char disable_spp_broadcast_[] = "AT+SPOF\r\n";
	static constexpr char change_spp_name_[] = "AT+SPNA";
	static constexpr char change_ble_name_[] = "AT+LENA";
	static constexpr char reset_[] = "AT+REST\r\n";
	static constexpr char factory_reset_[] = "AT+RDEF\r\n";
	static constexpr char ble_msb16_[] = "D867"; // Unique ID for Steam Pigeon receivers
	static constexpr char set_ble_address_[] = "AT+LEAD";

	ParseResult ParseLoraFrame(const uint8_t* data,
							 std::size_t   len,
							 uint8_t       expected_system_id,
							 ParsedMessage& out);

	inline uint16_t Crc16Update(uint16_t crc, uint8_t data)
	{
	  crc ^= data;
	  for (int i = 0; i < 8; i++) {
		  if (crc & 1)
			  crc = (crc >> 1) ^ kCrc16Poly;
		  else
			  crc >>= 1;
	  }
	  return crc;
	}

	inline uint16_t Crc16Keyed(const uint8_t* data, size_t len)
	{
	  uint16_t crc = kCrc16Key;   // your secret seed
	  for (size_t i = 0; i < len; ++i)
		  crc = Crc16Update(crc, data[i]);
	  return crc;
	}

	template<typename TMsg>
	inline uint16_t ComputeMessageCrc(const TMsg& msg)
	{
	  static_assert(std::is_trivially_copyable<TMsg>::value, "TMsg must be POD");

	  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&msg);

	  // 1) First 4 bytes of PacketHeader
	  uint16_t crc = kCrc16Key;
	  crc = Crc16Update(crc, bytes[0]);
	  crc = Crc16Update(crc, bytes[1]);
	  crc = Crc16Update(crc, bytes[2]);
	  crc = Crc16Update(crc, bytes[3]);

	  // 2) Skip CRC field (bytes 4–5)
	  // 3) Hash everything after header
	  const size_t payload_offset = sizeof(PacketHeader);

	  for (size_t i = payload_offset; i < sizeof(TMsg); ++i)
		  crc = Crc16Update(crc, bytes[i]);

	  return crc;
	}

	inline uint16_t ComputeMessageCrcPartial(const uint8_t* bytes, size_t msg_size)
	{
	  using FPHeader = PacketHeader;

	  // Where the CRC field lives inside the header
	  constexpr size_t header_crc_offset = offsetof(FPHeader, crc);
	  constexpr size_t header_crc_size   = sizeof(FPHeader) - sizeof(uint16_t);

	  // Start with the keyed seed
	  uint16_t crc = kCrc16Key;

	  // 1) Header bytes BEFORE the CRC field
	  crc = Crc16Keyed(bytes, header_crc_offset);

	  // 2) Header bytes AFTER the CRC field
	  crc = Crc16Keyed(bytes + header_crc_offset + sizeof(uint16_t),
					   header_crc_size - header_crc_offset);

	  // 3) Everything after the header
	  if (msg_size > sizeof(FPHeader)) {
		  const size_t tail_len = msg_size - sizeof(FPHeader);
		  crc = Crc16Keyed(bytes + sizeof(FPHeader), tail_len);
	  }

	  return crc;
	}

	inline bool ValidateCRC(const uint8_t* data, std::size_t len)
	{
	  if (len < sizeof(PacketHeader)) {
		  return false;
	  }

	  const PacketHeader* hdr =
		  reinterpret_cast<const PacketHeader*>(data);

	  constexpr size_t header_size      = sizeof(PacketHeader);
	  constexpr size_t crc_offset       = offsetof(PacketHeader, crc);
	  constexpr size_t bytes_before_crc = crc_offset;            // 0..3

	  uint16_t crc = kCrc16Key;

	  // 1) First 4 bytes of PacketHeader (system_id, msg_type, msg_count LSB/MSB)
	  for (size_t i = 0; i < bytes_before_crc; ++i) {
		  crc = Crc16Update(crc, data[i]);
	  }

	  // 2) Skip CRC field (bytes 4–5)

	  // 3) Header bytes AFTER CRC field
	  for (size_t i = crc_offset + 2; i < header_size; ++i) {
		  crc = Crc16Update(crc, data[i]);
	  }

	  // 4) Everything after the header
	  for (size_t i = header_size; i < len; ++i) {
		  crc = Crc16Update(crc, data[i]);
	  }

	  return crc == hdr->crc;
	}

	template<typename TMsg>
	inline uint16_t ComputeSendMessageCrc(const TMsg& msg)
	{
	  static_assert(std::is_trivially_copyable<TMsg>::value,
					"TMsg must be POD");

	  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&msg);

	  // 1) First 4 bytes of PacketHeader
	  uint16_t crc = kCrc16Key;
	  crc = Crc16Update(crc, bytes[0]);
	  crc = Crc16Update(crc, bytes[1]);
	  crc = Crc16Update(crc, bytes[2]);
	  crc = Crc16Update(crc, bytes[3]);

	  // 2) Skip CRC field (bytes 4–5)
	  // 3) Hash everything after header
	  const size_t payload_offset = sizeof(PacketHeader);

	  for (size_t i = payload_offset; i < sizeof(TMsg); ++i)
		  crc = Crc16Update(crc, bytes[i]);

	  return crc;
	}

    // Set in ISR (OnUART1Char), consumed in main-loop (ServiceBleNameUpdate /
    // ServiceReceiverInfoResponse).  volatile prevents the compiler from caching
    // these flags across the ISR/main-loop boundary.
    volatile bool ble_name_update_pending_      = false;
    volatile bool receiver_info_response_pending_ = false;
    char pending_ble_name_[device_name_buffer_size] = { 0 };

    void Reset() {
        parse_state_ = ParseState::IDLE;
        cursor_ = 0;
        message_length_ = 0;
    }

};
} // namespace Communication
