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

template<> struct MsgTraits<MsgType::FlightData> {
    using type = FlightDataPacket;
    static constexpr auto field = &ParsedMessage::flight_data_packet;
};

template<> struct MsgTraits<MsgType::FlightDataParity> {
    using type = FlightDataPacket;
    static constexpr auto field = &ParsedMessage::flight_data_packet;
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
	uint32_t last_radio_rx_end_ms_ = 0;
	bool radio_rx_led_status_serviced_ = true;
	// Timestamp of the last received PreLaunchData packet.  Used by
	// ServicePendingTx() to avoid forwarding app messages while the locator
	// is likely mid-TX on its next periodic PreLaunchData transmission.
	uint32_t last_prelaunch_rx_ms_    = 0;
	bool     prelaunch_ever_received_ = false;

	// Danger window around the expected next PreLaunchData TX.
	// PreLaunchData is sent at ~1 s intervals; we hold outbound LoRa TX
	// in [750 ms, 1250 ms) to avoid collision, then release.
	static constexpr uint32_t kPrelaunchDangerStartMs = 750u;
	static constexpr uint32_t kPrelaunchDangerEndMs   = 1250u;

	// Set when the first FlightData packet is received from the locator.
	// At that point the locator is in DataRequested state and will never send
	// PreLaunchData again this session, so the timing gate is unnecessary.
	bool in_flight_data_mode_ = false;

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

    void Reset() {
        parse_state_ = ParseState::IDLE;
        cursor_ = 0;
        message_length_ = 0;
    }

};
} // namespace Communication
