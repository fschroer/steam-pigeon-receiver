#pragma once

extern "C" {
#include "usart.h"
}

#include <type_traits>
#include "PowerManagement.hpp"
#include "MessageProtocol.hpp"

namespace Communication {

enum class ParseResult {
    Ok,
    TooShort,
    SystemIdMismatch,
    CrcMismatch,
    LengthMismatch,
    UnknownType
};

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
  Communication(Archive& archive, PowerManagement& power, UART_HandleTypeDef& huart1);
  void Init(IRadio& radio);
  void SetChannel(uint8_t channel);
  void OnRadioTxDone();   // called from ISR/callback
  void OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);   // ACK reception handler
  void ForwardToBluetooth(const uint8_t* buf, std::size_t len);
  void UpdateStatusLeds();

private:
  Archive& archive_;
  PowerManagement& power_;
  UART_HandleTypeDef& huart1_;
  IRadio* radio_ = nullptr;

  bool radio_busy_ = false;
  uint32_t last_tx_end_ms_ = 0;
  bool tx_led_status_serviced_ = true;
  uint32_t last_rx_end_ms_ = 0;
  bool rx_led_status_serviced_ = true;

  const char* lora_startup_message_ = "Rocket Receiver v1.0.1\r\n\0";
  const char* usb_connected_ = "Disconnect USB cable before arming locator\r\n\0";
  const char* bad_gps_data_ = "Bad GPS Data\r\n\0";

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
};
} // namespace Communication
