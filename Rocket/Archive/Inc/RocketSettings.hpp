#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <cstring>

// On-wire / struct field size: holds exactly device_name_length printable characters.
// A null terminator is NOT required in the protocol field; shorter names are zero-padded.
constexpr std::size_t device_name_length = 20;

// Local buffer size for C-string operations: one extra byte for a null terminator.
// Use device_name_buffer_size for stack / member buffers; use device_name_length for
// struct fields and wire-format copies.
constexpr std::size_t device_name_buffer_size = device_name_length + 1;

#pragma pack(push, 1)

struct RocketPersistentSettings
{
    uint8_t lora_channel = 0;
    char device_name[device_name_length] = {0};
};

// Layout-sensitive: this is the payload of the RuntimeMetadataStore journal,
// whose entry header carries a magic and a CRC but NO version or size field.
// Any change to the layout therefore re-defaults the journal once on the next
// flash — here that costs only boot_count, the sole field anything reads.
struct RocketRuntimeMetadata
{
    // NOTE: an `archive_position` field led this struct until 2026-08-07.  It
    // was never read AND never written on the receiver — a straight copy of the
    // locator's, where it held (last_closed_record_id + 1), i.e. the NEXT slot
    // to write rather than the record just written.  Removed on both sides
    // together: beside last_closed_record_id, the name invited an off-by-one.
    uint32_t boot_count = 0;
    // Unused on the receiver so far; kept because the layout cost of removing
    // them is the same re-default as above and they mirror the locator's.
    uint32_t last_flight_sequence = 0;
    uint32_t last_closed_record_id = 0;
};

#pragma pack(pop)

static_assert(std::is_trivially_copyable<RocketPersistentSettings>::value, "RocketPersistentSettings must be trivially copyable.");
static_assert(std::is_standard_layout<RocketPersistentSettings>::value, "RocketPersistentSettings must be standard layout.");

static_assert(std::is_trivially_copyable<RocketRuntimeMetadata>::value, "RocketRuntimeMetadata must be trivially copyable.");
static_assert(std::is_standard_layout<RocketRuntimeMetadata>::value, "RocketRuntimeMetadata must be standard layout.");
