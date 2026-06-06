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

struct RocketRuntimeMetadata
{
    uint8_t archive_position = 0;
    uint32_t boot_count = 0;
    uint32_t last_flight_sequence = 0;
    uint32_t last_closed_record_id = 0;
};

#pragma pack(pop)

static_assert(std::is_trivially_copyable<RocketPersistentSettings>::value, "RocketPersistentSettings must be trivially copyable.");
static_assert(std::is_standard_layout<RocketPersistentSettings>::value, "RocketPersistentSettings must be standard layout.");

static_assert(std::is_trivially_copyable<RocketRuntimeMetadata>::value, "RocketRuntimeMetadata must be trivially copyable.");
static_assert(std::is_standard_layout<RocketRuntimeMetadata>::value, "RocketRuntimeMetadata must be standard layout.");
