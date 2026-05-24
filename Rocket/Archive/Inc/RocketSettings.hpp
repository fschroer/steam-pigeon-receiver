#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <cstring>

constexpr std::size_t device_name_length = 12;

#pragma pack(push, 1)

struct RocketPersistentSettings
{
    uint8_t lora_channel = 0;
    char device_name[device_name_length] = {0};
    uint32_t ble_address_lsb32 = 0;
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
