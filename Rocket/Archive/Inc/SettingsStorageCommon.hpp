#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>

namespace SettingsStorage
{
    static constexpr uint32_t CONFIG_ENTRY_MAGIC = 0x43464743u; // CFGC

    class Crc32
    {
    public:
        static uint32_t Compute(const void* data, size_t length);
        static uint32_t Update(uint32_t crc, const void* data, size_t length);
    };

    inline constexpr uint32_t AlignUp(uint32_t value, uint32_t alignment)
    {
        return (value + alignment - 1u) & ~(alignment - 1u);
    }

    template<typename T>
    constexpr bool IsSerializable()
    {
        return std::is_trivially_copyable<T>::value &&
               std::is_standard_layout<T>::value;
    }

#pragma pack(push, 1)

    struct CompactConfigEntryHeader
    {
        uint32_t magic;
        uint32_t sequenceNumber;
        uint32_t payloadCrc32;
    };

#pragma pack(pop)
}
