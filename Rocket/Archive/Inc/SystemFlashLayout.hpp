#pragma once

#include <cstdint>

struct SystemFlashLayout
{
    uint32_t totalFlashBytes;

    uint32_t settingsBaseAddress;
    uint32_t settingsSizeBytes;

    uint32_t runtimeMetadataBaseAddress;
    uint32_t runtimeMetadataSizeBytes;
};

inline constexpr SystemFlashLayout MakeSettingsFlashLayout(uint32_t totalFlashBytes,
                                                           uint32_t settingsRegionBytes,
                                                           uint32_t runtimeMetadataRegionBytes)
{
    SystemFlashLayout layout{};
    layout.totalFlashBytes = totalFlashBytes;

    layout.settingsBaseAddress = totalFlashBytes - (settingsRegionBytes + runtimeMetadataRegionBytes);
    layout.settingsSizeBytes = settingsRegionBytes;

    layout.runtimeMetadataBaseAddress = layout.settingsBaseAddress + settingsRegionBytes;
    layout.runtimeMetadataSizeBytes = runtimeMetadataRegionBytes;

    return layout;
}
