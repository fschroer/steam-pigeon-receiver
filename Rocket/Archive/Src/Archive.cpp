#include <CompactSettingsJournal.hpp>
#include "Archive.hpp"

constexpr SystemFlashLayout layout =
        MakeSettingsFlashLayout(
            512u * 1024u, // total flash
            32u * 1024u,        // persistent settings region
            32u * 1024u         // runtime metadata region
        );
PersistentSettingsStore::Config Archive::MakePersistentStore()
{
	PersistentSettingsStore::Config persistent_cfg{};
	persistent_cfg.regionBaseAddress = layout.settingsBaseAddress;
	persistent_cfg.regionSizeBytes   = layout.settingsSizeBytes;
	return persistent_cfg;
}

RuntimeMetadataStore::Config Archive::MakeRuntimeStore()
{
	RuntimeMetadataStore::Config runtime_cfg{};
	runtime_cfg.regionBaseAddress = layout.runtimeMetadataBaseAddress;
	runtime_cfg.regionSizeBytes = layout.runtimeMetadataSizeBytes;
	return runtime_cfg;
}

Archive::Archive(IFlashDriver& flash)
	: flash_(flash),
		persistentStore_(flash_, MakePersistentStore()),
		runtimeStore_(flash_, MakeRuntimeStore()) {}

bool Archive::Init() {
	if (!persistentStore_.Init()) {	return false; }
	if (!runtimeStore_.Init()) { return false; }
	RocketPersistentSettings defaultSettings{};
	std::strncpy(defaultSettings.device_name, "Rocket01", device_name_length);
	defaultSettings.device_name[device_name_length] = '\0';
	RocketPersistentSettings settings{};
	if (!persistentStore_.LoadOrDefault(settings, defaultSettings)) {
		return false;
	}
	RocketRuntimeMetadata runtimeDefaults{};
	RocketRuntimeMetadata runtime{};
	if (!runtimeStore_.LoadOrDefault(runtime, runtimeDefaults)) {
		return false;
	}
	runtime.boot_count += 1u;
	bool runtimeSaved = false;
	if (!runtimeStore_.SaveIfChanged(runtime, runtimeSaved)) {
		return false;
	}
	return true;
}

bool Archive::SaveLocatorSettings(RocketPersistentSettings& locator_settings) {
	locator_settings_ = locator_settings;
	bool settings_saved = false;
	return persistentStore_.SaveIfChanged(locator_settings, settings_saved);
}

