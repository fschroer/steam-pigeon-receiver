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
	std::strncpy(default_settings_.device_name, "Rocket01", device_name_length);
	if (!persistentStore_.LoadOrDefault(receiver_settings_, default_settings_)) {
		return false;
	}
	if (!runtimeStore_.LoadOrDefault(runtime_, runtime_defaults_)) {
		return false;
	}
	runtime_.boot_count += 1u;
	if (!runtimeStore_.SaveIfChanged(runtime_, runtime_saved_)) {
		return false;
	}
	return true;
}

bool Archive::SaveReceiverSettings(RocketPersistentSettings& receiver_settings) {
	receiver_settings_ = receiver_settings;
	return persistentStore_.SaveIfChanged(receiver_settings, settings_saved_);
}

