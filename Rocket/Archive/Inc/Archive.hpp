#pragma once
#include <CompactSettingsJournal.hpp>
#include "FlashDriver.hpp"
#include "RocketSettings.hpp"
#include "SystemFlashLayout.hpp"

constexpr uint8_t record_count = 10;

using PersistentSettingsStore = SettingsStorage::CompactSettingsJournal<RocketPersistentSettings>;
using RuntimeMetadataStore    = SettingsStorage::CompactSettingsJournal<RocketRuntimeMetadata>;

class Archive{
public:
	explicit Archive(IFlashDriver& flash);
	bool Init();
	bool InitializeArchive();
	bool IsInitialized();

	RocketPersistentSettings& GetLocatorSettings() { return locator_settings_; }
	const RocketPersistentSettings& GetLocatorSettings() const { return locator_settings_; }
	bool SaveLocatorSettings(RocketPersistentSettings& locator_settings);
private:
	static PersistentSettingsStore::Config MakePersistentStore();
	static RuntimeMetadataStore::Config MakeRuntimeStore();

	IFlashDriver& flash_;
	PersistentSettingsStore persistentStore_;
	RuntimeMetadataStore runtimeStore_;
	RocketPersistentSettings locator_settings_ {};
};
