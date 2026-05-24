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

	RocketPersistentSettings& GetReceiverSettings() { return receiver_settings_; }
	const RocketPersistentSettings& GetReceiverSettings() const { return receiver_settings_; }
	bool SaveReceiverSettings(RocketPersistentSettings& receiver_settings_);
private:
	static PersistentSettingsStore::Config MakePersistentStore();
	static RuntimeMetadataStore::Config MakeRuntimeStore();

	IFlashDriver& flash_;
	PersistentSettingsStore persistentStore_;
	RuntimeMetadataStore runtimeStore_;
	RocketPersistentSettings default_settings_ { };
	RocketPersistentSettings receiver_settings_ {};
	RocketRuntimeMetadata runtime_defaults_ { };
	RocketRuntimeMetadata runtime_ { };
	bool runtime_saved_ = false;
	bool settings_saved_ = false;
};
