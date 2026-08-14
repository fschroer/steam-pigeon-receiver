#pragma once
#include <CompactSettingsJournal.hpp>
#include "FlashDriver.hpp"
#include "RocketSettings.hpp"
#include "SystemFlashLayout.hpp"
#include "DeviceUID.hpp"

// MUST equal the locator's record_count.  It is not used for storage here — the
// receiver has no flight archive — it exists solely to size FlightMetadata,
// which carries one FlightMetadataRecord per locator archive slot.
//
// 9 since the locator's ARCHIVE_VERSION 6 (locator #38): FlightSample grew
// 80 -> 88 B and ten records no longer fit its flash region.  Getting this
// wrong does not fail gracefully: decode_into() length-checks with strict
// equality, so a mismatch makes ProcessRadioRx drop every FlightMetadata
// frame as a bad frame and never forward it — the app then shows an empty
// flight list, which reads as a connection problem rather than a version
// mismatch.  The static_assert in MessageProtocol.hpp pins the resulting size.
constexpr uint8_t record_count = 9;

using PersistentSettingsStore = SettingsStorage::CompactSettingsJournal<RocketPersistentSettings>;
using RuntimeMetadataStore    = SettingsStorage::CompactSettingsJournal<RocketRuntimeMetadata>;

class Archive {
public:
	explicit Archive(DeviceUID& deviceUID, IFlashDriver& flash);
	bool Init();
	bool InitializeArchive();
	bool IsInitialized();

	RocketPersistentSettings& GetReceiverSettings() { return receiver_settings_; }
	const RocketPersistentSettings& GetReceiverSettings() const { return receiver_settings_; }
	bool SaveReceiverSettings(RocketPersistentSettings& receiver_settings_);
	// UART console baud rate, kept in the runtime metadata journal rather than the
	// settings journal — see the note on RocketRuntimeMetadata::console_baud.
	// Returns the fallback when nothing valid is stored; SetConsoleBaud rejects
	// anything outside ConsoleBaudRates.
	uint32_t GetConsoleBaud() const;
	bool SetConsoleBaud(uint32_t baud);
private:
	static PersistentSettingsStore::Config MakePersistentStore();
	static RuntimeMetadataStore::Config MakeRuntimeStore();

	DeviceUID& deviceUID_;
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
