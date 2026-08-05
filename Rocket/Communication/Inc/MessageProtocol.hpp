#pragma once
#include <cstddef>
#include <cstdint>
#include "Types.hpp"
#include "RocketSettings.hpp"
#include "Archive.hpp"

namespace Communication {

// Max total bytes for one LoRa packet.  The radio's payload-length register
// is 8 bits, so the hard limit is 255.
constexpr size_t kMaxPayloadBytes = 255;
constexpr uint8_t system_id = 0x44;
constexpr uint16_t kCrc16Poly = 0xA001;   // CRC‑16/IBM reflected polynomial
constexpr uint16_t kCrc16Key = 0xFFFF;   // standard initial value
static constexpr uint32_t MESSAGE_TIMEOUT_MS = 500;

// Message type for the packet header
enum class MsgType : uint8_t {
	Startup = 0, // Initial message at startup
	LocatorCfgChgRequest = 1, // Request to update locator configuration sent from the app via the receiver.
	ReceiverCfgChgRequest = 2, // Request to update receiver configuration sent from the app to the receiver.
	ArmRequest = 3, // Request to arm the locator sent from the app via the receiver.
	DisarmRequest = 4, // Request to disarm the locator sent from the app via the receiver.
	PreLaunchData = 5, // Unsolicited locator status sent from the locator while in an unarmed state.
	TelemetryData = 6, // Unsolicited locator status sent from the locator while in an armed state.
	FlightMetadataRequest = 7, // Request from the app, via the receiver, for high-level information necessary to identify each flight profile record archived by the locator.
	FlightMetadata = 8, // Flight profile metadata response from the locator to the app via the receiver.
	FlightDataRequest = 9, // Request from the app, via the receiver, for the data in one flight profile.
	FlightData = 10, // Flight profile data response from the locator to the app via the receiver consisting of multiple packets, which the app acknowledges via the receiver.
	FlightDataParity = 11, // Parity packet to allow the app to reconstruct profile data if one packet is lost.
	FlightDataAck = 12, // Profile data acknowledgement sent from the app via the receiver.
	DeploymentTestRequest = 13, // Request from the app, via the receiver, for the locator to execute a deployment test.
	DeploymentTest = 14,        // Deployment test countdown sent from the locator to the app via the receiver.
	ReceiverInfoRequest = 15,   // Request from the app to the receiver for its current channel and name (no locator needed).
	ReceiverInfo = 16,          // Response from the receiver with its current LoRa channel and device name.
	VersionRequest = 17,        // Request from the app, via the receiver, for both firmware versions.
	VersionInfo = 18,           // Response: locator version forwarded through receiver, which appends its own version.
	FlightEvents = 19           // Per-record flight event summary sent alongside a FlightData transfer.
};

// Flight event summary indices — wire order of FlightEventsMessage::event_timestamp_ms.
// Mirrors the locator's Communication::FlightEvent; keep the two in step.
enum class FlightEvent : uint8_t {
	Launch = 0,
	Burnout,
	Apogee,
	Noseover,
	DroguePrimaryDeploy,
	DrogueBackupDeploy,
	DrogueVelocityThreshold,
	MainPrimaryDeploy,
	MainBackupDeploy,
	MainVelocityThreshold,
	Landing,
	Count
};
constexpr size_t kFlightEventCount = static_cast<size_t>(FlightEvent::Count);

enum class ParseState {
	IDLE, TYPE, COUNT1, COUNT2, CRC1, CRC2, DATA, VALIDATE
};

#pragma pack(push, 1)

// Common packet header (on-wire)
struct PacketHeader {
	uint8_t system_id; // 1 byte
	MsgType msg_type;  // 1 byte
	uint16_t msg_count; // 2 bytes
	uint16_t crc;       // 2 bytes (CRC-16 with secret seed)
};

// Compute payload size AFTER PacketHeader is complete
constexpr size_t kPayloadSize = kMaxPayloadBytes - sizeof(PacketHeader)   // header
		- 2u                     // transfer_id
		- 2u                     // packet_index
		- 2u                     // packet_count
		- 4u;                    // total_samples

struct StartupMessage {
	PacketHeader packet_header;
	uint32_t serial_number;
	uint8_t version[64];
};

struct VersionInfoMessage {
	PacketHeader packet_header;
	uint8_t locator_version[64];
};

struct VersionInfoExtended {
	VersionInfoMessage base;        // locator version (original VersionInfo from locator)
	uint8_t receiver_version[64];   // receiver appends its own version before forwarding to app
};

struct PreLaunchData {
	PacketHeader packet_header;
	double latitude;
	double longitude;
	double raw_latitude;
	double raw_longitude;
	uint8_t satellites;
	float hacc;
	SensorHealth imu_status;
	SensorHealth baro_status;
	SensorHealth gps_status;
	uint8_t deploy_status;
	float agl;
	Vec3f accel;
	Vec3f gyro;
	DeployMode deploy_ch1_mode;
	DeployMode deploy_ch2_mode;
	DeployMode deploy_ch3_mode;
	DeployMode deploy_ch4_mode;
	uint8_t drogue_primary_deploy_delay;
	uint8_t drogue_backup_deploy_delay;
	uint16_t main_primary_deploy_altitude;
	uint16_t main_backup_deploy_altitude;
	char device_name[device_name_length];
	uint16_t battery_voltage_mvolt;
	uint32_t locator_id;   // cleartext STM MPU UID; passes through untouched to the app
	uint32_t auth_tag;     // password-seeded checksum; receiver never inspects it
};

struct PreLaunchMessageExtended {
	PreLaunchData base;     // original message
	uint8_t receiver_lora_channel;
	uint16_t receiver_battery_level;
	char receiver_name[device_name_length];
	int16_t rssi;           // RSSI seen by receiver (dBm)
	// Link quality / interference (ADR-0019).  Receiver-appended, so both sit
	// outside the authenticated region and auth_tag is unaffected.
	int8_t  snr;            // LoRa SNR of this packet (dB); was measured and discarded before
	int16_t noise_floor;    // peak idle-channel RSSI since the last report (dBm)
};

struct TelemetryData {
	PacketHeader packet_header;
	double latitude;
	double longitude;
	uint8_t satellites;
	float hacc;
	SensorHealth imu_status;
	SensorHealth baro_status;
	SensorHealth gps_status;
	uint8_t deployment_ch1_stats;
	uint8_t deployment_ch2_stats;
	uint8_t deployment_ch3_stats;
	uint8_t deployment_ch4_stats;
	uint8_t physical_deployment_stats;
	float agl;
	Vec3f vel_ned_mps;    // fused NED velocity (north, east, down) m/s
	Quaternionf q_bn;     // body-to-NED attitude quaternion (w, x, y, z)
	FlightStates flight_state;
	uint32_t locator_id;   // cleartext STM MPU UID; passes through untouched to the app
	uint32_t auth_tag;     // password-seeded checksum; receiver never inspects it
};

struct TelemetryMessageExtended {
	TelemetryData base;     // original message
	int16_t rssi;           // RSSI seen by receiver (dBm)
	// Link quality / interference (ADR-0019), as in PreLaunchMessageExtended.
	int8_t  snr;            // LoRa SNR of this packet (dB)
	int16_t noise_floor;    // peak idle-channel RSSI since the last report (dBm)
};

struct FlightMetadataRecord {
	uint32_t timestamp;
	float apogee;
	uint16_t flight_time;
};

struct FlightMetadata {
	PacketHeader packet_header;
	FlightMetadataRecord record[record_count];
};

// Per-record flight event summary.  The receiver only length-validates and
// forwards this; the app decodes it.  Layout must match the locator's
// Communication::FlightEventsMessage exactly.
struct FlightEventsMessage {
	PacketHeader packet_header;
	uint8_t  record;
	uint8_t  reserved;
	uint16_t present_mask;
	uint32_t flight_timestamp_s;
	uint32_t event_timestamp_ms[kFlightEventCount];
	float    max_altitude_m;
	uint8_t  deployment_ch_stats[4];
};
// Length-validated on receive, so a silent layout drift from the locator would
// drop every FlightEvents frame.  Pin it (the locator asserts the same 66).
static_assert(sizeof(FlightEventsMessage) == 66, "FlightEventsMessage size changed — sync locator + app");

// The two unsolicited broadcasts are length-validated on receive and mirrored
// byte-for-byte into the extended structs forwarded to the app, so a drift from
// the locator's copy silently drops every frame of the type that drifted — the
// failure looks like "the locator went out of range", which is the last thing
// anyone debugs. Pinned here as the third copy of the layout; the locator
// asserts the same numbers and the app's WireLayoutTest asserts the payloads.
static_assert(sizeof(PreLaunchData) == 115, "PreLaunchData size changed — sync locator + app");
static_assert(sizeof(TelemetryData) ==  76, "TelemetryData size changed — sync locator + app");

// The extended structs are what actually reach the app, and the app parses them
// by hand-computed byte offsets — but nothing pinned their size until ADR-0019.
// These are receiver-only (the locator never sees them), so the app's
// WireLayoutTest is the only counterpart: app payload = sizeof(struct) − header(6).
static_assert(sizeof(PreLaunchMessageExtended) == 143,
		"PreLaunchMessageExtended size changed — sync the app's PRELAUNCH_MESSAGE_PAYLOAD_SIZE (137)");
static_assert(sizeof(TelemetryMessageExtended) ==  81,
		"TelemetryMessageExtended size changed — sync the app's TELEMETRY_MESSAGE_PAYLOAD_SIZE (75)");

// On-wire packet for flight profile transfer
struct FlightDataPacket {
	PacketHeader packet_header;
	uint16_t transfer_id;   // identifies this flight profile transfer
	uint16_t packet_index;  // 0..packet_count-1 (data) or parity index
	uint16_t packet_count;  // total data packets (excluding parity)
	uint32_t total_samples; // total samples in transfer
	uint8_t payload[kPayloadSize]; // Compressed payload bytes
};

struct FlightDataAck {
	PacketHeader header;
	uint16_t transfer_id;
	uint16_t packet_count;
	static constexpr uint16_t kMaxPayloadBytes = 256;
	uint8_t bitmap[kMaxPayloadBytes / 8];
};

struct DeploymentTestCountdownMessage {
	PacketHeader packet_header;
	uint8_t count;
};

struct ParsedMessage {
    MsgType type;

    union {
    	StartupMessage startup;
    	PreLaunchData prelaunch;
    	TelemetryData telemetry;
    	FlightMetadata flight_metadata;
    	FlightEventsMessage flight_events;
    	FlightDataPacket flight_data_packet;
    	DeploymentTestCountdownMessage deployment_test;
    	VersionInfoMessage version_info;
//        PacketHeader packet_header;
    };
};

struct AppMessage {
	PacketHeader header;
	uint8_t payload[64];
};

struct LocatorRocketSettings {
	PacketHeader header;

	DeployMode deployment_ch1_mode = DeployMode::DroguePrimary;
	DeployMode deployment_ch2_mode = DeployMode::DrogueBackup;
	DeployMode deployment_ch3_mode = DeployMode::MainPrimary;
	DeployMode deployment_ch4_mode = DeployMode::MainBackup;

	uint16_t launch_detect_altitude;       // meters

	uint8_t drogue_primary_deploy_delay;   // tenths of a second
	uint8_t drogue_backup_deploy_delay;    // tenths of a second

	uint16_t main_primary_deploy_altitude; // meters
	uint16_t main_backup_deploy_altitude;  // meters

	uint8_t deploy_signal_duration;        // tenths of a second
	uint8_t lora_channel;

	char device_name[device_name_length] = { 0 };
};

struct ReceiverSettings {
	PacketHeader header;
	uint8_t lora_channel = 0;
	char device_name[device_name_length] = { 0 };
};

// Receiver-only status message sent in response to ReceiverInfoRequest.
// Never forwarded to the locator.
struct ReceiverInfoMessage {
	PacketHeader header;
	uint8_t      lora_channel = 0;
	char         device_name[device_name_length] = { 0 };
};

#pragma pack(pop)

} // namespace Communication
