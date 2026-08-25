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
	FlightDataAck = 12, // Profile data acknowledgment sent from the app via the receiver.
	DeploymentTestRequest = 13, // Request from the app, via the receiver, for the locator to execute a deployment test.
	DeploymentTest = 14,        // Deployment test countdown sent from the locator to the app via the receiver.
	ReceiverInfoRequest = 15,   // Request from the app to the receiver for its current channel and name (no locator needed).
	ReceiverInfo = 16,          // Response from the receiver with its current LoRa channel and device name.
	VersionRequest = 17,        // Request from the app, via the receiver, for both firmware versions.
	VersionInfo = 18,           // Response: locator version forwarded through receiver, which appends its own version.
	FlightEvents = 19,          // Per-record flight event summary sent alongside a FlightData transfer.
	ChannelSurveyRequest = 20,  // Request from the app to the receiver to sweep the band (no locator involved).
	ChannelSurvey = 21,         // Response from the receiver with per-channel occupancy.
	PadAlertSnoozeRequest = 22, // App→locator: suppress the prepped-and-disarmed alert for N minutes (#37).
	LocatorSearchRequest = 23,  // Request from the app to the receiver to listen for locators on named channels (no locator involved).
	LocatorSearchResult = 24    // Streamed response: one per channel searched, plus a terminator.
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
	NoseAxis nose_axis;    // locator mounting config (ADR-0021 Decision 6, #36)
	uint8_t armed;         // explicit arm state, 0/1 (ADR-0021 Decision 3, #35)
	uint8_t pad_alert;     // prepped rocket standing disarmed (ADR-0021 Decision 5, #37)
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
	uint8_t bad_frames;     // frames that arrived and failed to parse since the last report
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
	// Explicit arm state (ADR-0021 Decision 3, #35).  The receiver reads this — it
	// gates the channel survey on arm state — where it previously inferred from
	// the message type.  Passes through to the app untouched.
	uint8_t armed;         // 0 = disarmed, 1 = armed
	uint32_t locator_id;   // cleartext STM MPU UID; passes through untouched to the app
	uint32_t auth_tag;     // password-seeded checksum; receiver never inspects it
};

// Channel survey (ADR-0019 tier 3, #33).  Receiver-directed end to end: the app
// asks the receiver to sweep the band and report per-channel occupancy.  The
// locator is not involved and never sees either message.
inline constexpr uint8_t kSurveyChannelCount = 64;   // channels 0..63
// How many of the quietest coarse candidates get a full-period confirmation dwell.
// Only these are trustworthy enough to recommend; see ChannelSurveyResponse.
inline constexpr uint8_t kSurveyConfirmCount = 5;

enum class ChannelSurveyStatus : uint8_t {
	Ok           = 0,
	RefusedArmed = 1,   // locator is armed — sweeping would drop flight telemetry
	RefusedBusy  = 2,   // flight-data transfer in progress
};

struct ChannelSurveyRequest {
	PacketHeader packet_header;
};

struct ChannelSurveyResponse {
	PacketHeader packet_header;
	uint8_t status;         // ChannelSurveyStatus; levels are meaningless unless Ok
	uint8_t channel_count;  // channels actually measured; 0 when refused
	uint8_t home_channel;   // receiver's own channel, so the app can mark it in the ranking
	// Peak RSSI seen on each channel during its dwell, dBm.  Comparable to each
	// other within one sweep; NOT trustworthy as absolute levels (SX126x RSSI is
	// uncalibrated near the floor), which is why the app ranks rather than reports.
	int8_t  level[kSurveyChannelCount];
	// Channels that got the long confirmation dwell, and are therefore the only
	// ones the app may recommend.
	//
	// A locator transmits ~138 ms once per second — ~14% duty cycle — so a short
	// coarse dwell usually lands in the gap and reads an occupied channel as quiet.
	// (This is not hypothetical: a bench sweep ranked the channel BOTH locators
	// were sitting on as the quietest in the band.)  Catching a 1 Hz emitter
	// reliably needs a dwell longer than its period, which is unaffordable across
	// all 64 channels, so the sweep is two-phase: a fast coarse pass to shortlist,
	// then a full-period dwell on the quietest few.  Coarse levels for everything
	// else are reported for display but are not evidence of a free channel.
	uint8_t confirmed_count;
	uint8_t confirmed_channel[kSurveyConfirmCount];
	// Locator frames DECODED on each confirmed channel during its dwell.
	//
	// This is the only unambiguous answer to "is another locator using this
	// channel", and RSSI cannot give it: a locator within a few feet raises the
	// level on every channel at once (Appendix G), so power says "busy" everywhere
	// and distinguishes nothing.  A frame that decodes on the dwelt channel had to
	// be transmitted ON it — off-channel bleed does not survive the demodulator.
	//
	// Non-zero means occupied, whatever the level says.  Zero does not prove empty:
	// the dwell is one broadcast period, so a sparser emitter can still slip
	// through, and a non-locator device is invisible to this test entirely.
	uint8_t confirmed_frames[kSurveyConfirmCount];
	// The locator_id of the FIRST frame decoded on each confirmed channel, 0 for
	// none.  Cleartext (the locator broadcasts its MPU UID in the clear); the
	// receiver still never inspects auth_tag, so this is identity as CLAIMED, not
	// as authenticated.  Diagnostic only: the app labels an occupied channel with
	// it, and nothing is ever gated on it.
	//
	// Only PreLaunchData and TelemetryData carry an id.  Any other decoded frame
	// still counts in confirmed_frames but leaves this 0 — "occupied by something
	// that would not say who".
	uint32_t confirmed_locator_id[kSurveyConfirmCount];
};

struct TelemetryMessageExtended {
	TelemetryData base;     // original message
	int16_t rssi;           // RSSI seen by receiver (dBm)
	// Link quality / interference (ADR-0019), as in PreLaunchMessageExtended.
	int8_t  snr;            // LoRa SNR of this packet (dB)
	int16_t noise_floor;    // peak idle-channel RSSI since the last report (dBm)
	uint8_t bad_frames;     // frames that arrived and failed to parse since the last report
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

// Same hazard, and it had no guard until the locator's record_count changed
// underneath it (locator #38).  FlightMetadata is sized by record_count, so a
// drift here drops every flight-list response silently — decode_into() compares
// lengths with strict equality and ProcessRadioRx only forwards on Ok.
// 96 = header 6 + 9 records x 10.  The locator asserts the same 96; the app
// asserts the 90-byte payload in WireLayoutTest.
static_assert(sizeof(FlightMetadataRecord) == 10, "FlightMetadataRecord size changed — sync locator + app");
static_assert(sizeof(FlightMetadata) == 96, "FlightMetadata size changed — sync locator record_count + app");

// The two unsolicited broadcasts are length-validated on receive and mirrored
// byte-for-byte into the extended structs forwarded to the app, so a drift from
// the locator's copy silently drops every frame of the type that drifted — the
// failure looks like "the locator went out of range", which is the last thing
// anyone debugs. Pinned here as the third copy of the layout; the locator
// asserts the same numbers and the app's WireLayoutTest asserts the payloads.
static_assert(sizeof(PreLaunchData) == 118, "PreLaunchData size changed — sync locator + app");
static_assert(sizeof(TelemetryData) ==  77, "TelemetryData size changed — sync locator + app");

// The extended structs are what actually reach the app, and the app parses them
// by hand-computed byte offsets — but nothing pinned their size until ADR-0019.
// These are receiver-only (the locator never sees them), so the app's
// WireLayoutTest is the only counterpart: app payload = sizeof(struct) − header(6).
static_assert(sizeof(PreLaunchMessageExtended) == 147,
		"PreLaunchMessageExtended size changed — sync the app's PRELAUNCH_MESSAGE_PAYLOAD_SIZE (141)");
static_assert(sizeof(TelemetryMessageExtended) ==  83,
		"TelemetryMessageExtended size changed — sync the app's TELEMETRY_MESSAGE_PAYLOAD_SIZE (77)");

// Channel survey (ADR-0019 tier 3).  Receiver-only messages; the locator reserves
// the MsgType values but never sends or parses these.
static_assert(sizeof(ChannelSurveyRequest)  ==  6, "ChannelSurveyRequest is header-only");
static_assert(sizeof(ChannelSurveyResponse) == 104,
		"ChannelSurveyResponse size changed — sync the app's CHANNEL_SURVEY_PAYLOAD_SIZE (98)");

// ── Locator search (#33 follow-up) ────────────────────────────────────────────
//
// The survey answers "which channel is quiet".  This answers the opposite
// question — "which channel is my locator ON" — and the two cannot share a sweep:
// the survey's confirm phase dwells on the QUIETEST candidates, so the channel a
// locator is actively using is shortlisted only by accident.
//
// Receiver-directed end to end, like the survey: the locator plays no part and
// never sees either message.  Unlike the survey it streams, one result per
// channel as that channel finishes, because a full-band run is ~77 s and a single
// response at the end would leave the app with a dead progress bar and no way to
// show a hit the moment it happens.
inline constexpr uint8_t kSearchMaxChannels = 16;   // candidate list cap; 0 = whole band
// Stop a run in progress.  A flag on the request rather than a message of its
// own: cancel is meaningless except while a search is running, and the app
// already knows how to send this one.
inline constexpr uint8_t kSearchFlagCancel = 0x01;

enum class LocatorSearchStatus : uint8_t {
	Progress     = 0,   // one channel finished; `found` says whether anything was on it
	Done         = 1,   // run complete — all channels searched, or stopped on the target
	RefusedArmed = 2,   // locator armed or in flight — searching would go deaf over a live flight
	RefusedBusy  = 3,   // a survey or a flight-data transfer already owns the radio
	Cancelled    = 4,   // the app asked it to stop
};

struct LocatorSearchRequest {
	PacketHeader packet_header;
	uint8_t  flags;              // bit0 = cancel a run in progress; all other bits reserved 0
	uint8_t  channel_count;      // channels in `channel`; 0 = sweep the whole band
	// Stop as soon as THIS locator is decoded, rather than searching every listed
	// channel.  0 = report everything found, which is the only useful behaviour when
	// the app has never seen the locator before (a borrowed one has no known id).
	uint32_t target_locator_id;
	uint8_t  channel[kSearchMaxChannels];
};

struct LocatorSearchResult {
	PacketHeader packet_header;
	uint8_t  status;      // LocatorSearchStatus
	uint8_t  channel;     // the channel just searched; 0 on a terminator
	uint8_t  searched;    // 1-based position of this channel in the run
	uint8_t  total;       // channels in the run, so the app can show real progress
	uint8_t  found;       // 1 = a locator frame decoded on `channel`
	uint8_t  armed;       // the locator's own armed byte, 0 when !found
	int16_t  rssi;        // RSSI of the decoded frame (dBm), 0 when !found
	uint32_t locator_id;  // 0 when !found, or when the frame carried no id
	// Carried for the same reason the id is not enough: a borrowed locator is
	// unknown to the app, so an id alone would report it as a bare hex number.  The
	// name is what makes the hit readable.  Cleartext and unauthenticated, like the id.
	char     device_name[device_name_length];
};

static_assert(sizeof(LocatorSearchRequest) == 28,
		"LocatorSearchRequest size changed — sync the app's LOCATOR_SEARCH_REQUEST_PAYLOAD_SIZE (22)");
static_assert(sizeof(LocatorSearchResult) == 38,
		"LocatorSearchResult size changed — sync the app's LOCATOR_SEARCH_RESULT_PAYLOAD_SIZE (32)");

// On-wire packet for flight profile transfer
struct FlightDataPacket {
	PacketHeader packet_header;
	uint16_t transfer_id;   // identifies this flight profile transfer
	uint16_t packet_index;  // 0..packet_count-1 (data) or parity index
	uint16_t packet_count;  // total data packets (excluding parity)
	uint32_t total_samples; // total samples in transfer
	uint8_t payload[kPayloadSize]; // Compressed payload bytes
};

// ── Addressed app→locator commands (ADR-0020, #34) ──────────────────────────
// Mirrored from the locator's copy.  The receiver never inspects the target — it
// only sizes the frame so it can be forwarded intact.  Filtering here would be
// worthless anyway: the hazard is other people's receivers relaying their users'
// commands onto a shared channel, which this receiver cannot see.
struct TargetedRequest {          // ArmRequest, DisarmRequest,
	PacketHeader packet_header;   // FlightMetadataRequest, VersionRequest
	uint32_t target_locator_id;
};

struct FlightDataAck {
	PacketHeader header;
	uint32_t target_locator_id;
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
	uint32_t target_locator_id;   // ADR-0020

	DeployMode deployment_ch1_mode = DeployMode::DroguePrimary;
	DeployMode deployment_ch2_mode = DeployMode::DrogueBackup;
	DeployMode deployment_ch3_mode = DeployMode::MainPrimary;
	DeployMode deployment_ch4_mode = DeployMode::MainBackup;

	// RESERVED.  The app does not set this and the locator keeps its own value;
	// the slot stays so lora_channel below does not move (see the offsetof in
	// Communication::SendPendingTx, ADR-0011).  The receiver only relays these
	// bytes, so nothing here changes for it beyond the name.
	uint16_t launch_detect_altitude;       // meters — reserved, not applied

	uint8_t drogue_primary_deploy_delay;   // tenths of a second
	uint8_t drogue_backup_deploy_delay;    // tenths of a second

	uint16_t main_primary_deploy_altitude; // meters
	uint16_t main_backup_deploy_altitude;  // meters

	uint8_t deploy_signal_duration;        // tenths of a second — reserved, see above
	uint8_t lora_channel;

	char device_name[device_name_length] = { 0 };

	// Which raw sensor axis points at the rocket's nose (ADR-0021 Decision 6,
	// #36).  Relayed through untouched — the receiver never interprets it.
	// Appended AFTER device_name so every existing field keeps its offset, which
	// matters here: ProcessAppMessage reads the relayed lora_channel by
	// offsetof(LocatorRocketSettings, lora_channel) to follow a locator channel
	// change (ADR-0011), and that offset is unchanged by this.
	NoseAxis nose_axis = NoseAxis::Auto;
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
	// Channel status, carried here because this is the ONLY message the receiver
	// sends with no locator involved.
	//
	// ServiceNoiseFloor() already keeps sampling once a broadcast is overdue —
	// that is the whole point of the overdue branch — but until now the reading
	// had nowhere to go: both carriers are locator broadcasts, so the measurement
	// taken during silence could only be delivered by the packet whose absence
	// prompted it.  The app was left extrapolating from the last sample before the
	// locator went quiet, and reported "interference" on a channel that had simply
	// gone still.  Same fields, same units, same drain-on-read semantics as the
	// extended broadcasts above.
	//
	// INT16_MIN spelled out because Communication::kNoiseFloorUnknown is declared
	// in the header that includes THIS one. Same value, and the app pins it too.
	int16_t      noise_floor = INT16_MIN;
	uint8_t      bad_frames = 0;
};

#pragma pack(pop)

// The relayed locator config, length-validated on receive like the broadcasts
// above (message_length_ = sizeof(LocatorRocketSettings) - header).  Nothing
// pinned it until #36, so a drift from the locator's LocatorSettings silently
// dropped every config change the app sent — a failure indistinguishable from
// the command never arriving.  The locator asserts the same 45.
static_assert(sizeof(LocatorRocketSettings) == 45,
		"LocatorRocketSettings size changed — sync locator LocatorSettings + app");

// Receiver-only, but the app frames it by exact length before the CRC is checked,
// so a drift here desynchronises the framer rather than failing a check: the app
// waits for bytes that never arrive, the health probe goes unanswered, and the
// watchdog declares a phantom connection and reconnects forever.  Pinned by the
// app's WireLayoutTest at payload 24 (= 30 - 6 header).
static_assert(sizeof(ReceiverInfoMessage) == 30,
		"ReceiverInfoMessage size changed — sync app RECEIVER_INFO_PAYLOAD_SIZE");

} // namespace Communication
