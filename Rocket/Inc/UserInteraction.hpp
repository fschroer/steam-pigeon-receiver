#pragma once

extern "C" {
#include "time.h"
#include "math.h"
#include "usart.h"
}

#include <string>
#include "Communication.hpp"
#include "Archive.hpp"

#define UART_LINE_MAX_LENGTH 255
#define USER_INPUT_MAX_LENGTH 15
#define DATE_STRING_LENGTH 23
#define ALTIMETER_STRING_LENGTH 7
#define ACCELEROMETER_STRING_LENGTH 9

constexpr uint16_t uart_timeout = 5000;

enum UserInteractionState {
	WaitingForCommand = 0, ConfigHome, EditLoraChannel, EditDeviceName, DfuHome
};

class UserInteraction {
public:
	UserInteraction(Communication::Communication &comm, Archive &archive, UART_HandleTypeDef &huart2);
	void ProcessChar(uint8_t uart_char, DeviceState &device_state);
	void SetUserInteractionState(UserInteractionState user_interaction_state);
private:
	Communication::Communication &comm_;
	Archive &archive_;
	UART_HandleTypeDef &huart2_;

	UserInteractionState user_interaction_state_ = WaitingForCommand;
	char *uart_line_ = new char[UART_LINE_MAX_LENGTH + 1];
	char *user_input_ = new char[device_name_buffer_size];
	const char *clear_screen_ = "\x1b[2J\r\0";
	const char *config_command_ = "config\0";
	const char *dfu_command_ = "dfu\0";
	const char *crlf_ = "\r\n\0";
	const char *cr_ = "\r\0";
	const char *bs_ = "\b \b\0";
	const char *config_menu_intro_ = "Rocket Receiver Configuration\r\n\0";
	const char *config_save_text_ = "Saved Configuration\r\n\r\n\0";
	const char *cancel_text_ = "Canceled\r\n\r\n\0";
	const char *lora_channel_text_ = "0) Lora Channel (0-63):\t\t\t\0";
	const char *device_name_text_ = "1) Device Name:\t\t\t\t\0";
	const char *num_edit_guidance_text_ = "[ = down, ] = up. Hit Enter to update, Esc to cancel.\r\n\0";
	const char *text_edit_guidance_text_ = "Type text. Hit Enter to update, Esc to cancel.\r\n\0";
	const char *lora_channel_edit_text_ = "Edit Lora Channel (0-63):\r\n\0";
	//const char* device_name_edit_text_ = "Edit Device Name:\r\n\0";

	const char *dfu_intro_ = "Device Firmware Upgrade\r\n\r\n\0";
	const char *dfu_guidance_text_ = "Enter to continue, Esc to cancel\r\n\r\n\0";
	const char *dfu_warning_text_ = "Warning - device will stop working until reset by administrator\r\n\0";

	uint32_t lora_channel_;
	char device_name_[device_name_buffer_size]; // local copy; [device_name_length] is always 0

	void DisplayConfigSettingsMenu();
	void AdjustConfigNumericSetting(uint8_t uart_char, uint32_t *config_mode_setting, uint32_t max_setting_value, bool tenths);
	void AdjustConfigTextSetting(uint8_t uart_char, char *config_mode_setting);
	void DisplayDfuMenu();
	uint8_t StartBootloader();
};
