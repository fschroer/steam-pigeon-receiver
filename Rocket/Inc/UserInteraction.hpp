#pragma once

extern "C" {
#include "time.h"
#include "math.h"
#include "usart.h"
}

#include <string>
#include "Communication.hpp"
#include "Archive.hpp"
#include "ConsoleBaud.hpp"

#define UART_LINE_MAX_LENGTH 255
#define USER_INPUT_MAX_LENGTH 15
#define DATE_STRING_LENGTH 23
#define ALTIMETER_STRING_LENGTH 7
#define ACCELEROMETER_STRING_LENGTH 9

constexpr uint16_t uart_timeout = 5000;

enum UserInteractionState {
	WaitingForCommand = 0, ConfigHome, EditLoraChannel, EditDeviceName, EditConsoleBaud, DfuHome
};

class UserInteraction {
public:
	UserInteraction(Communication::Communication &comm, Archive &archive, UART_HandleTypeDef &huart2,
			ConsoleBaud &console_baud);
	void ProcessChar(uint8_t uart_char, DeviceState &device_state);
	void SetUserInteractionState(UserInteractionState user_interaction_state);
private:
	Communication::Communication &comm_;
	Archive &archive_;
	UART_HandleTypeDef &huart2_;
	ConsoleBaud &console_baud_;

	UserInteractionState user_interaction_state_ = WaitingForCommand;
	char *uart_line_ = new char[UART_LINE_MAX_LENGTH + 1];
	char *user_input_ = new char[device_name_buffer_size];
	// ESC ( B  designates US-ASCII as G0; SI (0x0F) invokes G0.  Both are needed
	// because a terminal can be put into DEC Special Graphics either by ESC ( 0 or
	// by SO (0x0E) with G1 already designated as graphics.
	//
	// This is not decoration.  Garbage emitted while the console and the terminal
	// are at different baud rates will, sooner or later, contain one of those
	// sequences by chance — and once it does, the terminal renders every LOWERCASE
	// letter as a line-drawing glyph while digits and capitals come through
	// untouched (the graphics set only remaps 0x5F-0x7E).  The result reads exactly
	// like a baud mismatch, survives power-cycling the device, and is immune to
	// changing baud rate on either end, because the broken state lives in the
	// terminal.  Diagnosed on the bench 2026-08-12 after a copy/paste of the
	// "garbled" config screen turned out to be byte-perfect.
	const char *clear_screen_ = "\x1b[2J" "\x1b(B" "\x0f" "\r\0";
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
	const char *console_baud_text_ = "b) Console Baud:\t\t\t\0";
	// Says how to get back in, on the one screen an operator is looking at when
	// they are about to make the console unreadable.  Stated here and not only in
	// the manual, because the manual is not what you have in front of you when the
	// terminal has just filled with garbage.
	const char *console_baud_edit_text_ = "Edit Console Baud:\r\n"
			"Takes effect immediately - change your terminal to match.\r\n"
			"If the console goes unreadable, nothing is lost. Either:\r\n"
			" - set your terminal to the rate you want and hold Shift+U, or\r\n"
			" - step your terminal through the rates until this menu returns.\r\n\0";
	//const char* device_name_edit_text_ = "Edit Device Name:\r\n\0";

	const char *dfu_intro_ = "Device Firmware Upgrade\r\n\r\n\0";
	const char *dfu_guidance_text_ = "Enter to continue, Esc to cancel\r\n\r\n\0";
	const char *dfu_warning_text_ = "Warning - device will stop working until reset by administrator\r\n\0";

	uint32_t lora_channel_;
	char device_name_[device_name_buffer_size]; // local copy; [device_name_length] is always 0
	// Index into ConsoleBaudRates::kStandardRates rather than the rate itself, so
	// the [ / ] edit keys step between legal rates instead of over a numeric range
	// that is mostly illegal values.
	int console_baud_index_ = 0;

	void DisplayConfigSettingsMenu();
	void AdjustConfigNumericSetting(uint8_t uart_char, uint32_t *config_mode_setting, uint32_t max_setting_value, bool tenths);
	void AdjustConfigTextSetting(uint8_t uart_char, char *config_mode_setting);
	void AdjustConsoleBaudSetting(uint8_t uart_char);
	// Position of `rate` in ConsoleBaudRates::kStandardRates, or the index of the
	// fallback rate when it is not in the table.
	static int ConsoleBaudIndexOf(uint32_t rate);
	void DisplayDfuMenu();
	uint8_t StartBootloader();
};
