extern "C" {
#include "usart.h"
}

#include <UserInteraction.hpp>
//#include "CubeMonitorGlobals.hpp"
//#include "Constants.hpp"
#include "StaticString.hpp"
#include "StaticStringWriter.hpp"
#include "Format.hpp"

constexpr uint8_t century = 100;
constexpr uint16_t max_drogue_primary_deploy_delay = 20;
constexpr uint16_t max_drogue_backup_deploy_delay = 40;
constexpr uint16_t max_main_primary_deploy_altitude = 400;
constexpr uint16_t max_main_backup_deploy_altitude = 400;
constexpr uint16_t max_lora_channel = 63;

UserInteraction::UserInteraction(Communication::Communication &comm, Archive &archive,
		UART_HandleTypeDef &huart2, ConsoleBaud &console_baud) :
		comm_(comm), archive_(archive), huart2_(huart2), console_baud_(console_baud) {
}

int UserInteraction::ConsoleBaudIndexOf(uint32_t rate) {
	for (std::size_t i = 0; i < ConsoleBaudRates::kStandardRateCount; i++)
		if (ConsoleBaudRates::kStandardRates[i] == rate)
			return static_cast<int>(i);
	return static_cast<int>(ConsoleBaudRates::kStandardRateCount) - 1;  // kFallbackRate is the last entry
}

void UserInteraction::ProcessChar(uint8_t uart_char, DeviceState &device_state) {
	StaticStringWriter<UART_LINE_MAX_LENGTH> export_line(&huart2_);
	RocketPersistentSettings &receiver_settings = archive_.GetReceiverSettings();
	static int char_pos = 0;
	switch (user_interaction_state_) {
	case UserInteractionState::WaitingForCommand:
		if (((uart_char >= 'A' && uart_char <= 'Z') || (uart_char >= 'a' && uart_char <= 'z'))
				&& char_pos < USER_INPUT_MAX_LENGTH) {
			HAL_UART_Transmit(&huart2_, &uart_char, 1, uart_timeout);
			user_input_[char_pos++] = uart_char;
		} else
			switch (uart_char) {
			case 13: // Enter key
				if (std::memcmp(user_input_, config_command_, char_pos) == 0) {
					device_state = DeviceState::Config;
					user_interaction_state_ = UserInteractionState::ConfigHome;
					lora_channel_ = receiver_settings.lora_channel;
					std::memcpy(device_name_, receiver_settings.device_name, device_name_length);
					device_name_[device_name_length] = 0; // null-terminate local buffer
					// Seeded from the live rate, not from flash: after a sync-byte
					// recovery those differ until the detection is saved, and the
					// menu must show what the operator is actually connected at.
					console_baud_index_ = ConsoleBaudIndexOf(console_baud_.CurrentRate());
					DisplayConfigSettingsMenu();
				} else if (std::memcmp(user_input_, dfu_command_, char_pos) == 0) {
					device_state = DeviceState::Config;
					user_interaction_state_ = UserInteractionState::DfuHome;
					DisplayDfuMenu();
				}
				char_pos = 0;
				user_input_[0] = 0;
				break;
			case 8: // Backspace
				HAL_UART_Transmit(&huart2_, &uart_char, 1, uart_timeout);
				user_input_[--char_pos] = 0;
				break;
			}
		break;
	case UserInteractionState::ConfigHome:
		switch (uart_char) {
		case 13: // Enter key
			receiver_settings.lora_channel = lora_channel_;
			std::memcpy(receiver_settings.device_name, device_name_, device_name_length);
			archive_.SaveReceiverSettings(receiver_settings);
			comm_.SetChannel(lora_channel_);
			comm_.QueueBleNameUpdate(device_name_);
			device_state = DeviceState::Receive;
			user_interaction_state_ = UserInteractionState::WaitingForCommand;
			export_line.WriteMany(config_save_text_);
			break;
		case 27: // Esc key
			device_state = DeviceState::Receive;
			user_interaction_state_ = UserInteractionState::WaitingForCommand;
			export_line.WriteMany(cancel_text_);
			break;
		case 48: // 0 = Edit LoRa channel
			user_interaction_state_ = UserInteractionState::EditLoraChannel;
			export_line.WriteMany(lora_channel_edit_text_, num_edit_guidance_text_, lora_channel_);
			break;
		case 49: // 1 = Edit device name
			user_interaction_state_ = UserInteractionState::EditDeviceName;
			export_line.WriteMany(text_edit_guidance_text_);
			break;
		case 'b': // b = Edit console baud (stored locally; never sent over the air)
		case 'B':
			user_interaction_state_ = UserInteractionState::EditConsoleBaud;
			// Two writes, not one.  The help text alone is 267 characters against a
			// 255-byte line buffer, and AppendMany discards the overflow SILENTLY —
			// which truncated this screen mid-word at "...until this me".  Each
			// WriteMany clears and flushes, so splitting bounds each by itself.
			export_line.WriteMany(console_baud_edit_text_);
			// Current rate after the guidance, so the [ / ] display has somewhere to
			// land and the operator can see what they are changing from.
			export_line.WriteMany(num_edit_guidance_text_, ConsoleBaudRates::kStandardRates[console_baud_index_]);
			break;
		}
		break;
	case UserInteractionState::EditLoraChannel:
		AdjustConfigNumericSetting(uart_char, &lora_channel_, max_lora_channel, false);
		break;
	case UserInteractionState::EditDeviceName:
		AdjustConfigTextSetting(uart_char, device_name_);
		break;
	case UserInteractionState::EditConsoleBaud:
		AdjustConsoleBaudSetting(uart_char);
		break;
	case UserInteractionState::DfuHome:
		if (uart_char == 13) // Enter key
			StartBootloader();
		else if (uart_char == 27) { // Esc key
			device_state = DeviceState::Receive;
			user_interaction_state_ = UserInteractionState::WaitingForCommand;
			export_line.WriteMany(cancel_text_);
		}
		break;
	}
}

void UserInteraction::DisplayConfigSettingsMenu() {
	StaticStringWriter<UART_LINE_MAX_LENGTH> export_line(&huart2_);
	export_line.WriteMany(clear_screen_, config_menu_intro_, crlf_);
	export_line.WriteMany(lora_channel_text_, lora_channel_, crlf_);
	export_line.WriteMany(device_name_text_, device_name_, crlf_);
	export_line.WriteMany(console_baud_text_, ConsoleBaudRates::kStandardRates[console_baud_index_]);
	export_line.WriteMany(crlf_, crlf_);
}

void UserInteraction::AdjustConsoleBaudSetting(uint8_t uart_char) {
	StaticStringWriter<UART_LINE_MAX_LENGTH> export_line(&huart2_);
	switch (uart_char) {
	case 13: { // Enter key — apply and persist
		const uint32_t rate = ConsoleBaudRates::kStandardRates[console_baud_index_];
		// Persist BEFORE switching the line.  If the operator has the rate wrong
		// they are about to stop being able to read anything, and a rate that was
		// applied but never stored would come back as the old one after a reset —
		// which sounds like a mercy, but it would also silently undo a sync-byte
		// recovery the moment the device was power-cycled.
		const bool saved = archive_.SetConsoleBaud(rate);
		// Emitted at the OLD rate, while the operator can still read it.
		if (saved)
			export_line.WriteMany(crlf_, "Console baud set to ", rate, " - switch your terminal now.", crlf_);
		else
			export_line.WriteMany(crlf_, "Console baud NOT saved - rate unchanged.", crlf_);
		if (saved)
			console_baud_.SetRate(rate);
		user_interaction_state_ = UserInteractionState::ConfigHome;
		DisplayConfigSettingsMenu();
		break;
	}
	case 27: // Esc key
		console_baud_index_ = ConsoleBaudIndexOf(console_baud_.CurrentRate());
		user_interaction_state_ = UserInteractionState::ConfigHome;
		DisplayConfigSettingsMenu();
		break;
	case 91: // [ = next rate down
		if (console_baud_index_ > 0)
			console_baud_index_--;
		break;
	case 93: // ] = next rate up
		if (console_baud_index_ < static_cast<int>(ConsoleBaudRates::kStandardRateCount) - 1)
			console_baud_index_++;
		break;
	}
	if (uart_char == 91 || uart_char == 93) {
		// Trailing blanks cover the digits of a wider rate when stepping down, so
		// 921600 -> 9600 does not read as "9600 0".
		export_line.WriteMany(cr_, ConsoleBaudRates::kStandardRates[console_baud_index_], "   ");
	}
}

void UserInteraction::AdjustConfigNumericSetting(uint8_t uart_char, uint32_t *config_mode_setting, uint32_t max_setting_value,
		bool tenths) {
	StaticStringWriter<UART_LINE_MAX_LENGTH> export_line(&huart2_);
	switch (uart_char) {
	case 13: // Enter key
		user_interaction_state_ = UserInteractionState::ConfigHome;
		DisplayConfigSettingsMenu();
		break;
	case 27: // Esc key
		user_interaction_state_ = UserInteractionState::ConfigHome;
		DisplayConfigSettingsMenu();
		break;
	case 91: // [ = decrease value
		if (*config_mode_setting > 0)
			(*config_mode_setting)--;
		break;
	case 93: // ] = increase value
		if (*config_mode_setting < max_setting_value)
			(*config_mode_setting)++;
		break;
	}
	if (uart_char == 91 || uart_char == 93) {
		export_line.WriteMany(cr_, *config_mode_setting);
	}
}

void UserInteraction::AdjustConfigTextSetting(uint8_t uart_char, char *config_mode_setting) {
	static uint16_t char_pos = 0;
	if (uart_char == 13 || uart_char == 27) {
		if (uart_char == 13) {
			uint16_t i = 0;
			for (; i < char_pos; i++)
				config_mode_setting[i] = user_input_[i];
			for (; i < device_name_length; i++)
				config_mode_setting[i] = 0;
			// Null-terminate the local buffer (beyond the on-wire field) so the
			// name is safe to use as a C string (e.g. for BLE AT commands, display).
			config_mode_setting[device_name_length] = 0;
		}
		char_pos = 0;
		for (uint8_t i = 0; i < device_name_buffer_size; i++)
			user_input_[i] = 0;
		user_interaction_state_ = UserInteractionState::ConfigHome;
		DisplayConfigSettingsMenu();
	} else if (uart_char == 8 && char_pos > 0) {
		HAL_UART_Transmit(&huart2_, (uint8_t*) bs_, 3, uart_timeout);
		user_input_[--char_pos] = 0;
	} else if (uart_char >= ' ' && uart_char <= '~' && char_pos < device_name_length) {
		HAL_UART_Transmit(&huart2_, &uart_char, 1, uart_timeout);
		user_input_[char_pos++] = uart_char;
	}
}

void UserInteraction::DisplayDfuMenu() {
	StaticStringWriter<UART_LINE_MAX_LENGTH> export_line(&huart2_);
	export_line.WriteMany(clear_screen_, dfu_intro_, dfu_guidance_text_, dfu_warning_text_, crlf_);
}

uint8_t UserInteraction::StartBootloader() {
	FLASH_OBProgramInitTypeDef ob_cfg;
	HAL_FLASHEx_OBGetConfig(&ob_cfg);
	ob_cfg.OptionType = OPTIONBYTE_USER;
	ob_cfg.UserType = OB_USER_nBOOT0 | OB_USER_nBOOT1 | OB_USER_nSWBOOT0;
	ob_cfg.UserConfig = OB_BOOT0_RESET | OB_BOOT1_SET | OB_BOOT0_FROM_OB;

	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();
	HAL_FLASHEx_OBProgram(&ob_cfg);
//    HAL_FLASH_OB_Launch();
//    HAL_FLASH_OB_Lock();
//    HAL_FLASH_Lock();
	return 0;
}

void UserInteraction::SetUserInteractionState(UserInteractionState user_interaction_state) {
	user_interaction_state_ = user_interaction_state;
}
