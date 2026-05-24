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
		UART_HandleTypeDef &huart2) :
		comm_(comm), archive_(archive), huart2_(huart2) {
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
					for (uint8_t i = 0; i < device_name_length; i++)
						device_name_[i] = receiver_settings.device_name[i];
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
			for (uint8_t i = 0; i < device_name_length; i++)
				receiver_settings.device_name[i] = device_name_[i];
			archive_.SaveReceiverSettings(receiver_settings);
			comm_.SetChannel(lora_channel_);
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
		}
		break;
	case UserInteractionState::EditLoraChannel:
		AdjustConfigNumericSetting(uart_char, &lora_channel_, max_lora_channel, false);
		break;
	case UserInteractionState::EditDeviceName:
		AdjustConfigTextSetting(uart_char, device_name_);
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
	export_line.WriteMany(device_name_text_, device_name_);
	export_line.WriteMany(crlf_, crlf_);
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
		}
		char_pos = 0;
		for (uint8_t i = 0; i < device_name_length; i++)
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
