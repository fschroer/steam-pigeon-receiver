/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "subghz_phy_app.h"
#include "sys_app.h"
#include "radio.h"
#include <Factory_C_Interface.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// The 1-byte buffer the HAL will write to
extern uint8_t bt_byte_;
extern uint8_t charRx;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void LoraTxCallback();
void LoraRxCallback(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);
void UART1_CharReception_Callback(void);
void UART2_CharReception_Callback(void);
void UART_TXEmpty_Callback(void);
void UART_CharTransmitComplete_Callback(void);
void UART_Error_Callback(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define USART1_TX_Pin GPIO_PIN_6
#define USART1_TX_GPIO_Port GPIOB
#define USARTx_RX_Pin GPIO_PIN_3
#define USARTx_RX_GPIO_Port GPIOA
#define USARTx_TX_Pin GPIO_PIN_2
#define USARTx_TX_GPIO_Port GPIOA
#define SOFT_LED5_Pin GPIO_PIN_7
#define SOFT_LED5_GPIO_Port GPIOA
#define SOFT_LED3_Pin GPIO_PIN_10
#define SOFT_LED3_GPIO_Port GPIOB
#define SOFT_LED2_Pin GPIO_PIN_4
#define SOFT_LED2_GPIO_Port GPIOA
#define SOFT_LED1_Pin GPIO_PIN_5
#define SOFT_LED1_GPIO_Port GPIOA
#define SOFT_LED4_Pin GPIO_PIN_8
#define SOFT_LED4_GPIO_Port GPIOA
#define BATTRD_Pin GPIO_PIN_12
#define BATTRD_GPIO_Port GPIOB
#define BT_LED_Pin GPIO_PIN_0
#define BT_LED_GPIO_Port GPIOA
#define SCLK_Pin GPIO_PIN_13
#define SCLK_GPIO_Port GPIOB
#define SDO_Pin GPIO_PIN_14
#define SDO_GPIO_Port GPIOB
#define CSB_MEM_Pin GPIO_PIN_13
#define CSB_MEM_GPIO_Port GPIOC
#define SDI_Pin GPIO_PIN_10
#define SDI_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_7
#define USART1_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
