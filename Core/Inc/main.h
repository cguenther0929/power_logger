/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "string.h"
#include "ad4681.h"
#include "timer.h"
#include "uart.h"
#include "oled_128b64.h"
#include "font.h"
#include "config.h"
#include "common.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
/** 
 * TODO need to comment the following
 */
void blocking_delay_10ms_ticks (uint16_t ticks);

/** 
 * TODO need to comment the following
 */
void blocking_delay_500ms_ticks(uint16_t ticks);

/**
 * @brief Function to update OLED screen 
 * TODO need to comment
 */
void update_screen( void );

/**
 * @brief Evaluate button inputs
 * TODO need to comment
 */

/**
 * FUNCTION: void evaluate_button_inputs ( void );
 * --------------------
 * @brief Evaluate button inputs.  This function
 *      also handles the proper debouncing of buttons
 *      and setting flags when buttons are pressed.
 * 
 * @return   Nothing 
 * 
*/
void evaluate_button_inputs ( void );

/**
 * FUNCTION: void evaluate_state();
 * --------------------
 * @brief Evaluate the current state, and whether or 
 *      not the application should proceed to the next 
 *      state.  
 * 
 * @return   Nothing 
 * 
*/
void evaluate_state ( void );



/**
 * @brief Log power statistics to SD card
 * TODO need to comment 
 */
void log_samples( void ); 

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_DUT_PWR_Pin GPIO_PIN_2
#define EN_DUT_PWR_GPIO_Port GPIOE
#define SWO_Pin GPIO_PIN_9
#define SWO_GPIO_Port GPIOB
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define PB1_n_Pin GPIO_PIN_3
#define PB1_n_GPIO_Port GPIOE
#define PB2_n_Pin GPIO_PIN_4
#define PB2_n_GPIO_Port GPIOE
#define HLTH_LED_Pin GPIO_PIN_1
#define HLTH_LED_GPIO_Port GPIOE
#define CONSOLE_TX_Pin GPIO_PIN_9
#define CONSOLE_TX_GPIO_Port GPIOA
#define PB3_n_Pin GPIO_PIN_5
#define PB3_n_GPIO_Port GPIOE
#define CONSOLE_RXD_Pin GPIO_PIN_10
#define CONSOLE_RXD_GPIO_Port GPIOA
#define PB4_n_Pin GPIO_PIN_6
#define PB4_n_GPIO_Port GPIOE
#define IN_FD_V_Pin GPIO_PIN_0
#define IN_FD_V_GPIO_Port GPIOC
#define ADC_SPI1_CSn_Pin GPIO_PIN_4
#define ADC_SPI1_CSn_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_10
#define LED4_GPIO_Port GPIOE
#define REV_2_Pin GPIO_PIN_15
#define REV_2_GPIO_Port GPIOD
#define ADC_SPI1_CLK_Pin GPIO_PIN_5
#define ADC_SPI1_CLK_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOE
#define REV_1_Pin GPIO_PIN_14
#define REV_1_GPIO_Port GPIOD
#define ADC_SPI1_MISO_Pin GPIO_PIN_6
#define ADC_SPI1_MISO_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOE
#define DISP_I2C2_SCL_Pin GPIO_PIN_10
#define DISP_I2C2_SCL_GPIO_Port GPIOB
#define REV_0_Pin GPIO_PIN_13
#define REV_0_GPIO_Port GPIOD
#define ADC_ALRTn_Pin GPIO_PIN_3
#define ADC_ALRTn_GPIO_Port GPIOA
#define ADC_SPI1_MOSI_Pin GPIO_PIN_7
#define ADC_SPI1_MOSI_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOE
#define DISP_I2C2_SDA_Pin GPIO_PIN_11
#define DISP_I2C2_SDA_GPIO_Port GPIOB
#define SD_SPI2_CSn_Pin GPIO_PIN_12
#define SD_SPI2_CSn_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi2

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
