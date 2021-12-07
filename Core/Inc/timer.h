/*
 * timer.h
 *
 *  Created on: May 16, 2021
 *      Author: C. Guenther
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "stdbool.h"
#include "stm32f1xx_hal.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

struct timing {
	uint8_t		ticks10ms;
	uint8_t		ticks100ms;
	uint8_t		ticks500ms;

	bool		flag_10ms_tick;
	bool		flag_100ms_tick;
	bool		flag_500ms_tick;

	bool		led_fast_blink;

};

/**
 * TODO NEED TO COMMENT 
 */
void blocking_us_delay (uint16_t us);

/**
 * @brief 
 * TODO need to comment
 */
void start_us_counter ( void );

/**
 * @brief 
 * TODO need to comment
 */
void stop_us_counter ( void ); 

/**
 * @brief 
 * TODO need to comment
 */
uint16_t get_us_counter ( void );


#endif /* INC_TIMER_H_ */
