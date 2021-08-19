/*
 * timer.h
 *
 *  Created on: May 16, 2021
 *      Author: CJGWORK
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "stdbool.h"

struct timing {
	uint8_t		ticks10ms;
	uint8_t		ticks100ms;
	uint8_t		ticks500ms;

	bool		flag_10ms_tick;
	bool		flag_100ms_tick;
	bool		flag_500ms_tick;

	bool		led_fast_blink;

};


#endif /* INC_TIMER_H_ */
