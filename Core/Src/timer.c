/*
 * timer.c
 *
 *  Created on: Nov 12, 2021
 *      Author: C. Guenther
 */
#include "timer.h"

void blocking_us_delay (uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim2,0);    // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
}


