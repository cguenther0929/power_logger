/*
 * uart.h
 *
 *  Created on: May 24, 2021
 *      Author: C. Guenther
 */

#ifndef INC_UART_H_
#define INC_UART_H_

// #include "stdint.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"
extern UART_HandleTypeDef huart1;


/* Linefeed and Enter Key */
#define LF                          1           //Define short hand for a line feed
#define CR                          2           //Define short hand for carriage return
#define ENTER_KEY                   0x0D     // ASCII code for enter key

/**
 * FUNCTION: void print_float (float number, uint8_t action)
 * --------------------
 * Leverage built-in sprintf to print
 * a float value to the terminal.  
 *
 * returns: Nothing 
 */
void print_float (float number, uint8_t action);

/**
 * FUNCTION: void print_string(const char * s, uint8_t action)
 * --------------------
 * Print a string through USART1
 *
 * returns: Nothing 
 */
void print_string(const char * s, uint8_t action);

/**
 * FUNCTION: void print_16b_binary_rep (uint16_t number, uint8_t action)
 * --------------------
 * Print a 16bit number, as binary representation
 *
 * returns: Nothing 
 */
void print_16b_binary_rep (uint16_t number, uint8_t action);

#endif /* INC_UART_H_ */
