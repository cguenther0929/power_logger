/*
 * uart.c
 *
 *  Created on: May 24, 2021
 *      Author: CJGWORK
 */

#include "uart.h"

void print_float (float number, uint8_t action) {
    uint8_t temp_buffer[8];        //Define the array that will hold the ASCII values

    /* USE SPRINT F TO BUILD THE ARRAY OF ASCII CHARACTERS */
    sprintf((char *)temp_buffer, "%.4f", number);   //f tells the function we want to print a float value

    
    HAL_UART_Transmit(&huart1, temp_buffer, sizeof(temp_buffer), HAL_MAX_DELAY);
    
    /* CHECK TO SEE IF THE USER WISHES TO CREATE A NEW LINE */
    if(action == LF) {
        HAL_UART_Transmit(&huart1,(uint8_t *) '\r', (uint16_t) 0x01, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t *) '\n', (uint16_t) 0x01, HAL_MAX_DELAY);
    }
    else if(action == CR) {
        HAL_UART_Transmit(&huart1, (uint8_t *) '\r', (uint16_t) 0x01, HAL_MAX_DELAY);
    }
}

void print_16b_binary_rep (uint16_t number, uint8_t action) {
    uint16_t tx_char;      
    uint16_t i;         

    for(i = 16; i > 0; i--) {
        tx_char = (uint16_t)(number >> (i - 1));
        tx_char = (uint8_t)(tx_char & 0x01);
        tx_char = (uint8_t)(tx_char + 0x30);         //Turn this into an ASCII value
        
        HAL_UART_Transmit(&huart1, (uint8_t *) tx_char, (uint16_t) 0x01, HAL_MAX_DELAY);
    }

    /* CHECK TO SEE IF THE USER WISHES TO CREATE A NEW LINE */
    if(action == LF) {
        HAL_UART_Transmit(&huart1,(uint8_t *) '\r', (uint16_t) 0x01, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t *) '\n', (uint16_t) 0x01, HAL_MAX_DELAY);
    }
    else if(action == CR) {
        HAL_UART_Transmit(&huart1, (uint8_t *) '\r', (uint16_t) 0x01, HAL_MAX_DELAY);
    }
}

void print_string(const char * s, uint8_t action) {

    while(*s != '\0') {
        HAL_UART_Transmit(&huart1, (uint8_t *) s, (uint16_t) 0x01, HAL_MAX_DELAY);
        s++;
    }

    /* CHECK TO SEE IF THE USER WISHES TO CREATE A NEW LINE */
    if(action == LF) {
        HAL_UART_Transmit(&huart1,(uint8_t *) '\r', (uint16_t) 0x01, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t *) '\n', (uint16_t) 0x01, HAL_MAX_DELAY);
    }
    else if(action == CR) {
        HAL_UART_Transmit(&huart1, (uint8_t *) '\r', (uint16_t) 0x01, HAL_MAX_DELAY);
    }

}
