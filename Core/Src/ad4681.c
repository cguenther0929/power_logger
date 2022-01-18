/*
 * ad4681.c
 *
 *  Created on: Sep 22, 2021
 *      Author: C. Guenther
 */

#include "ad4681.h"

// struct accel_data accel;    //TODO may need to define this
// struct ad4681Data a2d;


void init_ad4681 (ad4681Data * a2d) {

    /**
     * The configuration 1 register
     * is reset and defaults as  
     * 0x0000, which results in 
     * the following settings:
     * Oversampling Mode = Disabled
     * Oversampling Ratio = Disabled
     * CRC Write = Disabled
     * CRC Read = Disabled
     * Alert Indicator Function = Disabled
     * Resolution = Normal Resolution
     * Reference Select = Internal Reference Selected
     * Power Down Mode = Normal Mode
     * 
     * Details of this register can be 
     * read on datasheet p26 of 29.  
     */


    /**
     * The configuration 2 register
     * is reset and defaults as  
     * 0x0000, which results in 
     * the following settings:
     * Conversion results data format = 2-wire output,
     * so data is output on _both_ pins SDOA and SDOB
     * The reset word = 0x00 (no reset)
     * 
     * Details of this register can be 
     * read on datasheet p27 of 29.  
     */
    const uint8_t spi_data[] = {
                (AD4681_WRITE_BIT << AD4681_WR_BIT_OFFSET) |
                (AD4681_CONFIG2_REG_ADDR << AD4681_ADDR_BIT_OFFSET) |
                (OUTPUT_ON_SDOA_ONLY << CONVERSION_MODE_BIT_OFFSET )
    };

    HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_data, (uint16_t) 2, (uint32_t) 10000);     // Timeout in us


    /**
     * Initializations related to
     * the A2D interface
   */
    a2d -> sense_resistors[0] = 0.022f;
    a2d -> sense_resistors[1] = 0.091f;
    a2d -> sense_resistors[1] = 0.82f;
    a2d -> cs_res_index = 0;
    a2d -> cs_res_f = a2d -> sense_resistors[a2d -> cs_res_index];        // Set this to a default value
    a2d -> first_sample = false;
    a2d -> logging_status = false;

    a2d -> run_time_hr = 0.0f;
    a2d -> run_time_min = 0.0f;
    a2d -> time_us_elapsed = 0;

}

get_ad4681_samples( ad4681Data * a2d ) {
    
    /**
     * Get time value to know time 
     * elapsed between samples
     * TODO need to define this
     */
    if(!a2d -> first_sample) {
        a2d -> time_us_elapsed = get_us_counter();
    }
    
    
    
    /**
     *  Grab A and B samples.
     * In order for low latency p22
     * in datasheet, need to pulse the 
     * CS line low before taking the sample.
     */
    HAL_GPIO_WritePin(ADC_SPI1_CSn_GPIO_Port, ADC_SPI1_CSn_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    // blocking_us_delay(CS_PULSE_DELAY_uS);      //TODO we want this line back in
    HAL_GPIO_WritePin(ADC_SPI1_CSn_GPIO_Port, ADC_SPI1_CSn_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    // blocking_us_delay(CS_PULSE_DELAY_uS);        //TODO we want this line back in

    /** 
     * Drop the CS line 
     * to extract the data.
     * Extract data, then bring
     * CS line high again.
    */
    HAL_GPIO_WritePin(ADC_SPI1_CSn_GPIO_Port, ADC_SPI1_CSn_Pin, GPIO_PIN_RESET);
    
    HAL_SPI_Receive(&hspi1, (uint8_t *)a2d -> ad4681_buffer, 4, 1000);
    
    HAL_GPIO_WritePin(ADC_SPI1_CSn_GPIO_Port, ADC_SPI1_CSn_Pin, GPIO_PIN_SET);
    
    /* Parse into voltage vs. current */
    a2d -> voltage_sample = (uint16_t)((a2d -> ad4681_buffer[3] << 8) | (a2d -> ad4681_buffer[2]));
    a2d -> current_sample = (uint16_t)((a2d -> ad4681_buffer[1] << 8) | (a2d -> ad4681_buffer[0]));

    /**
     *  Determine if sign bit
     * is set for twos complement
     * start with voltage sample
     */
    if((uint16_t)(a2d -> voltage_sample >> 15 & 0x01) == 1 ) {   
        a2d -> voltage_sample ^= 0xFFFF;      // Invert all bits
        a2d -> voltage_sample += 0x01;        // Add one
        
        a2d -> voltage_f = (float)(A2D_VOLTAGE_PER_BIT * a2d -> voltage_sample);
        a2d -> voltage_f = (float)(a2d -> voltage_f * VOLTAGE_MEAS_GAIN * -1.0);
    }
    else {
        a2d -> voltage_f = (float)(A2D_VOLTAGE_PER_BIT * a2d -> voltage_sample);
        a2d -> voltage_f = (float)(a2d -> voltage_f * VOLTAGE_MEAS_GAIN);
    }

    /**
     * Determine current 
     * draw value
     */
    if((uint16_t)(a2d -> current_sample >> 15 & 0x01) == 1 ) {   
        a2d -> current_sample ^= 0xFFFF;      // Invert all bits
        a2d -> current_sample += 0x01;        // Add one

        a2d -> current_f = (float)(a2d -> current_sample * A2D_VOLTAGE_PER_BIT);        // Raw A2D voltage value
        a2d -> current_f = (float)(a2d -> current_f / a2d -> cs_res_f * -1.0);                 // Current = Voltage / Resistance 

    }
    else {
        a2d -> current_f = (float)(a2d -> current_sample * A2D_VOLTAGE_PER_BIT);        // Raw A2D voltage value
        a2d -> current_f = (float)(a2d -> current_f / a2d -> cs_res_f);                 // Current = Voltage / Resistance 
    }

    /**
     * Calculate power 
     * value 
     */
    a2d -> power_f = (float)(a2d -> current_f * a2d -> voltage_f);

    a2d -> first_sample = false;

    /**
     * Start us timer
     * so the elapsed time
     * can be calculated the next 
     * time this function is entered
    */
   start_us_counter();
    

}



