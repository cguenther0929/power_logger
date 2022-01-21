/*
 * ad4681.h
 *
 *  Created on: Sep 22, 2021
 *      Author: C. Guenther
 */

#ifndef INC_AD4681_H_
#define INC_AD4681_H_

#include "stm32f1xx_hal.h"
#include "main.h"
#include "timer.h"
#include "oled_128b64.h"
#include "font.h"
extern SPI_HandleTypeDef hspi1;

typedef struct {
    
    bool        first_sample;                   // False until first sample is recorded
    bool        logging_status;                 // Indicates if unit is 'running'
    uint8_t     ticks_10ms_between_samples;     // How many multiples of 10ms has passes since last reading
    uint16_t    time_us_elapsed;                // How many us elapsed since last reading
    
    
    uint8_t ad4681_buffer[4];       // Each transation will require four bytes (32 bits)
    uint16_t voltage_sample;
    uint16_t current_sample;
    
    float   voltage_f;
    float   current_f;
    float   power_f;

    uint8_t cs_res_index;
    float   cs_res_f;           // This value will need to be configured by the user
    float   sense_resistors[3];

    float   run_time_hr;
    float   run_time_min;


} ad4681Data;


/**
 * General AD4681 settings, 
 * like sample rates, and 
 * data resolution.
 * Note, data is returned in 
 * twos complement format.
 */
#define A2D_VOLTAGE_PER_BIT             (float)(0.0000763)
#define VOLTAGE_MEAS_GAIN               (float)(12.0476)
#define CS_PULSE_DELAY_uS               10          //TODO may need to increase the delay


/**
 * Read vs. Write Bit settings
 */
#define AD4681_WR_BIT_OFFSET            0x0F
#define AD4681_WRITE_BIT                0x01
#define AD4681_READ_BIT                 0x00

/**
 * Register addresses
 */
#define AD4681_ADDR_BIT_OFFSET          0x0C
#define AD4681_CONFIG1_REG_ADDR         0x01
#define AD4681_CONFIG2_REG_ADDR         0x02
#define AD4681_ALERT_REG_ADDR           0x03
#define AD4681_ALERT_LOW_REG_ADDR       0x04
#define AD4681_ALERT_HIGH_REG_ADDR      0x05

/** 
 * Configuration 1 
 * registers parameters
 * -------------------------
 * Oversample mode  
*/
#define OVSAMP_MODE_BIT_OFFSET          0x09
#define OVSAMP_MODE_DISABLED            0x00
#define OVSAMP_MODE_ENABLED             0x01

/** 
 * Oversampling ratio parameters
 * 000: disabled (default)
 * 001: 2X mode
 * 010: 4X mode
 * 011: 8X mode
 * 1xx: disabled
*/
#define OVSAMP_RATIO_BIT_OFFSET         0x06

/** 
 * CRC Write Mode
*/
#define CRC_WRITE_BIT_OFFSET            0x05
#define CRC_WRITE_DISABLED              0x00
#define CRC_WRITE_ENABLED               0x01

/** 
 * CRC Read Mode
*/
#define CRC_READ_BIT_OFFSET             0x04
#define CRC_READ_DISABLED               0x00
#define CRC_READ_ENABLED                0x01

/** 
 * Alert Mode Parameters
 * -------------------------
 * This alert function 
 * (on the SDOB/ALERT pin) 
 * is enabled when the SDO 
 * bit (Register 0x2, Bit 8) = 1.
 * Otherwise, the ALERT_EN bit is ignored. 
*/
#define ALERT_ENABLE_BIT_OFFSET         0x03
#define ALERT_PIN_DISABLED              0x00
#define ALERT_PIN_ENABLED               0x01

/** 
 * Resolution mode parameters 
 */
#define RESOLUTION_BIT_OFFSET           0x02
#define RESOLUTION_NORMAL               0x00
#define RESOLUTION_2BIT_HIGHER          0x01

/** 
 * Reference select parameters
*/
#define REF_SELECT_BIT_OFFSET           0x01
#define INTERNAL_REFERENCE_SELECTED     0x00
#define EXTERNAL_REFERENCE_SELECTED     0x01

/** 
 * Power down mode parameters
*/
#define PWR_DOWN_MODE_BIT_OFFSET        0x00
#define PWR_DOWN_DISABLED               0x00
#define PWR_DOWN_ENABLED                0x01



/** 
 * Configuration 2
 * registers parameters
 * -------------------------
 * conversion results serial 
 * data output
*/
#define CONVERSION_MODE_BIT_OFFSET      0x08
#define OUTPUT_ON_SDOA_AND_SDOB         0x00
#define OUTPUT_ON_SDOA_ONLY             0x01        // Both samples clocked out on same pin (SDOA)

/**
 * Chip reset options
 * of configuration 2 register
 */
#define RESET_CHIP_BIT_OFFSET           0x00
#define SW_RESET_ONLY                   0x3C        // Datsheet p27
#define HW_RESET                        0xFF        // Datasheet p27

/**
 * Alert Register Parameters
 * -------------------------
 * CRC Error bit indication.
 * The CRC Error bit is sticky
 * and remains set until the register
 * is read. 
 */
#define CRCW_ERROR_BIT_OFFSET           0x09        // Access is read only

/**
 * Load Error Bit 
 * 0: No setup error.
 * 1: Setup error.
 * Only way to clear this bit is by
 * performing a hard reset via 
 * the configuration two register
 */
#define SETUP_F_BIT_OFFSET              0x08        // Access is read only

/**
 * Alert Low Threshold Register
 * parameters
 * -------------------------
 * 
 */

/**
 * FUNCTION: void init_ad4681 (void)
 * --------------------
 * @brief    Initialize the AD461 simultaneously 
 * sampling ADC
 * 
 * @return   Nothing 
 * 
*/
void init_ad4681 ( ad4681Data * a2d );

/**
 * TODO need to comment 
 */
void get_ad4681_samples( ad4681Data * a2d );


#endif /* INC_AD4681_H_ */
