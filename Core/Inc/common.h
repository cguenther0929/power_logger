/*
 * common.h
 *
 *  Created on: Nov 28, 2021
 *      Author: clint
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#define BTN_DEBOUNCE_THRESHOLD      15      // Buttons are evaluated every 10ms, so 15*10ms = 150ms

typedef enum {
    UP_BTN_PUSHED,
    RT_BTN_PUSHED,
    DN_BTN_PUSHED,
    LT_BTN_PUSHED,
    NO_BTN_PUSHED
} ButtonPressed;

struct buttonStruct {         //Structure to pass information that is shared among modules

    ButtonPressed button_press_status;
    
    /* Button Related */
    uint8_t     up_btn_press_ctr;
    uint8_t     rt_btn_press_ctr;
    uint8_t     dn_btn_press_ctr;
    uint8_t     lt_btn_press_ctr;
    
    // bool        up_btn_pressed;
    // bool        rt_btn_pressed;
    // bool        dn_btn_pressed;
    // bool        lt_btn_pressed;

    
};

typedef enum
{
    STATE_IDLE,
    STATE_GRAB_SENSOR_DATA,
    STATE_LOG_SAMPLES,
    STATE_UPDATE_DISPLAY
} AppState;

typedef enum
{
    NO_ERROR,
    SD_FAILED_MOUNT,
    SD_OPEN_FILE,
    SD_CHECK_MEMORY,
    SD_LOW_ON_MEMORY,
    SD_CLOSING_FILE,
    READING_A2D
} ErrorCodes;

typedef struct {
    ErrorCodes error_code;
} errorCode;


#endif /* INC_COMMON_H_ */
