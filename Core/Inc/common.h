/*
 * common.h
 *
 *  Created on: Nov 28, 2021
 *      Author: clint
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#define BTN_DEBOUNCE_THRESHOLD      15      // Buttons are evaluated every 10ms, so 15*10ms = 150ms


struct buttonStruct {         //Structure to pass information that is shared among modules

    /* Button Related */
    uint8_t     up_btn_press_ctr;
    uint8_t     rt_btn_press_ctr;
    uint8_t     dn_btn_press_ctr;
    uint8_t     lt_btn_press_ctr;
    
    bool        up_btn_pressed;
    bool        rt_btn_pressed;
    bool        dn_btn_pressed;
    bool        lt_btn_pressed;

    
};


#endif /* INC_COMMON_H_ */
