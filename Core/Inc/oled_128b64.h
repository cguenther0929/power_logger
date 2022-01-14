/**
  ******************************************************************************
  * @file           : oled_128b64.h
  * @brief          : Header for oled routines 
  ******************************************************************************
  */

#ifndef INC_OLED_128B64_H_
#define INC_OLED_128B64_H_

#include "stm32f1xx_hal.h"
#include "font.h"
#include "ad4681.h"
#include "uart.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern I2C_HandleTypeDef hi2c2;

#define MAX_SCREEN_INDEX     4

typedef enum {
    SCREEN_MAIN,
    SCREEN_RUN_TIME_HR,
    SCREEN_RUN_TIME_MIN,
    SCREEN_SENSE_RESISTOR,
    SCREEN_LOGGING

} ScreenID;

#define MAX_SCREEN_BUFFER

struct oled {
    ScreenID    current_screen;

    uint8_t     screen_width;
    uint8_t     screen_height;

    uint8_t     textsize_x;
    uint8_t     textsize_y;

    bool        wrap_text;

    uint8_t     cursor_y;           // Current position of the y-cursor
    uint8_t     cursor_x;           // Current position of the x-cursor

    GFXfont     * oled_font;          // Pointer to font data

    uint8_t     screen_buffer[MAX_SCREEN_BUFFER];

};

#define OLED_SCREEN_ADDRESS             (0x3C << 1)

#define SSD1306_EXTERNALVCC             0x01    // External display voltage source
#define SSD1306_SWITCHCAPVCC            0x02    // Gen. display voltage from 3.3V

#define SSD1306_MEMORYMODE              0x20   
#define SSD1306_COLUMNADDR              0x21   
#define SSD1306_PAGEADDR                0x22   
#define SSD1306_SETCONTRAST             0x81   
#define SSD1306_CHARGEPUMP              0x8D   
#define SSD1306_SEGREMAP                0xA0   
#define SSD1306_DISPLAYALLON_RESUME     0xA4   
#define SSD1306_DISPLAYALLON            0xA5   
#define SSD1306_NORMALDISPLAY           0xA6   
#define SSD1306_INVERTDISPLAY           0xA7   
#define SSD1306_SETMULTIPLEX            0xA8       
#define SSD1306_DISPLAYOFF              0xAE       
#define SSD1306_DISPLAYON               0xAF       
#define SSD1306_COMSCANINC              0xC0       
#define SSD1306_COMSCANDEC              0xC8       
#define SSD1306_SETDISPLAYOFFSET        0xD3    
#define SSD1306_SETDISPLAYCLOCKDIV      0xD5  
#define SSD1306_SETPRECHARGE            0xD9       
#define SSD1306_SETCOMPINS              0xDA       
#define SSD1306_SETVCOMDETECT           0xDB       

#define SSD1306_SETSTARTLINE            0x40

#define SSD1306_DEACTIVATE_SCROLL       0x2E        //Stop scrolling

#define SSD1306_BLACK                   0           // Draw 'off' pixels
#define SSD1306_WHITE                   1           // Draw 'on' pixels
#define SSD1306_INVERSE                 2           // Invert pixels




/**
 * FUNCTION: void display_oled_init ( uint8_t voltage_state, uint8_t w, uint8_t h )
 * --------------------
 * @brief Initialize the OLED display
 * 
 * @param    voltage_state Determines how display 
 *                         generates power (i.e. internal vs. external)
 * @param    w   Width of screen 
 * @param    h   Height of screen
 * 
 * @return   Nothing 
 * 
*/
void display_oled_init ( uint8_t voltage_state, uint8_t w, uint8_t h );



/**************************************************************************/
/*!
//TODO cleanup comment
    @brief   Set text 'magnification' size. Each increase in s makes 1 pixel
   that much bigger.
    @param  s_x  Desired text width magnification level in X-axis. 1 is default
    @param  s_y  Desired text width magnification level in Y-axis. 1 is default

*/
/**************************************************************************/
/**
 * FUNCTION: void setTextSize (uint8_t s_x, uint8_t s_y)
 * --------------------
 *  @brief Set text 'magnification' size. Each increase in s makes 1 pixel
 *          that much bigger.
 * 
 *  @attention A scale of 1,1 allows for 21 characters per line
 *          while a scale of 2,2 allows for only 10 characters per line
 * 
 *  @param  s_x  Desired text width magnification level in X-axis. 1 is default
 *  @param  s_y  Desired text width magnification level in Y-axis. 1 is default
 * 
 * @return   Nothing 
 * 
*/
void setTextSize (uint8_t s_x, uint8_t s_y);

/**
 * FUNCTION: display_oled_drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
                              int16_t w, int16_t h, uint16_t color);
 * --------------------
 * @brief Draw a bitmap image to the screen
 * 
 * @param    x   Top left corner x coordinate
 * @param    y   Top left corner y coordinate
 * @param    bitmap  byte array with monochrome
 * @param    w   Width of bitmap in pixels
 * @param    h   Height of bitmap in pixels
 * @param    color 16-bit 5-6-5 Color to draw w
 * 
 * @return   Nothing 
 * 
*/
void display_oled_drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
                              int16_t w, int16_t h, uint16_t color);


/**
 * FUNCTION: void ssd1306_commandList(const uint8_t * command_pointer,
 *                                   uint8_t bytes_to_transmit)
 * --------------------
 * @brief    Send an array of commands to the OLED
 * 
 * @param    command_pointer   Pointer to array of commands to send 
 * @param    bytes_to_transmit Number of bytes to transmit
 * 
 * @return   Nothing 
 * 
*/
void ssd1306_commandList(const uint8_t * command_pointer, uint8_t bytes_to_transmit);


/**
 * FUNCTION: void ssd1306_command1(uint8_t command)
 * --------------------
 * @brief    Send a single command to the OLED driver 
 * 
 * @param    command    Command to send to OLED driver
 * @param    bytes_to_transmit Number of bytes to transmit
 * 
 * @return   Nothing 
 * 
*/
void ssd1306_command1(uint8_t command);


/**
 * FUNCTION: bool drawPixel(int16_t x, int16_t y, uint8_t color)
 * --------------------
 * @brief   Draw a single pixel to the screen.
 *          Color option can be passed in. 
 * 
 * @param   x   x cordinate location of pixel
 * @param   y   y cordinate location of pixel
 * 
 * @return  true if success 
 * 
*/
bool drawPixel(int16_t x, int16_t y, uint8_t color);

//TODO need to cleanup following function
void writeOledString(const char * c, uint8_t color);

/**
 * FUNCTION: void drawChar(int16_t x, int16_t y, unsigned char c,
 *                          uint16_t color, uint8_t size_x,
 *                          uint8_t size_y);
 * --------------------
 * @brief   Draw a single character at the defined 
 *          x/y location.    
 * 
 * @param   x       x location of character
 * @param   y       y location of character
 * @param   c       character to be drawn. 
 * @param   size_x  Character scaling (1 is normal
 *                  size, while 2 is 2x) 
 * @param   size_y  Character scaling (1 is normal
 *                  size, while 2 is 2x) 
 * 
 * @return  nothing  
 * 
*/
void drawChar(int16_t x, int16_t y, unsigned char c,
                            uint16_t color, uint8_t size_x,
                            uint8_t size_y);

/**
 * FUNCTION: void setFont (const GFXfont *f)
 * --------------------
 * @brief   Define the font to be used.  Associated 
 *          header file shall be included.  
 * 
 * @param   f   Name of font to be used.
 *              i.e. FreeSans9pt7b
 * 
 * @return  nothing  
 * 
*/
void setFont(GFXfont *f);


/**
 * FUNCTION: void fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                            uint16_t color);
 * --------------------
 * @brief   Make solid rectangle 
 * 
 * @param   x   Upper left location of 
 *              rectangle (x-coordinate).
 * @param   y   Upper left location of 
 *              rectangle (y-coordinate).
 * @param   w   Width of rectangle (pixels)
 * @param   h   Height of rectangle (pixels)
 * @param   color   Fill color 
 * 
 * @return  nothing  
 * 
*/
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                            uint16_t color);

//TODO need to comment
void drawFastVLine(int16_t x, int16_t y, int16_t h,
                                 uint16_t color);

//TODO need to comment 
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                             uint16_t color);

//TODO need to comment
void oled_clear(void);


/**
 * FUNCTION: void updateDisplay(void)
 * --------------------
 * 
 * @brief  Push data currently in RAM to SSD1306 display.
 * 
 * @note   Drawing operations are not visible until this function is
 * called. Call after each graphics command, or after a whole set
 * of graphics commands.
 * 
 * @return nothing
 * 
*/
void updateDisplay(void);

/**************************************************************************/
/*!
//TODO: need to cleanup this comment 
   @brief    Fill the screen completely with one color. Update in subclasses if
   desired!
    @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void fillScreen(uint16_t color);


/**************************************************************************/
/*!
//TODO need to cleanup comment
    @brief  Print one byte/character of data, used to support print()
    @param  c  The 8-bit ascii character to write
    @param  color Options are SSD1306_WHITE, SSD1306_BLACK, SSD1306_INVERSE
*/
/**************************************************************************/
void writeStringHelper(uint8_t c, uint8_t color);

#endif /* INC_OLED_128B64_H_ */
