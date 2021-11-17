/*
 * oled_128b64.c
 *
 *  Created on: Aug 18, 2021
 *      Author: C. Guenther
 */

#include "oled_128b64.h"

struct oled     oled;   

// extern const GFXfont FreeSans9pt7b;


#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
    {                                                                            \
        int16_t t = a;                                                             \
        a = b;                                                                     \
        b = t;                                                                     \
    }
#endif

void display_oled_init ( uint8_t voltage_state, uint8_t w, uint8_t h ) {

    oled.screen_width = w;
    oled.screen_height = h;

    oled.wrap_text = true;

    /**
     * Call function to clear the display
     */
    oled_clear();       


    /**
     * Massive Initialization Sequence 
     */
    static const uint8_t init1[] = {SSD1306_DISPLAYOFF,         // 0xAE
                                            SSD1306_SETDISPLAYCLOCKDIV, // 0xD5
                                            0x80, // the suggested ratio 0x80
                                            SSD1306_SETMULTIPLEX}; // 0xA8
    ssd1306_commandList(init1, sizeof(init1));
    ssd1306_command1(oled.screen_height - 1);

    static const uint8_t init2[] = {SSD1306_SETDISPLAYOFFSET, // 0xD3
                                            0x0,                      // no offset
                                            SSD1306_SETSTARTLINE | 0x0, // line #0
                                            SSD1306_CHARGEPUMP};        // 0x8D
    ssd1306_commandList(init2, sizeof(init2));

    ssd1306_command1((voltage_state == SSD1306_EXTERNALVCC) ? 0x10 : 0x14);

    static const uint8_t init3[] = {SSD1306_MEMORYMODE, // 0x20
                                            0x00, // 0x0 act like ks0108
                                            SSD1306_SEGREMAP | 0x1,
                                            SSD1306_COMSCANDEC};
    ssd1306_commandList(init3, sizeof(init3));

    uint8_t comPins = 0x02;
    uint8_t contrast = 0x8F;

    if ((oled.screen_width == 128) && (oled.screen_height == 32)) {
        comPins = 0x02;
        contrast = 0x8F;
    } else if ((oled.screen_width == 128) && (oled.screen_height == 64)) {
        comPins = 0x12;
        contrast = (voltage_state == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF;
    } else if ((oled.screen_width == 96) && (oled.screen_height == 16)) {
        comPins = 0x2; // ada x12
        contrast = (voltage_state == SSD1306_EXTERNALVCC) ? 0x10 : 0xAF;
    } else {
        // Other screen varieties -- TBD
    }

    ssd1306_command1(SSD1306_SETCOMPINS);
    ssd1306_command1(comPins);
    ssd1306_command1(SSD1306_SETCONTRAST);
    ssd1306_command1(contrast);

    ssd1306_command1(SSD1306_SETPRECHARGE); // 0xD9
    ssd1306_command1((voltage_state == SSD1306_EXTERNALVCC) ? 0x22 : 0xF1);
    
    static const uint8_t init5[] = {
        SSD1306_SETVCOMDETECT, // 0xDB
        0x40,
        SSD1306_DISPLAYALLON_RESUME, // 0xA4
        SSD1306_NORMALDISPLAY,       // 0xA6
        SSD1306_DEACTIVATE_SCROLL,
        SSD1306_DISPLAYON}; // Main screen turn on
    ssd1306_commandList(init5, sizeof(init5));
    
}

//TODO::: The following line is for reference only
//TODO::: see Adafruit_GFX.cpp line 1305
void setTextSize (uint8_t s_x, uint8_t s_y) {
    oled.textsize_x = (s_x > 0) ? s_x : 1;
    oled.textsize_y = (s_y > 0) ? s_y : 1;
}


void oled_clear(void) {
    memset(oled.screen_buffer, 0, oled.screen_width * ((oled.screen_height + 7) / 8));
}

void display_oled_drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
                              int16_t w, int16_t h, uint16_t color) {

    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for (int16_t j = 0; j < h; j++, y++) {
        for (int16_t i = 0; i < w; i++) {
            if (i & 7)
                byte <<= 1;
            else
                byte = bitmap[j * byteWidth + i / 8];
            if (byte & 0x80)
                drawPixel(x + i, y, color);
        }
    }
}


bool drawPixel(int16_t x, int16_t y, uint8_t color) {
    
    if ((x >= 0) && (x < oled.screen_width) && (y >= 0) && (y < oled.screen_height)) {
        // Pixel is in-bounds. Rotate coordinates if needed.
        // switch (getRotation()) {
            // case 1:
            // ssd1306_swap(x, y);
            // x = oled.screen_width - x - 1;
            // break;
            // case 2:
        x = oled.screen_width - x - 1;
        y = oled.screen_height - y - 1;
            // break;
            // case 3:
            // ssd1306_swap(x, y);
            // y = oled.screen_height - y - 1;
            // break;
        
        switch (color) {
            case SSD1306_WHITE:
                oled.screen_buffer[x + (y / 8) * oled.screen_width] |= (1 << (y & 7));
            break;
            
            case SSD1306_BLACK:
                oled.screen_buffer[x + (y / 8) * oled.screen_width] &= ~(1 << (y & 7));
            break;
            
            case SSD1306_INVERSE:
                oled.screen_buffer[x + (y / 8) * oled.screen_width] ^= (1 << (y & 7));
            break;
        }
        return true;
    }
    else {
        return false;
    }
}

void ssd1306_commandList(const uint8_t * command_pointer, uint8_t bytes_to_transmit) {

    /**
     * Set Co and D/C bit to zero
     */
    if (HAL_I2C_Master_Transmit(&hi2c2, OLED_SCREEN_ADDRESS, (uint8_t *)0x00, 1, 10000) != HAL_OK){
        asm("bkpt 255");
    }

    /**
     * Transmit the array of data
     */
    while(bytes_to_transmit--) {
        if (HAL_I2C_Master_Transmit(&hi2c2, OLED_SCREEN_ADDRESS, (uint8_t *) command_pointer, 1, 10000) != HAL_OK){
            asm("bkpt 255");
        }
        command_pointer++;
    }
}


void ssd1306_command1(uint8_t command) {
  
    /**
     * Set Co and D/C bit to zero
     */
    if (HAL_I2C_Master_Transmit(&hi2c2, OLED_SCREEN_ADDRESS, (uint8_t *) 0x00, 1, 10000) != HAL_OK){
        asm("bkpt 255");        //TODO need to figure out what this does
    }

    /**
     * Transmit the array of data
     */
    if (HAL_I2C_Master_Transmit(&hi2c2, OLED_SCREEN_ADDRESS, (uint8_t *) command, 1, 10000) != HAL_OK){
        asm("bkpt 255");        //TODO need to figure out what this does
    }

}

//TODO::: need to define a sort of "write string" function
//TODO::: this function will ultimately call "drawChar"
//TODO::: multiple times to wire a string to the display.  
//TODO::: see Adafruit_SSD1306.cpp file for such a function?  

// void UARTString (const char * y, uint8_t action ) {

//     while(*y != '\0'){
//         TXREG1 = *y;                    //Load the U1 TX buffer with the current character
//         TXWait();
//         y++;                           //Increment the pointer memory address
//     }

//     /* CHECK TO SEE IF THE USER WISHES TO CREATE A NEW LINE */
//     if(action == LF) {
//         TXREG1 = '\r';      //Return the cursor
//         TXWait();
//         TXREG1 = '\n';      //Put us on a new line -- must be in this order...
//         TXWait();
//     }
//     else if(action == CR) {
//         TXREG1 = '\r';      //Return the cursor
//         TXWait();
//     }
//     return;
// }

//TODO::: There needs to be a function for setting the 
//TODO::: cursor position 

void writeOledString(const char * c, uint8_t color) {
    while(*c != '\0'){
        writeStringHelper((uint8_t) *c,color);                    //Load the U1 TX buffer with the current character
        c++;                           //Increment the pointer memory address
    }
}


//TODO::: The following is for reference only and can be deleted
//TODO::: For this function, reference Adafruit_GFX.cpp  line 1243
//TODO::: See line 143 in this file, but color options are 
//TODO:::  SSD1306_WHITE, SSD1306_BLACK, SSD1306_INVERSE
void writeStringHelper(uint8_t c, uint8_t color) {
//   if (!gfxFont) { // 'Classic' built-in font

//     if (c == '\n') {              // Newline?
//       cursor_x = 0;               // Reset x to zero,
//       cursor_y += textsize_y * 8; // advance y one line
//     } else if (c != '\r') {       // Ignore carriage returns
//       if (oled.wrap_text && ((cursor_x + textsize_x * 6) > _width)) { // Off right?
//         cursor_x = 0;                                       // Reset x to zero,
//         cursor_y += textsize_y * 8; // advance y one line
//       }
//       drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize_x,
//                textsize_y);
//       cursor_x += textsize_x * 6; // Advance x one char
//     }

//   } 
  
//   else { // Custom font

    if (c == '\n') {
        oled.cursor_x = 0;
        oled.cursor_y += (int16_t)oled.textsize_y * (uint8_t)(oled.oled_font -> yAdvance);
    } 
    else if (c != '\r') {
        uint8_t first = oled.oled_font -> first;
        
        if ((c >= first) && (c <= (uint8_t)(oled.oled_font->last))) {

            // GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c - first);
            GFXglyph *glyph = oled.oled_font -> glyph + (c - first);

            uint8_t w = &glyph -> width;
            uint8_t h = &glyph -> height;
            
            if ((w > 0) && (h > 0)) { // Is there an associated bitmap?
                int16_t xo = (int8_t)(&glyph->xOffset); 
                if (oled.wrap_text && ((oled.cursor_x + oled.textsize_x * (xo + w)) > oled.screen_width)) {
                    oled.cursor_x = 0;
                    oled.cursor_y += (int16_t)oled.textsize_y * (uint8_t)(oled.oled_font -> yAdvance);
                }
                
                drawChar(oled.cursor_x, oled.cursor_y, c, color, 
                        oled.textsize_x, oled.textsize_y);
            }
            oled.cursor_x += (uint8_t)(&glyph -> xAdvance) * (int16_t)oled.textsize_x;
        }
    }
}


//TODO::: The following line is for reference only
//TODO::: see Adafruit_GFX.cpp line 1134
void drawChar(int16_t x, int16_t y, unsigned char c,
                            uint16_t color, uint8_t size_x,
                            uint8_t size_y) {

    /** 
     * Character is assumed previously filtered by write() to eliminate
     * newlines, returns, non-printable characters, etc.  Calling
     * drawChar() directly with 'bad' characters of font may cause mayhem!
     * 
    */

    c -= (uint8_t)(oled.oled_font -> first);
    
    GFXglyph *glyph = oled.oled_font -> glyph + c;

    // uint8_t *bitmap = pgm_read_bitmap_ptr(oled.oled_font);  //TODO can remove this line
    uint8_t *bitmap = oled.oled_font -> bitmap;

    uint16_t bo = (uint16_t)(&glyph->bitmapOffset);

    uint8_t w = (uint8_t)(&glyph -> width),
            h = (uint8_t)(&glyph -> height);

    int8_t xo = (int8_t)(&glyph -> xOffset),
           yo = (int8_t)(&glyph -> yOffset);
    
    uint8_t xx, yy, bits = 0, bit = 0;
    
    int16_t xo16 = 0, yo16 = 0;

    if (size_x > 1 || size_y > 1) {
      xo16 = xo;
      yo16 = yo;
    }

    // TODO: The following note about character clipping was 
    // TODO: copied directly from the arduino library, thus 
    // TODO: I'm not sure of it's meaning.
    // Todo: Add character clipping here 

    // NOTE: THERE IS NO 'BACKGROUND' COLOR OPTION ON CUSTOM FONTS.
    // THIS IS ON PURPOSE AND BY DESIGN.  The background color feature
    // has typically been used with the 'classic' font to overwrite old
    // screen contents with new data.  This ONLY works because the
    // characters are a uniform size; it's not a sensible thing to do with
    // proportionally-spaced fonts with glyphs of varying sizes (and that
    // may overlap).  To replace previously-drawn text when using a custom
    // font, use the getTextBounds() function to determine the smallest
    // rectangle encompassing a string, erase the area with fillRect(),
    // then draw new text.  This WILL infortunately 'blink' the text, but
    // is unavoidable.  Drawing 'background' pixels will NOT fix this,
    // only creates a new set of problems.  Have an idea to work around
    // this (a canvas object type for MCUs that can afford the RAM and
    // displays supporting setAddrWindow() and pushColors()), but haven't
    // implemented this yet.

    // startWrite();
    for (yy = 0; yy < h; yy++) {
      for (xx = 0; xx < w; xx++) {
        if (!(bit++ & 7)) {
          bits = (uint8_t)(&bitmap[bo++]);
        }
        if (bits & 0x80) {
          if (size_x == 1 && size_y == 1) {
            drawPixel(x + xo + xx, y + yo + yy, color);
          } else {
            fillRect(x + (xo16 + xx) * size_x, y + (yo16 + yy) * size_y,
                          size_x, size_y, color);
          }
        }
        bits <<= 1;
      }
    }
    // endWrite();


}

//TODO::: clean up stale code in the following
//TODO::: reference Adafruti_GFX.cpp line 1338
void setFont(GFXfont *f) {
//   if (f) {          // Font struct pointer passed in?
//     if (!font) { // And no current font struct?
      // Switching from classic to new font behavior.
      // Move cursor pos down 6 pixels so it's on baseline.
//       oled.cursor_y += 6;
//     }
//   } else if (font) { // NULL passed.  Current oledfont struct defined?
    // Switching from new to classic (5x7) font behavior.
    // Move cursor pos up 6 pixels so it's at top-left of char.
    // TODO The following was removed, as it shouldn't be needed
    // oled.cursor_y -= 6;
//   }
    oled.oled_font = f;
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                            uint16_t color) {
  for (int16_t i = x; i < x + w; i++) {
    drawFastVLine(i, y, h, color);
  }
}

//TODO::: The following line is for reference only
//TODO::: see Adafruit_GFX.cpp line 132

void drawFastVLine(int16_t x, int16_t y, int16_t h,
                                 uint16_t color) {
    drawLine(x, y, x, y + h - 1, color);
}

//TODO::: The following line is for reference only
//TODO::: see Adafruit_GFX.cpp line 316
void fillScreen(uint16_t color) {
  fillRect(0, 0, oled.screen_width, oled.screen_height, color);
}

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                             uint16_t color) {
  
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } 
    else {
        ystep = -1;
    }

    for (; x0 <= x1; x0++) {
        if (steep) {
            drawPixel(y0, x0, color);
        } 
        else {
            drawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}


void updateDisplay(void) {
//   TRANSACTION_START
    
    static const uint8_t dlist1[] = {
        SSD1306_PAGEADDR,
        0,                      // Page start address
        0xFF,                   // Page end (not really, but works here)
        SSD1306_COLUMNADDR, 0}; // Column start address
    ssd1306_commandList(dlist1, sizeof(dlist1));
    ssd1306_command1(oled.screen_width - 1); // Column end address

    uint16_t count = oled.screen_width * ((oled.screen_height + 7) / 8);
    uint8_t *data_pointer = oled.screen_buffer;
    // if (wire) { // I2C
        // wire->beginTransmission(i2caddr);
        // WIRE_WRITE((uint8_t)0x40);
        // uint16_t bytesOut = 1;
    while (count--) {
        
        /**
         * Transmit data from the 
         * internal buffer to the screen's 
         * memory
         */
        // while(bytes_to_transmit--) {
        if (HAL_I2C_Master_Transmit(&hi2c2, OLED_SCREEN_ADDRESS, (uint8_t *) data_pointer, 1, 10000) != HAL_OK){
            asm("bkpt 255");
        }
        data_pointer++;
        // }
        
        
        
        // if (bytesOut >= WIRE_MAX) {
        //     wire->endTransmission();
        //     wire->beginTransmission(i2caddr);
        //     WIRE_WRITE((uint8_t)0x40);
        //     bytesOut = 1;
        // }
        
        // WIRE_WRITE(*ptr++);
        // bytesOut++;
    }
    // wire->endTransmission();
    // } 
    // else { // SPI
    //     SSD1306_MODE_DATA
    //     while (count--)
    //     SPIwrite(*ptr++);
    // }
    // TRANSACTION_END
}


//TODO::: The following informaiton is for reference only 
//TODO::: see Adafruit_GFX.cpp line 1887
void fill_screen(uint16_t color) {
    uint16_t bytes = ((oled.screen_width + 7) / 8) * oled.screen_height;
    memset(oled.screen_buffer, color ? 0xFF : 0x00, bytes);
}

