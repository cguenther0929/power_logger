/*
 * oled_128b64.c
 *
 *  Created on: Aug 18, 2021
 *      Author: clint
 */

#include "oled_128b64.h"

struct oled     oled;   

void display_oled_init ( uint8_t voltage_state, uint8_t w, uint8_t h ) {

    oled.screen_width = w;
    oled.screen_height = h;
    
    /**
     * Call function to clear the display
     */
    // display_oled_clear();       //TODO need to define this function


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

void clearDisplay(void) {
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

    //TODO the following is for reference only, so okay to delete 
    // extern I2C_HandleTypeDef hi2c2
    // wire->beginTransmission(i2caddr);
    // WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
    // uint16_t bytesOut = 1;
    // while (n--) {
    //   if (bytesOut >= WIRE_MAX) {
    //     wire->endTransmission();
    //     wire->beginTransmission(i2caddr);
    //     WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
    //     bytesOut = 1;
    //   }
    //   WIRE_WRITE(pgm_read_byte(c++));
    //   bytesOut++;
    // }

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
  

    //TODO following can be deleted as it's for reference only 
    //   if (wire) { // I2C
    //     wire->beginTransmission(i2caddr);
    //     WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
    //     WIRE_WRITE(c);
    //     wire->endTransmission();
    //   } else { // SPI (hw or soft) -- transaction started in calling function
    //     SSD1306_MODE_COMMAND
    //     SPIwrite(c);
    //   }
  
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


//TODO this function was copied over, but needs to be cleaned up for
//TODO this application 
void ssd1306_drawChar(int16_t x, int16_t y, unsigned char c,
                            uint8_t size_x,
                            uint8_t size_y) {

    /** 
     * Character is assumed previously filtered by write() to eliminate
     * newlines, returns, non-printable characters, etc.  Calling
     * drawChar() directly with 'bad' characters of font may cause mayhem!
     * 
    */

    c -= (uint8_t)pgm_read_byte(&gfxFont->first);
    GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c);
    uint8_t *bitmap = pgm_read_bitmap_ptr(gfxFont);

    uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
    uint8_t w = pgm_read_byte(&glyph->width), h = pgm_read_byte(&glyph->height);
    int8_t xo = pgm_read_byte(&glyph->xOffset),
           yo = pgm_read_byte(&glyph->yOffset);
    uint8_t xx, yy, bits = 0, bit = 0;
    int16_t xo16 = 0, yo16 = 0;

    if (size_x > 1 || size_y > 1) {
      xo16 = xo;
      yo16 = yo;
    }

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
          bits = pgm_read_byte(&bitmap[bo++]);
        }
        if (bits & 0x80) {
          if (size_x == 1 && size_y == 1) {
            writePixel(x + xo + xx, y + yo + yy, color);
          } else {
            writeFillRect(x + (xo16 + xx) * size_x, y + (yo16 + yy) * size_y,
                          size_x, size_y, color);
          }
        }
        bits <<= 1;
      }
    }
    // endWrite();


}


void setFont(const GFXfont *f) {
//   if (f) {          // Font struct pointer passed in?
//     if (!gfxFont) { // And no current font struct?
      // Switching from classic to new font behavior.
      // Move cursor pos down 6 pixels so it's on baseline.
//       oled.cursor_y += 6;
//     }
//   } else if (gfxFont) { // NULL passed.  Current font struct defined?
    // Switching from new to classic (5x7) font behavior.
    // Move cursor pos up 6 pixels so it's at top-left of char.
    oled.cursor_y -= 6;
//   }
    old.gfxFont = (GFXfont *)f;
}
