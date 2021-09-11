/**
  ******************************************************************************
  * @file           : font.h
  * @brief          : Header file for fonts
  *                     Thank you Adafruit!
  ******************************************************************************
  */
#ifndef INC_FONT_H_
#define INC_FONT_H_

#include <stdint.h>

/**
 * Font data stored PER GLYPH
 */
typedef struct _tgfxglyph {
  uint16_t bitmapOffset; ///< Pointer into GFXfont->bitmap
  uint8_t width;         ///< Bitmap dimensions in pixels
  uint8_t height;        ///< Bitmap dimensions in pixels
  uint8_t xAdvance;      ///< Distance to advance cursor (x axis)
  int8_t xOffset;        ///< X dist from cursor pos to UL corner
  int8_t yOffset;        ///< Y dist from cursor pos to UL corner
} GFXglyph;

/**
 * Data stored for FONT AS A WHOLE
 */
typedef struct _tgfxfont {
  uint8_t *bitmap;  ///< Glyph bitmaps, concatenated
  GFXglyph *glyph;  ///< Glyph array
  uint16_t first;   ///< ASCII extents (first char)
  uint16_t last;    ///< ASCII extents (last char)
  uint8_t yAdvance; ///< Newline distance (y axis)
} GFXfont;

extern GFXfont FreeSans9pt7b;

#endif /* INC_FONT_H_ */