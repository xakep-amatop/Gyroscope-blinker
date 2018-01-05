#include "stm32f3xx_hal.h"

#ifndef __Fonts_h__
#define __Fonts_h__

//  Structure om font te definieren
typedef struct {
  const uint8_t      FontWidth;    /*!< Font width  in pixels */
  uint8_t            FontHeight;   /*!< Font height in pixels */
  const uint16_t *   data;         /*!< Pointer to data font data array */
} FontDef;

//  4 fonts
extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

#endif