#ifndef XPT2046_H
#define XPT2046_H

#include "main.h"

typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint16_t z1;
  uint16_t z2;
} XPT2046_Point;

void XPT2046_Init(void);
uint8_t XPT2046_IsTouched(void);
XPT2046_Point XPT2046_GetRawPoint(void);
uint8_t XPT2046_RawToScreen(const XPT2046_Point *point,
                            uint16_t *screen_x, uint16_t *screen_y);
uint8_t XPT2046_GetScreenPoint(uint16_t *screen_x, uint16_t *screen_y);

#endif
