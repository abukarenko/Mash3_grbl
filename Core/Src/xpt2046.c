#include "xpt2046.h"

#include <stddef.h>

#define TOUCH_CLK_GPIO_Port GPIOB
#define TOUCH_CLK_Pin GPIO_PIN_8
#define TOUCH_DIN_GPIO_Port GPIOB
#define TOUCH_DIN_Pin GPIO_PIN_9
#define TOUCH_DO_GPIO_Port GPIOB
#define TOUCH_DO_Pin GPIO_PIN_4
#define TOUCH_CS_GPIO_Port GPIOB
#define TOUCH_CS_Pin GPIO_PIN_3
#define TOUCH_IRQ_GPIO_Port GPIOC
#define TOUCH_IRQ_Pin GPIO_PIN_12

#define TFT_CS_GPIO_Port GPIOB
#define TFT_CS_Pin GPIO_PIN_6

#define TOUCH_SCREEN_WIDTH 480
#define TOUCH_SCREEN_HEIGHT 320
#define TOUCH_SAMPLE_COUNT 3U

/* Affine calibration from four screen targets. Coefficients are x1,000,000. */
#define TOUCH_CAL_SCALE 1000000L
#define TOUCH_X_CONSTANT 480264583L
#define TOUCH_X_FROM_RAW_X 921L
#define TOUCH_X_FROM_RAW_Y (-127892L)
#define TOUCH_Y_CONSTANT 307945740L
#define TOUCH_Y_FROM_RAW_X (-83199L)
#define TOUCH_Y_FROM_RAW_Y 1761L

static void xpt2046_clock_delay(void) {
  __NOP();
  __NOP();
  __NOP();
  __NOP();
}

static uint8_t xpt2046_transfer8(uint8_t value) {
  uint8_t received = 0U;

  for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
    HAL_GPIO_WritePin(TOUCH_DIN_GPIO_Port, TOUCH_DIN_Pin,
                      (value & mask) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    xpt2046_clock_delay();
    HAL_GPIO_WritePin(TOUCH_CLK_GPIO_Port, TOUCH_CLK_Pin, GPIO_PIN_SET);
    xpt2046_clock_delay();
    received <<= 1U;
    if (HAL_GPIO_ReadPin(TOUCH_DO_GPIO_Port, TOUCH_DO_Pin) == GPIO_PIN_SET) {
      received |= 1U;
    }
    HAL_GPIO_WritePin(TOUCH_CLK_GPIO_Port, TOUCH_CLK_Pin, GPIO_PIN_RESET);
  }

  return received;
}

static uint16_t xpt2046_read_adc(uint8_t command) {
  uint8_t high;
  uint8_t low;

  /* The TFT and touch controller share CLK, MOSI and MISO. */
  HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_RESET);
  (void)xpt2046_transfer8(command);
  high = xpt2046_transfer8(0U);
  low = xpt2046_transfer8(0U);
  HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);

  return (uint16_t)((((uint16_t)high << 8U) | low) >> 3U);
}

static uint16_t xpt2046_median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) {
    uint16_t temporary = a;
    a = b;
    b = temporary;
  }
  if (b > c) {
    uint16_t temporary = b;
    b = c;
    c = temporary;
  }
  if (a > b) {
    b = a;
  }
  return b;
}

void XPT2046_Init(void) {
  HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TOUCH_CLK_GPIO_Port, TOUCH_CLK_Pin, GPIO_PIN_RESET);
}

uint8_t XPT2046_IsTouched(void) {
  return HAL_GPIO_ReadPin(TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin) == GPIO_PIN_RESET;
}

XPT2046_Point XPT2046_GetRawPoint(void) {
  XPT2046_Point point = {0U, 0U, 0U, 0U, 0U};
  uint16_t x[TOUCH_SAMPLE_COUNT];
  uint16_t y[TOUCH_SAMPLE_COUNT];
  uint16_t z1[TOUCH_SAMPLE_COUNT];
  uint16_t z2[TOUCH_SAMPLE_COUNT];

  if (!XPT2046_IsTouched()) {
    return point;
  }

  for (uint8_t sample = 0U; sample < TOUCH_SAMPLE_COUNT; sample++) {
    x[sample] = xpt2046_read_adc(0xD0U);
    y[sample] = xpt2046_read_adc(0x90U);
    z1[sample] = xpt2046_read_adc(0xB0U);
    z2[sample] = xpt2046_read_adc(0xC0U);
  }

  point.x = xpt2046_median3(x[0], x[1], x[2]);
  point.y = xpt2046_median3(y[0], y[1], y[2]);
  point.z1 = xpt2046_median3(z1[0], z1[1], z1[2]);
  point.z2 = xpt2046_median3(z2[0], z2[1], z2[2]);
  point.z = XPT2046_IsTouched() ? 500U : 0U;
  return point;
}

uint8_t XPT2046_RawToScreen(const XPT2046_Point *point,
                            uint16_t *screen_x, uint16_t *screen_y) {
  int32_t x;
  int32_t y;

  if (point == NULL || point->z == 0U ||
      screen_x == NULL || screen_y == NULL) {
    return 0U;
  }

  x = (TOUCH_X_CONSTANT + TOUCH_X_FROM_RAW_X * (int32_t)point->x +
       TOUCH_X_FROM_RAW_Y * (int32_t)point->y) / TOUCH_CAL_SCALE;
  y = (TOUCH_Y_CONSTANT + TOUCH_Y_FROM_RAW_X * (int32_t)point->x +
       TOUCH_Y_FROM_RAW_Y * (int32_t)point->y) / TOUCH_CAL_SCALE;

  if (x < 0) {
    x = 0;
  } else if (x >= TOUCH_SCREEN_WIDTH) {
    x = TOUCH_SCREEN_WIDTH - 1;
  }
  if (y < 0) {
    y = 0;
  } else if (y >= TOUCH_SCREEN_HEIGHT) {
    y = TOUCH_SCREEN_HEIGHT - 1;
  }

  *screen_x = (uint16_t)x;
  *screen_y = (uint16_t)y;
  return 1U;
}

uint8_t XPT2046_GetScreenPoint(uint16_t *screen_x, uint16_t *screen_y) {
  XPT2046_Point point = XPT2046_GetRawPoint();
  return XPT2046_RawToScreen(&point, screen_x, screen_y);
}
