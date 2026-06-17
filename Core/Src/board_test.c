#include "board_test.h"

#include "usbd_cdc_if.h"
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  const char *name;
} BoardPin;

typedef struct {
  GPIO_TypeDef *step_port;
  uint16_t step_pin;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  char axis;
} AxisPins;

#define USB_PULLUP_GPIO_Port GPIOC
#define USB_PULLUP_Pin GPIO_PIN_11
#define USB_PULLUP_ACTIVE_STATE GPIO_PIN_RESET
#define USB_PULLUP_INACTIVE_STATE GPIO_PIN_SET

static const BoardPin outputs[] = {
    {OUT1_GPIO_Port, OUT1_Pin, "OUT1"},
    {OUT2_GPIO_Port, OUT2_Pin, "OUT2"},
    {OUT3_GPIO_Port, OUT3_Pin, "OUT3"},
    {OUT4_GPIO_Port, OUT4_Pin, "OUT4"},
};

static const BoardPin inputs[] = {
    {X_LIMIT_GPIO_Port, X_LIMIT_Pin, "X_LIMIT/IN1"},
    {Y_LIMIT_GPIO_Port, Y_LIMIT_Pin, "Y_LIMIT/IN2"},
    {Z_LIMIT_GPIO_Port, Z_LIMIT_Pin, "Z_LIMIT/IN3"},
    {A_LIMIT_GPIO_Port, A_LIMIT_Pin, "A_LIMIT/IN4"},
    {GPIOB, GPIO_PIN_8, "IDC3/PB8"},
    {GPIOB, GPIO_PIN_9, "IDC4/PB9"},
    {GPIOB, GPIO_PIN_6, "IDC5/PB6"},
    {GPIOB, GPIO_PIN_7, "IDC6/PB7"},
    {GPIOB, GPIO_PIN_4, "IDC7/PB4"},
    {GPIOB, GPIO_PIN_3, "IDC8/PB3"},
    {GPIOD, GPIO_PIN_2, "IDC9/PD2"},
    {GPIOC, GPIO_PIN_12, "IDC10/PC12"},
};

static const AxisPins axes[] = {
    {X_STEP_GPIO_Port, X_STEP_Pin, X_DIR_GPIO_Port, X_DIR_Pin, 'x'},
    {Y_STEP_GPIO_Port, Y_STEP_Pin, Y_DIR_GPIO_Port, Y_DIR_Pin, 'y'},
    {Z_STEP_GPIO_Port, Z_STEP_Pin, Z_DIR_GPIO_Port, Z_DIR_Pin, 'z'},
    {A_STEP_GPIO_Port, A_STEP_Pin, A_DIR_GPIO_Port, A_DIR_Pin, 'a'},
};

static char rx_line[96];
static char command_line[96];
static uint32_t rx_len;
static volatile uint8_t command_ready;
static uint16_t spindle_pwm_permille;

static void usb_write(const char *buf, uint16_t len) {
  uint32_t start = HAL_GetTick();

  while (CDC_Transmit_FS((uint8_t *)buf, len) == USBD_BUSY) {
    if (HAL_GetTick() - start > 50U) {
      break;
    }
  }
}

static void usb_printf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n <= 0) {
    return;
  }
  if (n >= (int)sizeof(buf)) {
    n = sizeof(buf) - 1;
  }

  usb_write(buf, (uint16_t)n);
}

static const AxisPins *find_axis(char c) {
  c = (char)tolower((unsigned char)c);
  for (uint32_t i = 0; i < sizeof(axes) / sizeof(axes[0]); i++) {
    if (axes[i].axis == c) {
      return &axes[i];
    }
  }

  return NULL;
}

static void print_help(void) {
  usb_printf("\r\nMash3_grbl board test\r\n");
  usb_printf("? | s | led 0/1 | en 0/1 | out 1..4 0/1\r\n");
  usb_printf("dir x/y/z/a 0/1 | step x/y/z/a N | spindle 0/1\r\n");
  usb_printf("pwm 0..1000 | idc | outtest 1..4 | usb 0/1 | cycle\r\n\r\n");
}

static void print_status(void) {
  usb_printf("\r\nSTATUS\r\n");
  usb_printf("LED=%u EN=%u PWM=%u/1000 CCR1=%lu USB_PULLUP_PC11=%u\r\n",
             HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_SET,
             HAL_GPIO_ReadPin(BUFFER_ENABLE_GPIO_Port, BUFFER_ENABLE_Pin) == GPIO_PIN_SET,
             spindle_pwm_permille,
             (unsigned long)TIM1->CCR1,
             HAL_GPIO_ReadPin(USB_PULLUP_GPIO_Port, USB_PULLUP_Pin) == GPIO_PIN_SET);

  for (uint32_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++) {
    usb_printf("%s=%u ", outputs[i].name,
               HAL_GPIO_ReadPin(outputs[i].port, outputs[i].pin) == GPIO_PIN_SET);
  }
  usb_printf("\r\n");

  for (uint32_t i = 0; i < sizeof(inputs) / sizeof(inputs[0]); i++) {
    usb_printf("%-12s=%u\r\n", inputs[i].name,
               HAL_GPIO_ReadPin(inputs[i].port, inputs[i].pin) == GPIO_PIN_SET);
  }
  usb_printf("\r\n");
}

static void print_idc_status(void) {
  usb_printf("\r\nIDC/MPG connector, top view\r\n");
  usb_printf("1=+5V 2=GND\r\n");
  usb_printf("3 PB8  =%u\r\n", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET);
  usb_printf("4 PB9  =%u\r\n", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET);
  usb_printf("5 PB6  =%u\r\n", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET);
  usb_printf("6 PB7  =%u\r\n", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET);
  usb_printf("7 PB4  =%u\r\n", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET);
  usb_printf("8 PB3  =%u\r\n", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET);
  usb_printf("9 PD2  =%u\r\n", HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_SET);
  usb_printf("10 PC12=%u\r\n\r\n", HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET);
}

static void set_output(uint32_t out_no, uint32_t value) {
  if (out_no < 1 || out_no > 4) {
    usb_printf("ERR output must be 1..4\r\n");
    return;
  }

  const BoardPin *pin = &outputs[out_no - 1];
  HAL_GPIO_WritePin(pin->port, pin->pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
  usb_printf("OK %s=%lu\r\n", pin->name, (unsigned long)(value ? 1 : 0));
}

static void set_dir(char axis, uint32_t value) {
  const AxisPins *a = find_axis(axis);
  if (a == NULL) {
    usb_printf("ERR axis must be x,y,z,a\r\n");
    return;
  }

  HAL_GPIO_WritePin(a->dir_port, a->dir_pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
  usb_printf("OK DIR %c=%lu\r\n", a->axis, (unsigned long)(value ? 1 : 0));
}

static void step_axis(char axis, uint32_t count) {
  const AxisPins *a = find_axis(axis);
  if (a == NULL) {
    usb_printf("ERR axis must be x,y,z,a\r\n");
    return;
  }
  if (count > 10000U) {
    count = 10000U;
  }

  for (uint32_t i = 0; i < count; i++) {
    HAL_GPIO_WritePin(a->step_port, a->step_pin, GPIO_PIN_SET);
    for (volatile uint32_t d = 0; d < 600; d++) {
    }
    HAL_GPIO_WritePin(a->step_port, a->step_pin, GPIO_PIN_RESET);
    for (volatile uint32_t d = 0; d < 600; d++) {
    }
  }

  usb_printf("OK STEP %c %lu\r\n", a->axis, (unsigned long)count);
}

static void cycle_outputs(void) {
  for (uint32_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++) {
    HAL_GPIO_WritePin(outputs[i].port, outputs[i].pin, GPIO_PIN_SET);
    usb_printf("%s ON\r\n", outputs[i].name);
    HAL_Delay(300);
    HAL_GPIO_WritePin(outputs[i].port, outputs[i].pin, GPIO_PIN_RESET);
    usb_printf("%s OFF\r\n", outputs[i].name);
    HAL_Delay(100);
  }
  usb_printf("OK cycle\r\n");
}

static void test_output(uint32_t out_no) {
  if (out_no < 1 || out_no > 4) {
    usb_printf("ERR output must be 1..4\r\n");
    return;
  }

  const BoardPin *pin = &outputs[out_no - 1];
  usb_printf("Testing %s: GPIO HIGH/LOW 10 times\r\n", pin->name);

  for (uint32_t i = 0; i < 10; i++) {
    HAL_GPIO_WritePin(pin->port, pin->pin, GPIO_PIN_SET);
    usb_printf("%s GPIO=1\r\n", pin->name);
    HAL_Delay(500);
    HAL_GPIO_WritePin(pin->port, pin->pin, GPIO_PIN_RESET);
    usb_printf("%s GPIO=0\r\n", pin->name);
    HAL_Delay(500);
  }

  usb_printf("OK outtest %lu\r\n", (unsigned long)out_no);
}

void BoardTest_SetSpindlePwm(uint16_t permille) {
  if (permille > 1000U) {
    permille = 1000U;
  }

  spindle_pwm_permille = permille;
  TIM1->CCR1 = permille;
}

static void parse_line(char *line) {
  char *argv[4] = {0};
  uint32_t argc = 0;

  for (char *p = strtok(line, " \t\r\n"); p != NULL && argc < 4; p = strtok(NULL, " \t\r\n")) {
    argv[argc++] = p;
  }

  if (argc == 0) {
    return;
  }

  for (char *p = argv[0]; *p; p++) {
    *p = (char)tolower((unsigned char)*p);
  }

  if (strcmp(argv[0], "?") == 0 || strcmp(argv[0], "help") == 0) {
    print_help();
  } else if (strcmp(argv[0], "s") == 0 || strcmp(argv[0], "status") == 0) {
    print_status();
  } else if (strcmp(argv[0], "idc") == 0) {
    print_idc_status();
  } else if (strcmp(argv[0], "led") == 0 && argc >= 2) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, atoi(argv[1]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    usb_printf("OK LED=%d\r\n", atoi(argv[1]) ? 1 : 0);
  } else if (strcmp(argv[0], "en") == 0 && argc >= 2) {
    HAL_GPIO_WritePin(BUFFER_ENABLE_GPIO_Port, BUFFER_ENABLE_Pin, atoi(argv[1]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    usb_printf("OK EN=%d\r\n", atoi(argv[1]) ? 1 : 0);
  } else if (strcmp(argv[0], "out") == 0 && argc >= 3) {
    set_output((uint32_t)atoi(argv[1]), (uint32_t)atoi(argv[2]));
  } else if (strcmp(argv[0], "outtest") == 0 && argc >= 2) {
    test_output((uint32_t)atoi(argv[1]));
  } else if (strcmp(argv[0], "dir") == 0 && argc >= 3) {
    set_dir(argv[1][0], (uint32_t)atoi(argv[2]));
  } else if (strcmp(argv[0], "step") == 0 && argc >= 3) {
    step_axis(argv[1][0], (uint32_t)strtoul(argv[2], NULL, 10));
  } else if (strcmp(argv[0], "spindle") == 0 && argc >= 2) {
    BoardTest_SetSpindlePwm(atoi(argv[1]) ? 1000U : 0U);
    usb_printf("OK PWM=%u/1000\r\n", spindle_pwm_permille);
  } else if (strcmp(argv[0], "pwm") == 0 && argc >= 2) {
    BoardTest_SetSpindlePwm((uint16_t)strtoul(argv[1], NULL, 10));
    usb_printf("OK PWM=%u/1000\r\n", spindle_pwm_permille);
  } else if (strcmp(argv[0], "usb") == 0 && argc >= 2) {
    if (atoi(argv[1])) {
      BoardTest_UsbConnect();
      usb_printf("OK USB pullup connected\r\n");
    } else {
      usb_printf("OK USB pullup disconnecting\r\n");
      BoardTest_UsbDisconnect();
    }
  } else if (strcmp(argv[0], "cycle") == 0) {
    cycle_outputs();
  } else {
    usb_printf("ERR unknown command. Type ?\r\n");
  }
}

void BoardTest_Init(void) {
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUFFER_ENABLE_GPIO_Port, BUFFER_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_RESET);
  BoardTest_IdcInit();
  BoardTest_PwmInit();
}

void BoardTest_IdcInit(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void BoardTest_PwmInit(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();

  GPIO_InitStruct.Pin = SPINDLE_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPINDLE_PWM_GPIO_Port, &GPIO_InitStruct);

  TIM1->CR1 = 0;
  TIM1->CR2 = 0;
  TIM1->SMCR = 0;
  TIM1->DIER = 0;
  TIM1->CCER = 0;

  TIM1->PSC = 47U;
  TIM1->ARR = 999U;
  TIM1->CCR1 = 0;

  TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S);
  TIM1->CCMR1 |= (6U << 4) | TIM_CCMR1_OC1PE;
  TIM1->CCER |= TIM_CCER_CC1E;
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->CR1 |= TIM_CR1_ARPE;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 |= TIM_CR1_CEN;

  spindle_pwm_permille = 0;
}

void BoardTest_UsbPullupInit(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(USB_PULLUP_GPIO_Port, USB_PULLUP_Pin, USB_PULLUP_INACTIVE_STATE);

  GPIO_InitStruct.Pin = USB_PULLUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PULLUP_GPIO_Port, &GPIO_InitStruct);
}

void BoardTest_UsbDisconnect(void) {
  HAL_GPIO_WritePin(USB_PULLUP_GPIO_Port, USB_PULLUP_Pin, USB_PULLUP_INACTIVE_STATE);
}

void BoardTest_UsbConnect(void) {
  HAL_GPIO_WritePin(USB_PULLUP_GPIO_Port, USB_PULLUP_Pin, USB_PULLUP_ACTIVE_STATE);
}

void BoardTest_Task(void) {
  if (command_ready) {
    char line[sizeof(command_line)];

    __disable_irq();
    strncpy(line, command_line, sizeof(line));
    line[sizeof(line) - 1U] = '\0';
    command_ready = 0;
    __enable_irq();

    parse_line(line);
  }
}

void BoardTest_OnUsbRx(uint8_t *data, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    char c = (char)data[i];

    if (c == '\r' || c == '\n') {
      if (rx_len > 0 && !command_ready) {
        rx_line[rx_len] = '\0';
        strncpy(command_line, rx_line, sizeof(command_line));
        command_line[sizeof(command_line) - 1U] = '\0';
        command_ready = 1;
        rx_len = 0;
      }
    } else if (rx_len < sizeof(rx_line) - 1U) {
      rx_line[rx_len++] = c;
    } else {
      rx_len = 0;
    }
  }
}
