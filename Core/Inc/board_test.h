#ifndef BOARD_TEST_H
#define BOARD_TEST_H

#include "main.h"
#include <stdint.h>

void BoardTest_Init(void);
void BoardTest_Task(void);
void BoardTest_OnUsbRx(uint8_t *data, uint32_t len);
void BoardTest_UsbPullupInit(void);
void BoardTest_UsbDisconnect(void);
void BoardTest_UsbConnect(void);
void BoardTest_PwmInit(void);
void BoardTest_SetSpindlePwm(uint16_t permille);
void BoardTest_IdcInit(void);

#endif
