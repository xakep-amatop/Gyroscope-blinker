#ifndef __UART_COMMAND_PARSER_H__
#define __UART_COMMAND_PARSER_H__

#include "main.h"
#include "UART_Interface.h"

#include <string.h>
#include <math.h>

#define GYROSCOPE_MODE    (0)
#define MAGNETOMETER_MODE (1)
#define JOYSTICK_MODE     (2)

typedef struct {
  uint8_t * name;
  uint32_t  length;
} uart_str;

extern volatile uint32_t current_mode;

void ParseCommand();

#endif