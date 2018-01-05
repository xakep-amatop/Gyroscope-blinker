#ifndef __MAGNETOMETER_H__
#define __MAGNETOMETER_H__

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "stm32f3_discovery.h"

#include "UART_Interface.h"

#include "stm32f3_discovery_magnetometer.h"
#include "Accelerometer.h"

extern volatile float magneto_xyz[3];

void MAGNETO_Init(void);
void MAGNETO_UpdateValues(void);

#endif