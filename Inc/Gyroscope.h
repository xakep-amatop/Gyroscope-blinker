/**
  ******************************************************************************
  * @file     Gyroscope.h
  * @author   Mykola Kvach
  * @brief    This file contains functions to provide starting, getting
              measurement from gyroscope and orgazing LEDs blinking relatively
              gyro measurement.
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GYROSCOPE_H__
#define __GYROSCOPE_H__

/* Includes ------------------------------------------------------------------*/
#include <math.h>

#include "stm32f3_discovery.h"
#include "stm32f3_discovery_gyroscope.h"

/**
  * gyro_xyz 3 angular accelerations:
  * gyro_xyz[0] = X axis, gyro_xyz[1] = Y axis, gyro_xyz[2] = Z axis
*/
extern float gyro_xyz[3];

/* Gyroscope functions -------------------------------------------------------*/

/* Initialize gyroscope function */
void GYRO_Init(void);

/*  Calculate the gyroscope angular data, write it to gyro_xyz and organize LEDs
  * blinking relatively gyro measurement
*/
void GYRO_UpdateValues(void);

#endif /* __GYROSCOPE_H__ */
