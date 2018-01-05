/**
  ******************************************************************************
  * @file   Accelerometer.h
  * @author Mykola Kvach
  * @brief  This file contains functions to provide starting and getting
            measurement from accelerometer.
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ACCELEROMETER_H__
#define __ACCELEROMETER_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f3_discovery_accelerometer.h"

/**
  * acc_xyz 3 angular accelerations:
  * acc_xyz[0] = X axis, acc_xyz[1] = Y axis, acc_xyz[2] = Z axis
  *
  * Array volatile because it changes in interruption.
*/
extern volatile float acc_xyz[3];

/* Acc functions -------------------------------------------------------------*/
/* Initialize accelerometer function */
void ACC_Init(void);

/* Get new measurement from accelerometer and write it to acc_xyz*/
void ACC_UpdateValues(void);

#endif /* __ACCELEROMETER_H__ */