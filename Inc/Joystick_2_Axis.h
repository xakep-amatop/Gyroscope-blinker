/**
  ******************************************************************************
  * @file   Joystick_2_Axis.h
  * @author Mykola Kvach
  * @brief  This file contains function prototypes to provide configuration and getting
            measurement from joystick. It the values of which are obtained from the ADC1.
            And defenition of Joystick2 struction and definitions of some gloabal
            structures and variables
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __JOYSTICK_2_AXIS_H__
#define __JOYSTICK_2_AXIS_H__

/* Includes ------------------------------------------------------------------*/
#include <math.h>

#include "main.h"
#include "UART_Interface.h"
#include "stm32f3_discovery.h"

typedef struct {
  uint32_t            max_x;
  uint32_t            min_x;
  uint32_t            mid_x;

  uint32_t            max_y;
  uint32_t            min_y;
  uint32_t            mid_y;

  uint32_t            eps_x;
  uint32_t            eps_y;

  int32_t             relative_x;
  int32_t             relative_y;

  volatile uint32_t   cur_xy[2];

  float               distance;
  float               intensity;
  float               angle;

} Joystick2;

extern    Joystick2           _joystick;
extern    ADC_HandleTypeDef   hadc1;
extern    UART_HandleTypeDef  huart4;
extern    uint8_t             tx_buff[];

extern uint32_t const         leds[];
extern const uint32_t         LED_NUM;

/* Joystick functions --------------------------------------------------------*/

// fill in some fields in structure of object "_joystick"
void JOYSTICK_Init();

/*  Calculate the joystick angular data, write it to _joystick and organize LEDs
  * blinking relatively joystick measurement
*/
void JOYSTICK_UpdateValues();

#endif /* __JOYSTICK_2_AXIS_H__ */