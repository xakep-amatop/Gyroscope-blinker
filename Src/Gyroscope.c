/**
  ******************************************************************************
  * @file     Gyroscope.c
  * @author   Mykola Kvach
  * @brief    This file contains functions to provide starting, getting
              measurement from gyroscope and orgazing LEDs blinking relatively
              gyro measurement.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "Gyroscope.h"

/* Glabal variables ----------------------------------------------------------*/
float                 gyro_xyz[3];

extern uint32_t const leds[];
extern const uint32_t LED_NUM;

extern UART_HandleTypeDef huart4;
extern uint8_t tx_buff[];
#include <string.h>

uint32_t tick1 = 0;

/* Gyroscope functions -------------------------------------------------------*/

/**
  * @brief  Init gyroscope MEMS
  * @note   If failed to initialize gyroscope, then go into the error handler function set by default
* @retval None
*/
void GYRO_Init(){

  if(BSP_GYRO_Init() != HAL_OK){
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  Calculate distance between current coordinates of point and O(0, 0)
  * @param  x coordinate
            y coordinate
* @retval Distance
*/
static inline float CalcDistance(float x, float y){
  return sqrt(x*x + y*y);
}

/**
  * @brief  Calculate the L3GD20 angular data, change values in global array
  *         gyro_xyz, and organize LEDs blinking relatively gyro measurement
* @retval None
*/
void GYRO_UpdateValues(void){

  static  uint16_t 	curr_led  = 0;
  uint8_t 			old_led   = curr_led;

  /* Read Gyro Angular data */
  BSP_GYRO_GetXYZ(gyro_xyz);

  /**
    * If the length of the resultant angular velocity vector along the X and Y axes
    * is less than the specified value, then we do not change the state of the LED panel
  */
  if (CalcDistance(gyro_xyz[0], gyro_xyz[1]) <
	  50000.f){ /* in fact, this coefficient should depend on L3GD20_FULLSCALE_ */
    return;
  }

  float angle  = atan2(gyro_xyz[1], gyro_xyz[0]) * 180. / M_PI;

  if (angle < 0.f){
    angle += 360.f;
  }

  /**
    * Since the matrix of LEDs "leds" was formed for the joystick
    * and the axis X is deviated from the direction of the gyro axis by 90 degrees,
    * so correct it.
  */
  angle += 90.f;

  curr_led = ( ( (uint32_t) ( angle / ( 180.f / LED_NUM ) ) + 1) >> 1 ) % LED_NUM;

  if( old_led != curr_led ){
    BSP_LED_Off( leds[old_led]  );
    BSP_LED_On ( leds[curr_led] );
  }

}
