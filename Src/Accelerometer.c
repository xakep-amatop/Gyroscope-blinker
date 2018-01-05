/**
  ******************************************************************************
  * @file     Accelerometer.с
  * @author   Mykola Kvach
  * @brief    This file contains functions to provide starting and getting values
              from accelerometer.
  ******************************************************************************
  */

/* Includes -----------------------------------------------------------------------*/
#include "Accelerometer.h"

/* Glabal variables ---------------------------------------------------------------*/
volatile float acc_xyz[3];

/* Acc functions ------------------------------------------------------------------*/

/**
  * @brief  Init accelerometer MEMS
  * @note   If failed to initialize accelerometer, then go into the error handler function set by default
* @retval None
*/
void ACC_Init(void){

  if(BSP_ACCELERO_Init() != ACCELERO_OK){
    /* Initialization Error */
    Error_Handler();
  }

}

/**
  * @brief  Get XYZ acceleration, change values in global array acc_xyz
* @retval None
*/
void ACC_UpdateValues(void){

  if( BSP_ACCELERO_GetXYZ(acc_xyz) != ACCELERO_OK ){
    /* Initialization Error */
  }

}
