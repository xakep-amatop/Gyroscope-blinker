/**
  ******************************************************************************
  * @file    stm32f3_discovery_magnetometer.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for stm32f3_discovery_amgnetometer.c
  *          firmware driver.
  ******************************************************************************/

#ifndef __STM32F3_DISCOVERY_MAGNETO_H
#define __STM32F3_DISCOVERY_MAGNETO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3_discovery.h"
/* Include Gyroscope component driver */
#include "lsm303dlhc.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F3_DISCOVERY
  * @{
  */

/** @addtogroup STM32F3_DISCOVERY_MAGNETOMETER
  * @{
  */

/** @defgroup STM32F3_DISCOVERY_MAGNETOMETER_Exported_Types Exported Types
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_MAGNETOMETER_Exported_Constants Exported Constants
  * @{
  */
typedef enum {
  MAGNETO_OK      = 0,
  MAGNETO_ERROR   = 1,
  MAGNETO_TIMEOUT = 2
} MAGNETO_StatusTypeDef;

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_MAGNETOMETER_Exported_Macros Exported Macros
  * @{
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup STM32F3_DISCOVERY_MAGNETOMETER_Exported_Functions Exported Functions
  * @{
  */
/* Acc functions */
uint8_t   BSP_MAGNETO_Init  (void);
void      BSP_MAGNETO_Reset (void);
void      BSP_MAGNETO_GetXYZ(volatile float * pDataXYZ);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/