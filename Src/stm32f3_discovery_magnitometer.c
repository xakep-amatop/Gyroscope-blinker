/**
  ******************************************************************************
  * @file    stm32f3_discovery_magnetometer.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the MAGNETOMETER
  *          MEMS available on STM32F3-Discovery Kit.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f3_discovery_magnetometer.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F3_DISCOVERY
  * @{
  */

/** @defgroup STM32F3_DISCOVERY_MAGNETOMETER STM32F3-DISCOVERY MAGNETOMETER
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @defgroup STM32F3_DISCOVERY_MAGNETOMETER_Private_Types Private Types
  * @{
  */
/**
  * @}
  */

/* Private defines ------------------------------------------------------------*/
/** @defgroup STM32F3_DISCOVERY_MAGNETOMETER_Private_Constants Private Constants
  * @{
  */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup STM32F3_DISCOVERY_MAGNETOMETER_Private_Macros Private Macros
  * @{
  */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup STM32F3_DISCOVERY_MAGNETOMETER_Private_Variables Private Variables
  * @{
  */
static MAGNETO_DrvTypeDef * MagnetometerDrv;

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @addtogroup STM32F3_DISCOVERY_MAGNETOMETER_Private_FunctionPrototypes Private Functions
  * @{
  */
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @addtogroup STM32F3_DISCOVERY_MAGNETOMETER_Exported_Functions
  * @{
  */
uint8_t BSP_MAGNETO_Init(void){

  MAGNETO_InitTypeDef LSM303DLHC_InitStructure;

  uint8_t   ret   = MAGNETO_ERROR;

  if(Lsm303dlhcDrv_A.ReadID() == I_AM_LMS303DLHC){

    MagnetometerDrv = & Lsm303dlhcDrv_M;

    /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
    LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
    LSM303DLHC_InitStructure.MagOutput_DataRate = LSM303DLHC_ODR_400_HZ ;
    LSM303DLHC_InitStructure.MagFull_Scale      = LSM303DLHC_FS_1_3_GA;
    LSM303DLHC_InitStructure.Working_Mode       = LSM303DLHC_CONTINUOS_CONVERSION;

    uint32_t ctrl                               = LSM303DLHC_InitStructure.Temperature_Sensor | LSM303DLHC_InitStructure.MagOutput_DataRate;
    ctrl                                        = ctrl | (((uint32_t) (LSM303DLHC_InitStructure.MagFull_Scale)) <<  8);
    ctrl                                        = ctrl | (((uint32_t) (LSM303DLHC_InitStructure.Working_Mode))  << 16);

    MagnetometerDrv->Init(ctrl);

    ret = MAGNETO_OK;
  }
  else{
    ret = MAGNETO_ERROR;
  }

  return ret;
}

/**
  * @brief  Reboot memory content of MAGNETOMETER
  * @retval None
  */
void BSP_MAGNETO_Reset(void){
  if(MagnetometerDrv->Reset != NULL){
    MagnetometerDrv->Reset();
  }
}

/**
  * @brief  Get XYZ
  * @param pDataXYZ Pointeur on 3 angular
  *                 pDataXYZ[0] = X axis, pDataXYZ[1] = Y axis, pDataXYZ[2] = Z axis
* @retval None
*/
void BSP_MAGNETO_GetXYZ(volatile float * pDataXYZ){
  if(MagnetometerDrv->GetXYZ!= NULL){
    MagnetometerDrv->GetXYZ(pDataXYZ);
  }
}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/