/**
******************************************************************************
* @file    sensor.h
* @author  MEMS Application Team
* @version V2.0.0
* @date    10-December-2015
* @brief   This header file contains the functions prototypes common for all
*          sensor drivers
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSOR_H
#define __SENSOR_H

  /* Includes ------------------------------------------------------------------*/

#include <string.h>

  
typedef enum { SENSOR_CONFIG_STATE0, SENSOR_CONFIG_STATE1, SENSOR_CONFIG_STATE2, SENSOR_CONFIG_STATE3, SENSOR_CONFIG_STATE4, SENSOR_CONFIG_STATE5} sensorConfigStateT;
typedef enum { SENSOR_AXIS_X, SENSOR_AXIS_Y, SENSOR_AXIS_Z} sensorAxisSelectT;

  /** @addtogroup BSP BSP
   * @{
   */

  /** @addtogroup COMPONENTS COMPONENTS
   * @{
   */

  /** @addtogroup COMMON COMMON
   * @{
   */

  /** @addtogroup SENSOR SENSOR
   * @{
   */

  /** @addtogroup SENSOR_Public_Types SENSOR Public types
   * @{
   */



  uint16_t sensorTemperatureGet();
  uint16_t sensorAbsolutePressureGet();
  int16_t sensorAccGet(sensorAxisSelectT axis);
  int16_t sensorAccP2PGet(sensorAxisSelectT axis);
  ReturnCode_T sensorStartCaptureP2PAcc(int16_t startAxisX, int16_t startAxisY, int16_t startAxisZ);
  ReturnCode_T sensorStopCaptureP2PAcc();
  ReturnCode_T sensorUpdateP2PValues(int16_t newAxisX, int16_t newAxisY, int16_t newAxisZ);
  
  float sensorPressureOPCSet(uint16_t PressureOPCValue, float PressureSensorOffsetLastDiff);
  ReturnCode_T sensorPressureOffsetSet(float PressureOffsetValue);
  
  int16_t sensorConvertToAccelParam(uint16_t msb, uint16_t lsb, float sensitivity);
  /**
   * @brief  Sensor axes raw data structure definition
   */
  typedef struct
  {
    int16_t AXIS_X;
    int16_t AXIS_Y;
    int16_t AXIS_Z;
  } SensorAxesRaw_t;

  /**
   * @brief  Sensor axes data structure definition
   */
  typedef struct
  {
    int32_t AXIS_X;
    int32_t AXIS_Y;
    int32_t AXIS_Z;
  } SensorAxes_t;



  /**
   * @brief  Sensor output data rate enumerator definition
   */
  typedef enum
    {
      ODR_LOW,
      ODR_MID_LOW,
      ODR_MID,
      ODR_MID_HIGH,
      ODR_HIGH
    } SensorOdr_t;



  /**
   * @brief  Sensor full scale enumerator definition
   */
  typedef enum
    {
      FS_LOW,
      FS_MID,
      FS_HIGH
    } SensorFs_t;

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

  /**
   * @}
   */

#endif /* __SENSOR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
