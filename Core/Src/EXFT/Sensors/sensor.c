#include <hwdrivers.h>
#include <stdlib.h>
#include <stdarg.h>
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "vlapConfig.h"
#include "queue.h"
#include "lps22hbRegs.h"
#include "pccommAppLayer.h"
#include "lsm6ds3Regs.h"
#include "sensor.h"
#include "config.h"
#include "spiflash.h"
#include <stdio.h>

#include "../common.h"
#include "i2cwork.h"



ReturnCode_T sensorPressureReqSetCallBack(uint8_t LedIndex, i2cworkSchedulerFunctionParameter_T* ReturnedCommandPtr);
ReturnCode_T sensorPressureReqCallBack(uint8_t LedIndex, i2cworkSchedulerFunctionParameter_T* ReturnedCommandPtr);
ReturnCode_T sensorPressurePostCallBack(uint8_t LedIndex, i2cworkSchedulerFunctionParameter_T* PostFunctionPtr);

sensorConfigStateT SensorConfigState;

// Lps22 Configuration Register values starting from LPS22HB_CTRL_REG1
uint8_t Lps22hbConfigArray[]={0x4e, 0x10, 0x00}; 

// LSM6DS3 Configuration Registers values starting from LSM6DS3_ACC_GYRO_CTRL1_XL
uint8_t Lsm6ds3ConfigArray1[]={0xa0, 0x80, 0x44};
// LSM6DS3 Configuration Registers values starting from LSM6DS3_ACC_GYRO_CTRL9_XL
uint8_t Lsm6ds3ConfigArray2[]={0x3c, 0x38};

uint8_t ReturnedI2cSensorBuffer[20];
uint8_t ReturnedI2cAccelGyroBuffer[20];
uint8_t sensorOffsetValue[2];

uint16_t sensorAbsolutePressure;
float sensorAbsolutePressureFloat;
uint16_t sensorTemperature;
uint16_t sensorExternalPressure = 0;

uint8_t sensoreCaptureAccP2P = 0;

int16_t sensorMaxAccXP2P;
int16_t sensorMaxAccYP2P;
int16_t sensorMaxAccZP2P;

int16_t sensorMinAccXP2P;
int16_t sensorMinAccYP2P;
int16_t sensorMinAccZP2P;

int16_t sensorAccXP2P;
int16_t sensorAccYP2P;
int16_t sensorAccZP2P;

int16_t sensorAccX;
int16_t sensorAccY;
int16_t sensorAccZ;

int16_t sensorGyroX;
int32_t sensorGyroY;
int32_t sensorGyroZ;

SemaphoreHandle_t sensorFirstMeasureSemaphoreHandle;

/******************************************************************************
* @brief  ReturnCode_T sensorPressureReqSchedule( )
* @param  
* @retval 
******************************************************************************/
ReturnCode_T sensorPressureReqSchedule( )
{
  ReturnCode_T MyReturnCode; 
  i2cworkSchedulerEntry_T MySchedulerEntry;
  
  // Create binary semaphore to initialize sensor
  sensorFirstMeasureSemaphoreHandle = xSemaphoreCreateBinary();
  
  SensorConfigState = SENSOR_CONFIG_STATE0;
  // Register I2C activity 
  MySchedulerEntry.Mode = I2CWORK_REQMODE_PERIODIC;
  MySchedulerEntry.workType = I2CWORK_OTHER_REQUEST;
  MySchedulerEntry.LedIndex = 0;
  MySchedulerEntry.Period = 10;
  MySchedulerEntry.I2cWorkerPreTranscationCallBackPtr = &sensorPressureReqCallBack;
  // For write operation we don't need the Post transaction callback
  MySchedulerEntry.I2cWorkerPostTranscationCallBackPtr = &sensorPressurePostCallBack;
  i2cworkSchedulerRegister( &MySchedulerEntry);
  // Success
  MyReturnCode = RETURNCODE_OK;
  // Return code
  return(MyReturnCode);
}
        
uint8_t counter = 0;

ReturnCode_T sensorPressureReqCallBack(uint8_t LedIndex, i2cworkSchedulerFunctionParameter_T* ReturnedCommandPtr)
{    
  switch( SensorConfigState)
  {
  case SENSOR_CONFIG_STATE0:
       ReturnedCommandPtr->Cmd = I2CWORK_TRANSACTION_CMD_WRITE;
       ReturnedCommandPtr->I2cSlaveAddress          = LPS22HB_ADDRESS;
       ReturnedCommandPtr->SlaveRegAddress          = LPS22HB_CTRL_REG1;
       ReturnedCommandPtr->TransactionDataPtr       = Lps22hbConfigArray;
       ReturnedCommandPtr->TranscationLength        = sizeof(Lps22hbConfigArray);
       SensorConfigState++;
       break;
    case SENSOR_CONFIG_STATE1:
       // Set the OPC after the pressure sensor done it's configuration
       sensorPressureOffsetSet(configProductionDb.PressureSensorOffsetDelta);
       ReturnedCommandPtr->Cmd = I2CWORK_TRANSACTION_CMD_WRITE;
       ReturnedCommandPtr->I2cSlaveAddress          = LSM6DS3_ACC_GYRO_I2C_ADDRESS_LOW;
       ReturnedCommandPtr->SlaveRegAddress          = LSM6DS3_ACC_GYRO_CTRL1_XL;
       ReturnedCommandPtr->TransactionDataPtr       = Lsm6ds3ConfigArray1;
       ReturnedCommandPtr->TranscationLength        = sizeof(Lsm6ds3ConfigArray1);
       SensorConfigState ++;
       break;
 
    case SENSOR_CONFIG_STATE2:
       ReturnedCommandPtr->Cmd = I2CWORK_TRANSACTION_CMD_WRITE;
       ReturnedCommandPtr->I2cSlaveAddress          = LSM6DS3_ACC_GYRO_I2C_ADDRESS_LOW;
       ReturnedCommandPtr->SlaveRegAddress          = LSM6DS3_ACC_GYRO_CTRL9_XL;
       ReturnedCommandPtr->TransactionDataPtr       = Lsm6ds3ConfigArray2;
       ReturnedCommandPtr->TranscationLength        = sizeof(Lsm6ds3ConfigArray2);
       SensorConfigState = SENSOR_CONFIG_STATE3 + 1;
       break;
      
       
     case SENSOR_CONFIG_STATE3:
       ReturnedCommandPtr->Cmd = I2CWORK_TRANSACTION_CMD_READ;
       ReturnedCommandPtr->I2cSlaveAddress          = LPS22HB_ADDRESS;
       ReturnedCommandPtr->SlaveRegAddress          = LPS22HB_PRESS_OUT_XL_REG;
       ReturnedCommandPtr->TransactionDataPtr       = ReturnedI2cSensorBuffer;
       ReturnedCommandPtr->TranscationLength        = 5;
       SensorConfigState++;
       break;
       
	  case SENSOR_CONFIG_STATE4:
		ReturnedCommandPtr->Cmd = I2CWORK_TRANSACTION_CMD_READ;
		ReturnedCommandPtr->I2cSlaveAddress          = LSM6DS3_ACC_GYRO_I2C_ADDRESS_LOW;
		ReturnedCommandPtr->SlaveRegAddress          = LSM6DS3_ACC_GYRO_OUTX_L_G;
		//ReturnedCommandPtr->SlaveRegAddress          = LSM6DS3_ACC_GYRO_OUTX_L_XL;
		//ReturnedCommandPtr->SlaveRegAddress          = LSM6DS3_ACC_GYRO_WHO_AM_I_REG;
		ReturnedCommandPtr->TransactionDataPtr       = ReturnedI2cAccelGyroBuffer;
		ReturnedCommandPtr->TranscationLength        = 12;
		//SensorConfigState = SENSOR_CONFIG_STATE4;
		SensorConfigState--;
		break;
	  default:
		  break;
  }
  return(RETURNCODE_OK);     
}

ReturnCode_T sensorPressureReqSetCallBack(uint8_t LedIndex, i2cworkSchedulerFunctionParameter_T* ReturnedCommandPtr)
{ 
  ReturnedCommandPtr->Cmd = I2CWORK_TRANSACTION_CMD_WRITE;
  ReturnedCommandPtr->I2cSlaveAddress          = LPS22HB_ADDRESS;
  ReturnedCommandPtr->SlaveRegAddress          = LPS22HB_RPDS_L;
  ReturnedCommandPtr->TransactionDataPtr       = sensorOffsetValue;
  ReturnedCommandPtr->TranscationLength        = sizeof(sensorOffsetValue);

  return(RETURNCODE_OK);     
}

ReturnCode_T sensorPressurePostCallBack(uint8_t LedIndex, i2cworkSchedulerFunctionParameter_T* PostFunctionPtr)
{
  float Pressure;
  uint32_t Temperature =0;
  //uint32_t Temperature =0;
  
  // Parse the Pressure
  Temperature =  ((uint32_t)ReturnedI2cSensorBuffer[0]) + ((uint32_t)ReturnedI2cSensorBuffer[1]<<8) + ((uint32_t)ReturnedI2cSensorBuffer[2]<<16);
  Pressure = (float)Temperature / (float)(4096*1.3332239);
  sensorAbsolutePressureFloat = Pressure;
  pccpmmAppLayerStruct.Board1SystemRegisters.ExternalPressure = TYPES_ENDIAN16_CHANGE(10*Pressure);
  sensorExternalPressure = (uint32_t)(10*Pressure);
  // Parse the temperature
  Temperature =  ((uint32_t)ReturnedI2cSensorBuffer[3]) + ((uint32_t)ReturnedI2cSensorBuffer[4]<<8);
  Temperature = (uint32_t)((float)Temperature /(float)(10));
  pccpmmAppLayerStruct.Board1SystemRegisters.ExternalTemperature = TYPES_ENDIAN16_CHANGE(Temperature);
  
  // Update the global variables
  sensorAbsolutePressure = Pressure;
  sensorTemperature = Temperature;
  
  // Calculate gyro axis
  sensorGyroX = sensorConvertToAccelParam(ReturnedI2cAccelGyroBuffer[1], ReturnedI2cAccelGyroBuffer[0], 4.375);
  sensorGyroY = sensorConvertToAccelParam(ReturnedI2cAccelGyroBuffer[3], ReturnedI2cAccelGyroBuffer[2], 4.375);  
  sensorGyroZ = sensorConvertToAccelParam(ReturnedI2cAccelGyroBuffer[5], ReturnedI2cAccelGyroBuffer[4], 4.375);
  // Calculate accelerometer axis
  sensorAccX = sensorConvertToAccelParam(ReturnedI2cAccelGyroBuffer[7], ReturnedI2cAccelGyroBuffer[6], 0.061);
  sensorAccY = sensorConvertToAccelParam(ReturnedI2cAccelGyroBuffer[9], ReturnedI2cAccelGyroBuffer[8], 0.061);  
  sensorAccZ = sensorConvertToAccelParam(ReturnedI2cAccelGyroBuffer[11], ReturnedI2cAccelGyroBuffer[10], 0.061);
  
  pccpmmAppLayerStruct.Board1SystemRegisters.AccX = TYPES_ENDIAN16_CHANGE(sensorAccX);
  pccpmmAppLayerStruct.Board1SystemRegisters.AccY = TYPES_ENDIAN16_CHANGE(sensorAccY);
  pccpmmAppLayerStruct.Board1SystemRegisters.AccZ = TYPES_ENDIAN16_CHANGE(sensorAccZ);
  
  // Check if we need to captur P2P
  if(sensoreCaptureAccP2P)
  {
    sensorUpdateP2PValues(sensorAccX, sensorAccY, sensorAccZ);
  }
  
  return(RETURNCODE_OK);     
}

int16_t sensorConvertToAccelParam(uint16_t msb, uint16_t lsb, float sensitivity)
{
  int16_t sum = ((((int8_t )msb ) << 8 ) + (int8_t )lsb);
  int16_t result = (int16_t)(sum * (float) sensitivity);
  return result;
}


float sensorPressureOPCSet(uint16_t PressureOPCValue, float PressureSensorOffsetLastDiff)
{  
  float pressureOPCValueFloat = (float) PressureOPCValue / (float) 10;
  float PressureOffsetValue = (float) pressureOPCValueFloat - sensorAbsolutePressureFloat + configProductionDb.PressureSensorOffsetDelta;
  sensorPressureOffsetSet(PressureOffsetValue);    
  //printf("new delta is: %.6f\n", PressureOffsetValue);
  return(PressureOffsetValue);
}

ReturnCode_T sensorPressureOffsetSet(float PressureOffsetValue)
{
  configProductionDb.PressureSensorOffsetDelta = (float) PressureOffsetValue;
  float OffsetCalc = (float) (-1 * PressureOffsetValue * 4096 * (float) 1.3332239 /(float) 256);
  int16_t OffsetCalcInt = (int16_t) OffsetCalc;
  sensorOffsetValue[1] = (OffsetCalcInt >> 8) & 0xFF;
  sensorOffsetValue[0] = OffsetCalcInt & 0xFF; 
  
  //sensorOffsetValue[1] = 0x00;
  //sensorOffsetValue[0] = 0x15;
  
  ReturnCode_T MyReturnCode; 
  i2cworkSchedulerEntry_T MySchedulerEntry;
  // Register I2C activity 
  MySchedulerEntry.Mode = I2CWORK_REQMODE_ONESHOT;
  MySchedulerEntry.workType = I2CWORK_OTHER_REQUEST;
  MySchedulerEntry.LedIndex = 0;
  MySchedulerEntry.Period = 10;
  MySchedulerEntry.I2cWorkerPreTranscationCallBackPtr = &sensorPressureReqSetCallBack;
  // For write operation we don't need the Post transcation callback  
  MySchedulerEntry.I2cWorkerPostTranscationCallBackPtr = 0;
  i2cworkSchedulerRegister( &MySchedulerEntry);
  // Success
  MyReturnCode = RETURNCODE_OK;
  // Return code
  return(MyReturnCode);
}

int16_t sensorAccGet(sensorAxisSelectT axis)
{
  switch(axis)
  {
  case SENSOR_AXIS_X:
    return sensorAccX;
  case SENSOR_AXIS_Y:
    return sensorAccY;
  case SENSOR_AXIS_Z:
    return sensorAccZ;   
  default:
    return 0;
  }
}

int16_t sensorAccP2PGet(sensorAxisSelectT axis)
{
  switch(axis)
  {
  case SENSOR_AXIS_X:
    return sensorAccXP2P;
  case SENSOR_AXIS_Y:
    return sensorAccYP2P;
  case SENSOR_AXIS_Z:
    return sensorAccZP2P;
  default:
    return 0;
  }
}

ReturnCode_T sensorStartCaptureP2PAcc(int16_t startAxisX, int16_t startAxisY, int16_t startAxisZ)
{
  sensoreCaptureAccP2P = 1;
  
  sensorAccXP2P = 0;
  sensorAccYP2P = 0;
  sensorAccZP2P = 0;
  
  // Initialize max acc P2P value
  sensorMaxAccXP2P = startAxisX;
  sensorMaxAccYP2P = startAxisY; 
  sensorMaxAccZP2P = startAxisZ;
  
  sensorMinAccXP2P = startAxisX;
  sensorMinAccYP2P = startAxisY; 
  sensorMinAccZP2P = startAxisZ;
  
  // Return code
  return(RETURNCODE_OK);
}

ReturnCode_T sensorStopCaptureP2PAcc()
{
  sensoreCaptureAccP2P = 0;
  
  // Return code
  return(RETURNCODE_OK);
}

ReturnCode_T sensorUpdateP2PValues(int16_t newAxisX, int16_t newAxisY, int16_t newAxisZ)
{
  // Check if max P2P value has changed
  if(newAxisX > sensorMaxAccXP2P)
    sensorMaxAccXP2P = newAxisX;
  if(newAxisY > sensorMaxAccYP2P)
    sensorMaxAccYP2P = newAxisY;
  if(newAxisZ > sensorMaxAccZP2P)
    sensorMaxAccZP2P = newAxisZ;
  
  // Check if min P2P value has changed
  if(newAxisX < sensorMinAccXP2P)
    sensorMinAccXP2P = newAxisX;
  if(newAxisY < sensorMinAccYP2P)
    sensorMinAccYP2P = newAxisY;
  if(newAxisZ < sensorMinAccZP2P)
    sensorMinAccZP2P = newAxisZ;
    
  // Update P2P
  sensorAccXP2P = sensorMaxAccXP2P - sensorMinAccXP2P;
  sensorAccYP2P = sensorMaxAccYP2P - sensorMinAccYP2P;
  sensorAccZP2P = sensorMaxAccZP2P - sensorMinAccZP2P;
  
  // Return code
  return(RETURNCODE_OK);
}

uint16_t sensorTemperatureGet()
{
  return(sensorTemperature);
}

uint16_t sensorAbsolutePressureGet()
{
  return((uint16_t)sensorExternalPressure);
}

