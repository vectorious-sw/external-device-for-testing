
#pragma once

#include "sensor.h"
#include "lsm6ds3Regs.h"
#include "vlapConfig.h"

void lsm6ds3Config(void);
uint8_t lsm6ds3GetDeviceId(void);
void lsm6ds3_X_GetAxes(I2CREQ *i2cReq);



#if 0
uint8_t lsm6ds0_ACC_GYRO_W_BlockDataUpdate(LSM6DS0_ACC_GYRO_BDU_t newValue);
uint8_t lsm6ds0_ACC_GYRO_W_AccelerometerDataRate(LSM6DS0_ACC_GYRO_ODR_XL_t newValue);
uint8_t lsm6ds0_X_Set_FS(SensorFs_t fullScale);
uint8_t lsm6ds0_ACC_GYRO_W_AccelerometerFullScale(LSM6DS0_ACC_GYRO_FS_XL_t newValue);
uint8_t lsm6ds0_X_Set_Axes_Status(uint8_t *enable_xyz);
uint8_t lsm6ds0_ACC_GYRO_W_AccelerometerAxisX(LSM6DS0_ACC_GYRO_XEN_XL_t newValue);
uint8_t lsm6ds0_ACC_GYRO_W_AccelerometerAxisY(LSM6DS0_ACC_GYRO_YEN_XL_t newValue);
uint8_t lsm6ds0_ACC_GYRO_W_AccelerometerAxisZ(LSM6DS0_ACC_GYRO_ZEN_XL_t newValue);
uint8_t lsm6ds0_X_Set_ODR_Value_When_Enabled(float odr);
uint8_t lsm6ds0_X_Enable(void);
void lsm6ds0_X_GetAngles(float *pitch, float *roll);
#endif

