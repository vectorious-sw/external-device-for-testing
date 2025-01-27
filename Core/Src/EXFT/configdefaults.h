
#include "config.h"
#pragma once


const configNvramConfigurationDb_t configDefaultConfigStruct = 
{  
	.NvramConfigImage.ConfigurationType                    = 1,
	.NvramConfigImage.ConfigurationVersionId               = 1,
	.NvramConfigImage.MeasurementModemLockDelay            = 600,
	.NvramConfigImage.VoltageHysteresisWidth               = 200,          // 200mV
	.NvramConfigImage.VbattUpperThreshold                  = 10000,        // 10V
	.NvramConfigImage.VbattLowerThreshold                  = 7000,         // 7V
	.NvramConfigImage.SleepModeTimer                       = 20,           // Go to sleep mode after 20 Seconds
	.NvramConfigImage.MeasurementSequence                  = 109,
	.NvramConfigImage.ManualRelayStateForSequence          = 0,
	.NvramConfigImage.MeasurementValidIcMax                = 84000,	// TODO: These numbers must be revised
	.NvramConfigImage.MeasurementValidIcMin                = 80000,
	.NvramConfigImage.MeasurementValidReffMax              = 84000,
	.NvramConfigImage.MeasurementValidReffMin              = 80000,
	.NvramConfigImage.MeasurementValidVmicTempMax          = 185000,
	.NvramConfigImage.MeasurementValidVmicTempMin          = 160000,
	.NvramConfigImage.MeasurementValidPressureCapTempMax   = 100000,
	.NvramConfigImage.MeasurementValidPressureCapTempMin   = 70000,
	.NvramConfigImage.MovementThreshold                    = 200,          // mG
	.NvramConfigImage.MeasurementManualPslForSequence      = 10,
	.NvramConfigImage.ProtocolRetryTimeout                 = 200,          // 10mS units
	.NvramConfigImage.ProtocolRetryCount                   = 5,
	.NvramConfigImage.MeasurementDelayStartMilliSeconds    = 0,         // Sec
	.NvramConfigImage.BuzzerControl                        = 1,            // Enable Buzzer
	.NvramConfigImage.vibratorControl                      = 1,            // Enable Vibrator
	.NvramConfigImage.AutoResonaceControl                  = 1,            // Enable Autoresonance
	.NvramConfigImage.AutoPowerControl                     = 1,            // Enable AutoPower
	.NvramConfigImage.MeasurementPslControl                = 1,
	.NvramConfigImage.PushButtonTurnOffTime                = 1,
	.NvramConfigImage.PatientNurseMode                     = 0,
	.NvramConfigImage.EnableMeasurmentWhileCharging        = 0,
	.NvramConfigImage.MeasurementMinimalPslMarginPercentage = 0,           // %
	.NvramConfigImage.MinimalSequencePsl                  = 10,           // 0 - accelerated PSL, other should be minimum 10
	.NvramConfigImage.MaximalSequencePsl                  = 140,           // %
	.NvramConfigImage.PslSequenceStepPercentage           = 1,           // %
	.NvramConfigImage.BeltSize                            = 2,           //
	.Crc32                                                 = 0,            // To be calculated.....
	.NvramConfigImage.NurseModePsl                         = 10,
	.NvramConfigImage.EnableChargingWhilePolling           = 0,
	.NvramConfigImage.EnableSleepWhilePolling              = 0,
	.NvramConfigImage.MeasurementMarginPercentage          = 0,
	.NvramConfigImage.MeasurementTimeoutSeconds            = 240
 };

const configNvramProductionDb_t configProductionDefaultStruct = 
{
  .NvramProductionImage.PressureSensorOffsetDelta = 0,
  .NvramProductionImage.UpStreamAesCbcKey =  { 
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa }, 

  .NvramProductionImage.DownStreamAesCbcKey = {
    0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 
    0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 
    0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc,
    0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc }, 
  .Crc32                                     = 0
};
