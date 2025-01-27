
#pragma once

/**
******************************************************************************
* @file    lps22hbRegs.h
* @author  MEMS Application Team
* @version V2.0.0
* @date    10-December-2015
* @brief   This file contains definitions for the LPS22HB_Driver.c firmware driver
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


#include <stdint.h>

/* Uncomment the line below to expanse the "assert_param" macro in the  drivers code */
#define USE_FULL_ASSERT_LPS22HB

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT_LPS22HB

/**
 * @brief  The assert_param macro is used for function's parameters check.
 * @param  expr: If expr is false, it calls assert_failed function which reports
 *         the name of the source file and the source line number of the call
 *         that failed. If expr is true, it returns no value.
 * @retval None
 */
#define LPS22HB_assert_param(expr) ((expr) ? (void)0 : LPS22HB_assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void LPS22HB_assert_failed(uint8_t* file, uint32_t line);
#else
#define LPS22HB_assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT_LPS22HB */


#ifdef __cplusplus
extern "C" {
#endif

  /** @addtogroup Environmental_Sensor
   * @{
   */

  /** @addtogroup LPS22HB_DRIVER
   * @{
   */

  /* Exported Types -------------------------------------------------------------*/
  /** @defgroup LPS22HB_Exported_Types
   * @{
   */

  /**
   * @brief  Error type.
   */
  typedef enum {LPS22HB_OK = (uint8_t)0, LPS22HB_ERROR = !LPS22HB_OK} LPS22HB_Error_et;

  /**
   * @brief  Enable/Disable type.
   */
  typedef enum {LPS22HB_DISABLE = (uint8_t)0, LPS22HB_ENABLE = !LPS22HB_DISABLE} LPS22HB_State_et;
#define IS_LPS22HB_State(MODE) ((MODE == LPS22HB_ENABLE) || (MODE == LPS22HB_DISABLE) )

  /**
   * @brief  Bit status type.
   */
  typedef enum {LPS22HB_RESET = (uint8_t)0, LPS22HB_SET = !LPS22HB_RESET} LPS22HB_BitStatus_et;
#define IS_LPS22HB_BitStatus(MODE) ((MODE == LPS22HB_RESET) || (MODE == LPS22HB_SET))

  /**
   * @brief  Pressure average.
   */
  typedef enum
    {
      LPS22HB_AVGP_8          = (uint8_t)0x00,         /*!< Internal average on 8 samples */
      LPS22HB_AVGP_32         = (uint8_t)0x01,         /*!< Internal average on 32 samples */
      LPS22HB_AVGP_128        = (uint8_t)0x02,         /*!< Internal average on 128 samples */
      LPS22HB_AVGP_512        = (uint8_t)0x03          /*!< Internal average on 512 sample */
    } LPS22HB_Avgp_et;
#define IS_LPS22HB_AVGP(AVGP) ((AVGP == LPS22HB_AVGP_8) || (AVGP == LPS22HB_AVGP_32) || \
                               (AVGP == LPS22HB_AVGP_128) || (AVGP == LPS22HB_AVGP_512))

  /**
   * @brief  Temperature average.
   */
  typedef enum
    {
      LPS22HB_AVGT_8          = (uint8_t)0x00,        /*!< Internal average on 8 samples */
      LPS22HB_AVGT_16         = (uint8_t)0x04,        /*!< Internal average on 16 samples */
      LPS22HB_AVGT_32         = (uint8_t)0x08,        /*!< Internal average on 32 samples */
      LPS22HB_AVGT_64         = (uint8_t)0x0C         /*!< Internal average on 64 samples */
    } LPS22HB_Avgt_et;

#define IS_LPS22HB_AVGT(AVGT) ((AVGT == LPS22HB_AVGT_8) || (AVGT == LPS22HB_AVGT_16) || \
                               (AVGT == LPS22HB_AVGT_32) || (AVGT == LPS22HB_AVGT_64))

typedef enum {

  LPS22HB_ODR_ONE_SHOT  = (uint8_t)0x00,         /*!< Output Data Rate: one shot */
  LPS22HB_ODR_1HZ       = (uint8_t)0x10,         /*!< Output Data Rate: 1Hz */
  LPS22HB_ODR_10HZ       = (uint8_t)0x20,         /*!< Output Data Rate: 10Hz */
  LPS22HB_ODR_25HZ    = (uint8_t)0x30,         /*!< Output Data Rate: 25Hz */
  LPS22HB_ODR_50HZ      = (uint8_t)0x40,          /*!< Output Data Rate: 50Hz */
  LPS22HB_ODR_75HZ      = (uint8_t)0x50          /*!< Output Data Rate: 75Hz */
} LPS22HB_Odr_et;

#define IS_LPS22HB_ODR(ODR) ((ODR == LPS22HB_ODR_ONE_SHOT) || (ODR == LPS22HB_ODR_1HZ) || \
(ODR == LPS22HB_ODR_10HZ) || (ODR == LPS22HB_ODR_25HZ)|| (ODR == LPS22HB_ODR_50HZ) || (ODR == LPS22HB_ODR_75HZ))

  /**
   * @brief  LPS22HB Spi Mode configuration.
   */
  typedef enum
    {
      LPS22HB_SPI_4_WIRE   =  (uint8_t)0x00,
      LPS22HB_SPI_3_WIRE   =  (uint8_t)0x01
    } LPS22HB_SPIMode_et;

#define IS_LPS22HB_SPIMode(MODE) ((MODE == LPS22HB_SPI_4_WIRE) || (MODE == LPS22HB_SPI_3_WIRE))

  /**
   * @brief  Block data update.
   */

  typedef enum
    {
      LPS22HB_BDU_CONTINUOUS_UPDATE     =  (uint8_t)0x00,  /*!< Data updated continuously */
      LPS22HB_BDU_NO_UPDATE             =  (uint8_t)0x04   /*!< Data updated after a read operation */
    } LPS22HB_Bdu_et;
#define IS_LPS22HB_BDUMode(MODE) ((MODE == LPS22HB_BDU_CONTINUOUS_UPDATE) || (MODE == LPS22HB_BDU_NO_UPDATE))

  /**
   * @brief  Data Signal on INT1 pad control bits.
   */
  typedef enum
    {
      LPS22HB_DATA = (uint8_t)0x00,
      LPS22HB_P_HIGH = (uint8_t)0x01,
      LPS22HB_P_LOW = (uint8_t)0x02,
      LPS22HB_P_LOW_HIGH = (uint8_t)0x03
    } LPS22HB_OutputSignalConfig_et;
#define IS_LPS22HB_OutputSignal(MODE) ((MODE == LPS22HB_DATA) || (MODE == LPS22HB_P_HIGH)|| \
                                       (MODE == LPS22HB_P_LOW) || (MODE == LPS22HB_P_LOW_HIGH))

  /**
   * @brief  LPS22HB INT1 Interrupt pins configuration.
   */
  typedef enum
    {
      LPS22HB_DISABLE_DATA_INT = (uint8_t)0x00,
      LPS22HB_DATA_READY = (uint8_t)0x01,
      LPS22HB_OVR = (uint8_t)0x02,
      LPS22HB_WTM = (uint8_t)0x04,
      LPS22HB_EMPTY      = (uint8_t)0x08
    } LPS22HB_DataSignalType_et;
#define IS_LPS22HB_DataSignal(MODE) ((MODE == LPS22HB_DISABLE_DATA_INT) ||(MODE == LPS22HB_EMPTY) || (MODE ==LPS22HB_WTM)|| \
                                     (MODE == LPS22HB_OVR) || (MODE == LPS22HB_DATA_READY))

  /**
   * @brief  LPS22HB Push-pull/Open Drain selection on Interrupt pads.
   */
  typedef enum
    {
      LPS22HB_PushPull = (uint8_t)0x00,
      LPS22HB_OpenDrain  = (uint8_t)0x40
    } LPS22HB_OutputType_et;
#define IS_LPS22HB_OutputType(MODE) ((MODE == LPS22HB_PushPull) || (MODE == LPS22HB_OpenDrain))

  /**
   * @brief  LPS22HB Interrupt Differential Configuration.
   */
  typedef enum
    {
      LPS22HB_DISABLE_INT = (uint8_t)0x00,
      LPS22HB_PHE = (uint8_t)0x01,
      LPS22HB_PLE = (uint8_t)0x02,
      LPS22HB_PLE_PHE = (uint8_t)0x03
    } LPS22HB_InterruptDiffConfig_et;
#define IS_LPS22HB_InterruptDiff(MODE) ((MODE == LPS22HB_DISABLE_INT) || (MODE ==LPS22HB_PHE)|| \
                                        (MODE == LPS22HB_PLE) || (MODE == LPS22HB_PLE_PHE))

  /**
   * @brief  LPS22HB Interrupt Differential Status.
   */

  typedef struct
  {
    uint8_t PH;          /*!< High Differential Pressure event occured */
    uint8_t PL;          /*!< Low Differential Pressure event occured */
    uint8_t IA;          /*!< One or more interrupt events have been  generated.Interrupt Active */
  } LPS22HB_InterruptDiffStatus_st;


  /**
   * @brief  LPS22HB Pressure and Temperature data status.
   */
  typedef struct
  {
    uint8_t TempDataAvailable;           /*!< Temperature data available bit */
    uint8_t PressDataAvailable;          /*!< Pressure data available bit */
    uint8_t TempDataOverrun;             /*!< Temperature data over-run bit */
    uint8_t PressDataOverrun;            /*!< Pressure data over-run bit */
  } LPS22HB_DataStatus_st;


  /**
   * @brief  LPS22HB Fifo Mode.
   */

  typedef enum
    {
      LPS22HB_FIFO_BYPASS_MODE                   = (uint8_t)0x00,   /*!< The FIFO is disabled and empty. The pressure is read directly*/
      LPS22HB_FIFO_MODE                           = (uint8_t)0x20,    /*!< Stops collecting data when full */
      LPS22HB_FIFO_STREAM_MODE                    = (uint8_t)0x40,    /*!< Keep the newest measurements in the FIFO*/
      LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE      = (uint8_t)0x60,    /*!< STREAM MODE until trigger deasserted, then change to FIFO MODE*/
      LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE    = (uint8_t)0x80,    /*!< BYPASS MODE until trigger deasserted, then STREAM MODE*/
      LPS22HB_FIFO_MEAN_MODE                      = (uint8_t)0xC0,    /*!< FIFO is used to generate a running average filtered pressure */
      LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE      = (uint8_t)0xE0     /*!< BYPASS mode until trigger deasserted, then FIFO MODE*/
    } LPS22HB_FifoMode_et;

#define IS_LPS22HB_FifoMode(MODE) ((MODE == LPS22HB_FIFO_BYPASS_MODE) || (MODE ==LPS22HB_FIFO_MODE)|| \
                                   (MODE == LPS22HB_FIFO_STREAM_MODE) || (MODE == LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE)|| \
                                   (MODE == LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE) || (MODE == LPS22HB_FIFO_MEAN_MODE)|| \
                                   (MODE == LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE))


  /**
   * @brief  LPS22HB Fifo Mean Mode Sample Size.
   */

  typedef enum
    {
      LPS22HB_FIFO_SAMPLE_2    = (uint8_t)0x01,     /*!< 2 Fifo Mean Mode samples */
      LPS22HB_FIFO_SAMPLE_4    = (uint8_t)0x03,     /*!< 4 Fifo Mean Mode samples */
      LPS22HB_FIFO_SAMPLE_8    = (uint8_t)0x07,     /*!< 8 Fifo Mean Mode samples */
      LPS22HB_FIFO_SAMPLE_16   = (uint8_t)0x0F,     /*!< 16 Fifo Mean Mode samples */
      LPS22HB_FIFO_SAMPLE_32   = (uint8_t)0x1F      /*!< 32 Fifo Mean Mode samples */
    } LPS22HB_FifoMeanModeSample_et;

#define IS_LPS22HB_FifoMeanModeSample(MODE) ((MODE == LPS22HB_FIFO_SAMPLE_2) || (MODE ==LPS22HB_FIFO_SAMPLE_4)|| \
                                             (MODE == LPS22HB_FIFO_SAMPLE_8) || (MODE == LPS22HB_FIFO_SAMPLE_16)|| \
                                             (MODE == LPS22HB_FIFO_SAMPLE_32))
  /**
   * @brief  LPS22HB Fifo Satus.
   */
  typedef struct
  {
    uint8_t FIFO_LEVEL;          /*!< FIFO Stored data level t */
    uint8_t FIFO_EMPTY;          /*!< Empy FIFO bit.1 FIFO is empty */
    uint8_t FIFO_FULL;           /*!< Overrun bit status. 1 FIFO is full */
    uint8_t FIFO_WTM;            /*!< Watermark Status. 1 FIFO is equal or higher then wtm level.*/
  } LPS22HB_FifoStatus_st;


  /**
   * @brief  LPS22HB Configuration structure definition.
   */
  typedef struct
  {
    LPS22HB_Avgp_et         PressResolution;          /*!< Pressure sensor resolution */
    LPS22HB_Avgt_et         TempResolution;           /*!< Temperature sensor resolution */
    LPS22HB_Odr_et          OutputDataRate;           /*!< Output Data Rate */
    LPS22HB_Bdu_et
    BDU;                      /*!< Enable to inhibit the output registers update between the reading of upper and lower register parts.*/
    LPS22HB_State_et
    AutoZero;                 /* < Auto zero feature enabled.the actual pressure output is copied in the REF_P*/
    LPS22HB_State_et
    Reset_AZ;                 /*!< Reset the Auto ZeroFunc. The daefualt RPDS value is copied in REF_P   */
    LPS22HB_SPIMode_et    Sim;                      /*!< SPI Serial Interface Mode selection */
  } LPS22HB_ConfigTypeDef_st;



  /**
   * @brief  LPS22HB Interrupt structure definition .
   */
  typedef struct
  {
    LPS22HB_State_et        INT_H_L;                /*!< Interrupt active high, low. Default value: 0 */
    LPS22HB_OutputType_et
    PP_OD;            /*!< Push-pull/open drain selection on interrupt pads. Default value: 0 */
    LPS22HB_OutputSignalConfig_et
    OutputSignal_INT1;  /*!< Data signal on INT1 Pad: Data,Pressure High, Preessure Low,P High or Low*/
    LPS22HB_DataSignalType_et           DataInterrupt_INT1; /*!< Interrupt Pin Config:DRDY, WTM, OVERRUN, EMPTY*/
    LPS22HB_InterruptDiffConfig_et    PressureInterrupt;      /*!< Interrupt differential Configuration: PL_E and/or PH_E*/
    LPS22HB_State_et        LatchIRQ;   /*!< Latch Interrupt request in to INT_SOURCE reg*/
    int16_t           fP_threshold;   /*!< Threshold value for pressure interrupt generation*/
  } LPS22HB_InterruptTypeDef_st;

  /**
   * @brief  LPS22HB FIFO structure definition.
   */
  typedef struct
  {
    LPS22HB_FifoMode_et       FIFO_MODE;               /*!< Fifo Mode Selection */
    LPS22HB_State_et      WTM_INT;    /*!< Enable/Disable the watermark interrupt*/
    LPS22HB_State_et      FIFO_MEAN_DEC;    /*!< Enable/Disable 1 Hz ODR decimation */
    LPS22HB_FifoMeanModeSample_et     MEAN_MODE_SAMPLE; /*!< FIFO Mean Mode Sample Size*/
    uint8_t         WTM_LEVEL;    /*!< FIFO threshold.Watermark level setting*/
  } LPS22HB_FIFOTypeDef_st;

#define IS_LPS22HB_WtmLevel(LEVEL) ((LEVEL > 0) && (LEVEL <=31))

  /**
   * @brief  LPS22HB Measure Type definition.
   */
  typedef struct
  {
    int16_t Tout;
    int32_t Pout;
  } LPS22HB_MeasureTypeDef_st;


  /**
   * @brief  LPS22HB Driver Version Info structure definition.
   */
  typedef struct
  {
    uint8_t   Major;
    uint8_t   Minor;
    uint8_t Point;
  } LPS22HB_DriverVersion_st;


  /**
   * @brief  Bitfield positioning.
   */
#define LPS22HB_BIT(x) ((uint8_t)x)

  /**
   * @brief  I2C address.
   */
#define LPS22HB_ADDRESS  (uint8_t)0xB8 

  /**
   * @brief  Set the LPS22HB driver version.
   */
#define LPS22HB_DriverVersion_Major (uint8_t)1
#define LPS22HB_DriverVersion_Minor (uint8_t)2
#define LPS22HB_DriverVersion_Point (uint8_t)5

  /**
   * @}
   */


  /* Exported Constants ---------------------------------------------------------*/
  /** @defgroup LPS22HB_Exported_Constants
   * @{
   */


  /**
   * @addtogroup LPS22HB_Register
   * @{
   */



  /**
   * @brief Device Identification register.
   * \code
   * Read
   * Default value: 0xBD
   * 7:0 This read-only register contains the device identifier that, for LPS22HB, is set to BDh.
   * \endcode
   */

#define LPS22HB_WHO_AM_I_REG         (uint8_t)0x0F

  /**
   * @brief Device Identification value.
   */
#define LPS22HB_WHO_AM_I_VAL         (uint8_t)0xB1

#define LPS22HB_CTRL_REG1      (uint8_t)0x10

#define LPS22HB_ODR_MASK                (uint8_t)0x70
#define LPS22HB_LPFP_MASK               (uint8_t)0x08
#define LPS22HB_LPFP_CUTOFF_MASK        (uint8_t)0x04
#define LPS22HB_BDU_MASK                (uint8_t)0x02
#define LPS22HB_SIM_MASK                (uint8_t)0x01

#define LPS22HB_LPFP_BIT    LPS22HB_BIT(3)

#define LPS22HB_CTRL_REG2      (uint8_t)0x11
#define LPS22HB_RPDS_L         (uint8_t)0x18
#define LPS22HB_RPDS_H         (uint8_t)0x19
  
#define LPS22HB_BOOT_BIT       LPS22HB_BIT(7)
#define LPS22HB_FIFO_EN_BIT    LPS22HB_BIT(6)
#define LPS22HB_WTM_EN_BIT     LPS22HB_BIT(5)
#define LPS22HB_ADD_INC_BIT    LPS22HB_BIT(4)
#define LPS22HB_I2C_BIT        LPS22HB_BIT(3)
#define LPS22HB_SW_RESET_BIT   LPS22HB_BIT(2)

#define LPS22HB_FIFO_EN_MASK   (uint8_t)0x40
#define LPS22HB_WTM_EN_MASK    (uint8_t)0x20
#define LPS22HB_ADD_INC_MASK   (uint8_t)0x10
#define LPS22HB_I2C_MASK       (uint8_t)0x08
#define LPS22HB_ONE_SHOT_MASK  (uint8_t)0x01


  /**
   * @brief Interrupt Differential configuration Register
   * \code
   * Read/write
   * Default value: 0x00.
   * 7:3 Reserved.
   * 2 LIR: Latch Interrupt request into INT_SOURCE register. 0 - interrupt request not latched; 1 - interrupt request latched
   * 1 PL_E: Enable interrupt generation on differential pressure low event. 0 - disable; 1 - enable
   * 0 PH_E: Enable interrupt generation on differential pressure high event. 0 - disable; 1 - enable
   * \endcode
   */
#define LPS22HB_INTERRUPT_CFG_REG  (uint8_t)0x24
#define LPS22HB_LIR_BIT            LPS22HB_BIT(2)
#define LPS22HB_PL_E_BIT           LPS22HB_BIT(1)
#define LPS22HB_PH_E_BIT           LPS22HB_BIT(0)

#define LPS22HB_LIR_MASK           (uint8_t)0x04
#define LPS22HB_PL_E_MASK          (uint8_t)0x02
#define LPS22HB_PH_E_MASK          (uint8_t)0x01
#define LPS22HB_PE_MASK          (uint8_t)0x03


  /**
   * @brief Interrupt source Register (It is cleared by reading it)
   * \code
   * Read
   * Default value: 0x00.
   * 7:3 Reserved: Keep these bits at 0
   * 2 IA: Interrupt Active.0: no interrupt has been generated; 1: one or more interrupt events have been generated.
   * 1 PL: Differential pressure Low. 0: no interrupt has been generated; 1: Low differential pressure event has occurred.
   * 0 PH: Differential pressure High. 0: no interrupt has been generated; 1: High differential pressure event has occurred.
   * \endcode
   */
#define LPS22HB_INTERRUPT_SOURCE_REG   (uint8_t)0x22
#define LPS22HB_IA_BIT                 LPS22HB_BIT(2)
#define LPS22HB_PL_BIT                 LPS22HB_BIT(1)
#define LPS22HB_PH_BIT                 LPS22HB_BIT(0)

#define LPS22HB_IA_MASK               (uint8_t)0x04
#define LPS22HB_PL_MASK               (uint8_t)0x02
#define LPS22HB_PH_MASK               (uint8_t)0x01

  /**
   * @brief  Status Register
   * \code
   * Read
   * Default value: 0x00
   * 7:6 Reserved: 0
   * 5 P_OR: Pressure data overrun. 0: no overrun has occurred; 1: new data for pressure has overwritten the previous one.
   * 4 T_OR: Temperature data overrun. 0: no overrun has occurred; 1: a new data for temperature has overwritten the previous one.
   * 3:2 Reserved: 0
   * 1 P_DA: Pressure data available. 0: new data for pressure is not yet available; 1: new data for pressure is available.
   * 0 T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
   * \endcode
   */
#define LPS22HB_STATUS_REG         (uint8_t)0x27
#define LPS22HB_POR_BIT            LPS22HB_BIT(5)
#define LPS22HB_TOR_BIT            LPS22HB_BIT(4)
#define LPS22HB_PDA_BIT            LPS22HB_BIT(1)
#define LPS22HB_TDA_BIT            LPS22HB_BIT(0)

#define LPS22HB_POR_MASK           (uint8_t)0x20
#define LPS22HB_TOR_MASK           (uint8_t)0x10
#define LPS22HB_PDA_MASK           (uint8_t)0x02
#define LPS22HB_TDA_MASK           (uint8_t)0x01

  /**
   * @brief  Pressure data (LSB) register.
   * \code
   * Read
   * Default value: 0x00.(To be verified)
   * POUT7 - POUT0: Pressure data LSB (2's complement).
   * Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
   * PRESS_OUT_XL)[dec]/4096.
   * \endcode
   */

#define LPS22HB_PRESS_OUT_XL_REG        (uint8_t)0x28
  /**
   * @brief  Pressure data (Middle part) register.
   * \code
   * Read
   * Default value: 0x80.
   * POUT15 - POUT8: Pressure data middle part (2's complement).
   * Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
   * PRESS_OUT_XL)[dec]/4096.
   * \endcode
   */
#define LPS22HB_PRESS_OUT_L_REG        (uint8_t)0x29

  /**
   * @brief  Pressure data (MSB) register.
   * \code
   * Read
   * Default value: 0x2F.
   * POUT23 - POUT16: Pressure data MSB (2's complement).
   * Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
   * PRESS_OUT_XL)[dec]/4096.
   * \endcode
   */
#define LPS22HB_PRESS_OUT_H_REG        (uint8_t)0x2A

  /**
   * @brief  Temperature data (LSB) register.
   * \code
   * Read
   * Default value: 0x00.
   * TOUT7 - TOUT0: temperature data LSB.
   * T(degC) = 42.5 + (Temp_OUTH & TEMP_OUT_L)[dec]/480.
   * \endcode
   */
#define LPS22HB_TEMP_OUT_L_REG         (uint8_t)0x2B

  /**
   * @brief  Temperature data (MSB) register.
   * \code
   * Read
   * Default value: 0x00.
   * TOUT15 - TOUT8: temperature data MSB.
   * T(degC) = 42.5 + (Temp_OUTH & TEMP_OUT_L)[dec]/480.
   * \endcode
   */
#define LPS22HB_TEMP_OUT_H_REG         (uint8_t)0x2C

  /**
   * @brief Threshold pressure (LSB) register.
   * \code
   * Read/write
   * Default value: 0x00.
   * 7:0 THS7-THS0: LSB Threshold pressure Low part of threshold value for pressure interrupt
   * generation. The complete threshold value is given by THS_P_H & THS_P_L and is
   * expressed as unsigned number. P_ths(hPA)=(THS_P_H & THS_P_L)[dec]/16.
   * \endcode
   */
#define LPS22HB_THS_P_LOW_REG           (uint8_t)0x30

  /**
   * @brief Threshold pressure (MSB)
   * \code
   * Read/write
   * Default value: 0x00.
   * 7:0 THS15-THS8: MSB Threshold pressure. High part of threshold value for pressure interrupt
   * generation. The complete threshold value is given by THS_P_H & THS_P_L and is
   * expressed as unsigned number. P_ths(mbar)=(THS_P_H & THS_P_L)[dec]/16.
   * \endcode
   */
#define LPS22HB_THS_P_HIGH_REG         (uint8_t)0x31

  /**
   * @brief FIFO control register
   * \code
   * Read/write
   * Default value: 0x00
   * 7:5 F_MODE2, F_MODE1, F_MODE0: FIFO mode selection.
   *     FM2   | FM1   | FM0   |    FIFO MODE
   *   ---------------------------------------------------
   *      0    |  0    |  0    | BYPASS MODE
   *      0    |  0    |  1    | FIFO MODE. Stops collecting data when full
   *      0    |  1    |  0    | STREAM MODE: Keep the newest measurements in the FIFO
   *      0    |  1    |  1    | STREAM MODE until trigger deasserted, then change to FIFO MODE
   *      1    |  0    |  0    | BYPASS MODE until trigger deasserted, then STREAM MODE
   *      1    |  0    |  1    | Reserved for future use
   *      1    |  1    |  0    | FIFO_MEAN MODE: Fifo is used to generate a running average filtered pressure
   *      1    |  1    |  1    | BYPASS mode until trigger deasserted, then FIFO MODE
   *
   * 4:0 WTM_POINT4-0 : FIFO threshold.Watermark level setting (0-31)
   * 4:0 WTM_POINT4-0 is FIFO Mean Mode Sample size if FIFO_MEAN MODE is used
   *     WTM_POINT4 | WTM_POINT4 | WTM_POINT4 |  WTM_POINT4 | WTM_POINT4 | Sample Size
   *   ----------------------------------------------------------------------------------
   *      0         |    0       |    0       |      0      |     1      |       2
   *      0         |    0       |    0       |      1      |     1      |       4
   *      0         |    0       |    1       |      1      |     1      |       8
   *      0         |    1       |    1       |      1      |     1      |       16
   *      1         |    1       |    1       |      1      |     1      |       32
   * other values operation not guaranteed
   * \endcode
   */
#define LPS22HB_CTRL_FIFO_REG          (uint8_t)0x2E
#define LPS22HB_FMODE_BIT              LPS22HB_BIT(5)
#define LPS22HB_WTM_POINT_BIT          LPS22HB_BIT(0)

#define LPS22HB_FMODE_MASK             (uint8_t)0xE0
#define LPS22HB_WTM_POINT_MASK         (uint8_t)0x1F

  /**
   * @brief FIFO Status register
   * \code
   * Read
   * Default value: 0x00
   * 7 WTM_FIFO: Watermark status. 0:FIFO filling is lower than watermark level; 1: FIFO is equal or higher than watermark level.
   * 6 FULL_FIFO: Overrun bit status. 0 - FIFO not full; 1 -FIFO is full.
   * 5 EMPTY_FIFO: Empty FIFO bit. 0 - FIFO not empty; 1 -FIFO is empty.
   * 4:0 DIFF_POINT4-0: FIFOsStored data level.
   * \endcode
   */
#define LPS22HB_STATUS_FIFO_REG        (uint8_t)0x2F
#define LPS22HB_WTM_FIFO_BIT           LPS22HB_BIT(7)
#define LPS22HB_FULL_FIFO_BIT          LPS22HB_BIT(6)
#define LPS22HB_EMPTY_FIFO_BIT         LPS22HB_BIT(5)
#define LPS22HB_DIFF_POINT_BIT         LPS22HB_BIT(0)

#define LPS22HB_WTM_FIFO_MASK          (uint8_t)0x80
#define LPS22HB_FULL_FIFO_MASK         (uint8_t)0x40
#define LPS22HB_EMPTY_FIFO_MASK        (uint8_t)0x20
#define LPS22HB_DIFF_POINT_MASK        (uint8_t)0x1F

  /**
   * @brief Pressure offset register  (LSB)
   * \code
   * Read/write
   * Default value: 0x00
   * 7:0 RPDS7-0:Pressure Offset for 1 point calibration after soldering.
   * This register contains the low part of the pressure offset value after soldering,for
   * differential pressure computing. The complete value is given by RPDS_L & RPDS_H
   * and is expressed as signed 2 complement value.
   * \endcode
   */
#define LPS22HB_RPDS_L_REG        (uint8_t)0x39

  /**
   * @brief Pressure offset register (MSB)
   * \code
   * Read/write
   * Default value: 0x00
   * 7:0 RPDS15-8:Pressure Offset for 1 point calibration after soldering.
   * This register contains the high part of the pressure offset value after soldering (see description RPDS_L)
   * \endcode
   */
#define LPS22HB_RPDS_H_REG        (uint8_t)0x3A

  /**
   * @}
   */


  /**
   * @}
   */



  /* Exported Functions -------------------------------------------------------------*/
  /** @defgroup LPS22HB_Exported_Functions
   * @{
   */

  LPS22HB_Error_et LPS22HB_Get_DriverVersion(LPS22HB_DriverVersion_st *Version);
  LPS22HB_Error_et LPS22HB_DeInit(void *handle);
  LPS22HB_Error_et LPS22HB_Activate(void *handle);
  LPS22HB_Error_et LPS22HB_DeActivate(void *handle);
  LPS22HB_Error_et LPS22HB_IsMeasurementCompleted(void *handle, uint8_t *Is_Measurement_Completed);
  LPS22HB_Error_et LPS22HB_Get_Measurement(void *handle, LPS22HB_MeasureTypeDef_st *Measurement_Value);
  LPS22HB_Error_et LPS22HB_Get_ReferencePressure(void *handle, int32_t *RefP);
  LPS22HB_Error_et LPS22HB_Set_PowerDownMode(void *handle, LPS22HB_BitStatus_et powerdown);
  LPS22HB_Error_et LPS22HB_Set_Avg(void *handle, LPS22HB_Avgp_et avgp, LPS22HB_Avgt_et avgt);
  LPS22HB_Error_et LPS22HB_Set_AvgP(void *handle, LPS22HB_Avgp_et avgp);
  LPS22HB_Error_et LPS22HB_Set_AvgT(void *handle, LPS22HB_Avgt_et avgt);
  LPS22HB_Error_et LPS22HB_Get_AvgP(void *handle, LPS22HB_Avgp_et* avgpress);
  LPS22HB_Error_et LPS22HB_Get_AvgT(void *handle, LPS22HB_Avgt_et* avgtemp);
  LPS22HB_Error_et LPS22HB_Set_Odr(void *handle, LPS22HB_Odr_et odr);
  LPS22HB_Error_et LPS22HB_Get_Odr(void *handle, LPS22HB_Odr_et* odr);
  LPS22HB_Error_et LPS22HB_Set_InterruptCircuitEnable(void *handle, LPS22HB_State_et diff_en) ;
  LPS22HB_Error_et LPS22HB_Get_InterruptCircuitEnable(void *handle, LPS22HB_State_et* diff_en);
  LPS22HB_Error_et LPS22HB_Set_Bdu(void *handle, LPS22HB_Bdu_et bdu);
  LPS22HB_Error_et LPS22HB_Get_Bdu(void *handle, LPS22HB_Bdu_et* bdu);
  LPS22HB_Error_et LPS22HB_ResetAZ(void *handle);
  LPS22HB_Error_et LPS22HB_Set_SpiInterface(void *handle, LPS22HB_SPIMode_et spimode);
  LPS22HB_Error_et LPS22HB_Get_SpiInterface(void *handle, LPS22HB_SPIMode_et* spimode);
  LPS22HB_Error_et LPS22HB_Set_I2C(void *handle, LPS22HB_State_et i2cstate);
  LPS22HB_Error_et LPS22HB_StartOneShotMeasurement(void *handle);
  LPS22HB_Error_et LPS22HB_Set_AutoZeroFunction(void *handle, LPS22HB_BitStatus_et autozero);
  LPS22HB_Error_et LPS22HB_SwReset(void *handle);
  LPS22HB_Error_et LPS22HB_MemoryBoot(void *handle);
  LPS22HB_Error_et LPS22HB_SwResetAndMemoryBoot(void *handle);
  LPS22HB_Error_et LPS22HB_Set_FifoModeUse(void *handle, LPS22HB_State_et status);
  LPS22HB_Error_et LPS22HB_Set_FifoWatermarkLevelUse(void *handle, LPS22HB_State_et status);
  LPS22HB_Error_et LPS22HB_Set_FifoMeanDecUse(void *handle, LPS22HB_State_et status);
  LPS22HB_Error_et LPS22HB_Set_InterruptActiveLevel(void *handle, LPS22HB_State_et status);
  LPS22HB_Error_et LPS22HB_Set_InterruptOutputType(void *handle, LPS22HB_OutputType_et output);
  LPS22HB_Error_et LPS22HB_Set_InterruptControlConfig(void *handle, LPS22HB_OutputSignalConfig_et config);
  LPS22HB_Error_et LPS22HB_Set_InterruptDataConfig(void *handle, LPS22HB_DataSignalType_et signal);
  LPS22HB_Error_et LPS22HB_Set_InterruptDifferentialConfig(void *handle, LPS22HB_InterruptDiffConfig_et config);
  LPS22HB_Error_et LPS22HB_LatchInterruptRequest(void *handle, LPS22HB_State_et status);
  LPS22HB_Error_et LPS22HB_Get_InterruptDifferentialEventStatus(void *handle,
                                                                LPS22HB_InterruptDiffStatus_st* interruptsource);
  LPS22HB_Error_et LPS22HB_Get_DataStatus(void *handle, LPS22HB_DataStatus_st* datastatus);
  LPS22HB_Error_et LPS22HB_Get_RawPressure(void *handle, int32_t *raw_press);
  LPS22HB_Error_et LPS22HB_Get_Pressure(void *handle, int32_t* Pout);
  LPS22HB_Error_et LPS22HB_Get_RawTemperature(void *handle, int16_t *raw_data);
  LPS22HB_Error_et LPS22HB_Get_Temperature(void *handle, int16_t* Tout);
  LPS22HB_Error_et LPS22HB_Get_PressureThreshold(void *handle, int16_t *P_ths);
  LPS22HB_Error_et LPS22HB_Set_PressureThreshold(void *handle, int16_t P_ths);
  LPS22HB_Error_et LPS22HB_Set_FifoMode(void *handle, LPS22HB_FifoMode_et fifomode);
  LPS22HB_Error_et LPS22HB_Get_FifoMode(void *handle, LPS22HB_FifoMode_et* fifomode);
  LPS22HB_Error_et LPS22HB_Get_FifoStatus(void *handle, LPS22HB_FifoStatus_st* status);
  LPS22HB_Error_et LPS22HB_Set_FifoWatermarkLevel(void *handle, uint8_t wtmlevel);
  LPS22HB_Error_et LPS22HB_Get_FifoWatermarkLevel(void *handle, uint8_t *wtmlevel);
  LPS22HB_Error_et LPS22HB_Set_FifoSampleSize(void *handle, LPS22HB_FifoMeanModeSample_et samplesize);
  LPS22HB_Error_et LPS22HB_Get_FifoSampleSize(void *handle, LPS22HB_FifoMeanModeSample_et* samplesize);
  LPS22HB_Error_et LPS22HB_Get_DeviceID(void *handle, uint8_t* deviceid);
  LPS22HB_Error_et LPS22HB_Get_PressureOffsetValue(void *handle, int16_t *pressoffset);
  LPS22HB_Error_et LPS22HB_Set_GenericConfig(void *handle, LPS22HB_ConfigTypeDef_st* pxLPS22HBInit);
  LPS22HB_Error_et LPS22HB_Get_GenericConfig(void *handle, LPS22HB_ConfigTypeDef_st* pxLPS22HBInit);
  LPS22HB_Error_et LPS22HB_Set_InterruptConfig(void *handle, LPS22HB_InterruptTypeDef_st* pLPS22HBInt);
  LPS22HB_Error_et LPS22HB_Get_InterruptConfig(void *handle, LPS22HB_InterruptTypeDef_st* pLPS22HBInt);
  LPS22HB_Error_et LPS22HB_Set_FifoConfig(void *handle, LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO);
  LPS22HB_Error_et LPS22HB_Get_FifoConfig(void *handle, LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO);



#ifdef __cplusplus
}
#endif


/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

typedef enum {
  LPS22HB_LowNoise   =  (uint8_t)0x00,       /*!< Low Noise mode */
  LPS22HB_LowPower   =  (uint8_t)0x01        /*!< Low Current mode */
} LPS22HB_PowerMode_et;

#define IS_LPS22HB_PowerMode(MODE) ((MODE == LPS22HB_LowNoise) || (MODE == LPS22HB_LowPower))
#define LPS22HB_RES_CONF_REG     (uint8_t)0x1A
#define LPS22HB_LCEN_MASK        (uint8_t)0x01

typedef enum {
  LPS22HB_ODR_9  = (uint8_t)0x00,         /*!< Filter Cutoff ODR/9 */
  LPS22HB_ODR_20 = (uint8_t)0x04          /*!< Filter Cutoff ODR/20 */
} LPS22HB_LPF_Cutoff_et;

#define IS_LPS22HB_LPF_Cutoff(CUTOFF) ((CUTOFF == LPS22HB_ODR_9) || (CUTOFF == LPS22HB_ODR_20) )
