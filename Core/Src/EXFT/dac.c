#include "common.h"
#include "vlapConfig.h"
#include "dac.h"



// L O C A L    D E F I N I T I O N S


// L O C A L    P R O T O T Y P E S


// M O D U L E   G L O B A L S






/******************************************************************************
* @brief  void dacInit()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T dacInit(void)
{
	// Will be managed by the HAL init
    HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
  
  return(RETURNCODE_OK);
}


/******************************************************************************
* @brief  void dacWrite()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T dacWrite(uint16_t DacWord)
{
	ReturnCode_T MyReturn = RETURNCODE_OK;

	HAL_StatusTypeDef  MyHalReturn = HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DacWord);
  
	if(MyHalReturn == HAL_OK)
		MyReturn = RETURNCODE_OK;
	else
		MyReturn = RETURNCODE_ERROR;

  return(MyReturn);
}

