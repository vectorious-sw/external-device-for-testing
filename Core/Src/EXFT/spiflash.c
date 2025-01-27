
#include "vlapConfig.h"
#include "spiflash.h"
#include "semphr.h"
#include "generic.h"



// L O C A L    D E F I N I T I O N S


// L O C A L    P R O T O T Y P E S
 void waitForWriteEnd(void);
 void finish(volatile SPIFLASH_Job_Type *SpiRequestDescriptor);
 void waitForNBusy(void);

// M O D U L E   G L O B A L S
QueueHandle_t spiflashReqQ;
QueueHandle_t SpiFlashLowLevelReqQ;
SPIFLASH_Job_Type SPIFLASH_jobs[2];
SemaphoreHandle_t spiflashTransactionSemaphore = NULL;

SPIFLASH_Job_Type* SpiRequestDescriptor;

SpiflashFsmState_T SpiFlashFsmState;


volatile HAL_StatusTypeDef MyStatus;
volatile HAL_StatusTypeDef MyStatusBusy;
volatile uint32_t BeforeCnt;
volatile uint32_t AfterCnt;



/******************************************************************************
* @brief  void spiflashInit()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T spiflashInit(void)
{
  // Create the Semaphore
  spiflashTransactionSemaphore = xSemaphoreCreateBinary();
  // Create the  queue  
  spiflashReqQ = xQueueCreate(3, sizeof(spiflashReqQueueEntry_T));

  SpiFlashLowLevelReqQ = xQueueCreate(3, sizeof(spiflashReqQueueEntry_T));


  // Configure the spi master module
  SPIFLASH_CS_HIGH;

  SpiFlashFsmState = SPIFLASH_FSM_STATE_IDLE;
    
  // CReate the task
  xTaskCreate(spiflashTask, spiflashTaskName, spiflashTaskSTACK_SIZE, NULL, 
              spiflashTaskPriority, ( TaskHandle_t * ) NULL );
  // Always return OK
  return(RETURNCODE_OK);
}


volatile uint8_t MyCounter;

/*******************************************************************************
* Function Name  : portTASK_FUNCTION(spiflashTask, pvParameters )
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
portTASK_FUNCTION(spiflashTask, pvParameters )
{
  spiflashReqQueueEntry_T QueueEntry;
  uint8_t Header[4];
  uint32_t TaskMsecDelay = 0;
  
  
//  // Reset spi
//  hwdriversGpioBitWrite(HWDRIVERS_PC8_SFLASH_RESET_N, 0);
//  vTaskDelay(1);
//  hwdriversGpioBitWrite(HWDRIVERS_PC8_SFLASH_RESET_N, 1);
  
  
  while(1)
  {
    // Wait for queue event
    if (xQueueReceive(spiflashReqQ, &QueueEntry, 3000) == pdTRUE)
    {

        uint32_t WriteAddr = QueueEntry.SpiFlashAddress;
  #if (SPIFLASH_SPI_PAGESIZE == 256)
        Header[1] = (WriteAddr & 0xFF0000) >> 16;						// Page Size = 256
        Header[2] = (WriteAddr & 0xFF00) >> 8;
        Header[3] = (WriteAddr & 0xFF) >> 0;
  #else
        Header[1] = Header[2] = Header[3] = 0;
        Header[3] = WriteAddr % SPIFLASH_SPI_PAGESIZE;                            // Byte addressing  support
        Header[2] = (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x007f) << 1);   	// Page Size = 264, See AT45DB641E: Table 15-7. Detailed Bit-level Addressing Sequence for Standard DataFlash Page Size (264 bytes)
        Header[2] |= (((WriteAddr % SPIFLASH_SPI_PAGESIZE) & 0x0100)>>8);
        Header[1] =  (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x0080) >> 7);         //                      PA14  Header[1]     Header[2] PA9        Header[3] PA0
        Header[1] |= (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x7f00) >> 7);         // 01h 0 0 0 0 0 0 0 1  P P P P P P P P   P P P P P P P B    B B B B B B B B
                                                                                  // 02h 0 0 0 0 0 0 1 0  P P P P P P P P   P P P P P P P B    B B B B B B B B
  #endif                                                                          // 03h 0 0 0 0 0 0 1 1  P P P P P P P P   P P P P P P P B    B B B B B B B B



        // Switch over the command and set the SPI FLash command and the wait time (the actual time it takes for the flash device to implement the comment after the spi transaction ends)
        switch(QueueEntry.SpiRequestType)
        {
        case SPIFLASH_CMD_READ:
          Header[0] = SPIFLASH_AT45DB641E_CMD_READ;
          TaskMsecDelay = 0;
          vdbg(">>>>>> SPI read address %06x header %02x %02x %02x\n\r", WriteAddr, Header[3], Header[2], Header[1] );
          break;
        case  SPIFLASH_CMD_PROG_VIA_BUFF_1:
          Header[0] = SPIFLASH_AT45DB641E_CMD_ERASE_WRITE_VIA_BUFF_1;
          TaskMsecDelay = 35/portTICK_PERIOD_MS;
          vdbg(">>>>>> SPI write address %06x header %02x %02x %02x\n\r", WriteAddr, Header[3], Header[2], Header[1] );
          break;
        case  SPIFLASH_CMD_PROG_BUFF_2_TO_MAIN:
          Header[0] = SPIFLASH_AT45DB641E_CMD_ERASE_WRITE_BUFF_2;
          TaskMsecDelay = 35/portTICK_PERIOD_MS;
          vdbg(">>>>>> SPI write address %06x header %02x %02x %02x\n\r", WriteAddr, Header[3], Header[2], Header[1] );
          break;
        case SPIFLASH_CMD_BUFF_2_WRITE:
          Header[0] = SPIFLASH_AT45DB641E_CMD_WRITE_BUFF_2;
          TaskMsecDelay = 0;
          vdbg(">>>>>> SPI write buffer 2 at address %06x header %02x %02x %02x\n\r", WriteAddr, Header[3], Header[2], Header[1] );
          break;
        case SPIFLASH_CMD_PAGE_ERASE:
          Header[0] = SPIFLASH_AT45DB641E_CMD_ERASE_PAGE;
          Header[1] = ((WriteAddr/SPIFLASH_SPI_PAGESIZE)<<1) >> 8;
          Header[2] = ((WriteAddr/SPIFLASH_SPI_PAGESIZE)<<1);
          Header[3] = 0;
          TaskMsecDelay = 35/portTICK_PERIOD_MS;
          break;
        case SPIFLASH_CMD_PAGE_CONFIGURE:
          Header[0] = SPIFLASH_AT45DB641E_CMD_CONFIG;
          Header[1] = 0x2a;
          Header[2] = 0x80;
          Header[3] = 0xa6;
          TaskMsecDelay = 35/portTICK_PERIOD_MS;
          vdbg(">>>>>> SPI page config address %06x header %02x %02x %02x\n\r", WriteAddr, Header[3], Header[2], Header[1] );
          break;
        case SPIFLASH_CMD_CHIP_ERASE:
          Header[0] = SPIFLASH_AT45DB641E_CMD_CHIP_ERASE;
          Header[1] = 0x94;
          Header[2] = 0x80;
          Header[3] = 0x9a;
          TaskMsecDelay = 100000/portTICK_PERIOD_MS;
          vdbg(">>>>>> SPI page config address %06x header %02x %02x %02x\n\r", WriteAddr, Header[3], Header[2], Header[1] );
          break;
        default:
          TaskMsecDelay = 35/portTICK_PERIOD_MS;
          break;
        }
        // Enable the SPI FLASH device
        SPIFLASH_CS_LOW;
        // Fill the global flash job descriptor, will be used by the ISR to complete the operation
        SPIFLASH_jobs[0].mode = QueueEntry.SpiRequestType;
        SPIFLASH_jobs[0].Header = Header;
        SPIFLASH_jobs[0].HeaderLen = 4;
        SPIFLASH_jobs[0].TxBuffer = QueueEntry.TxBuffer;
        SPIFLASH_jobs[0].RxBuffer = QueueEntry.RxBuffer;
        SPIFLASH_jobs[0].Count = QueueEntry.Count;
        SPIFLASH_jobs[0].CompletionCallBackPtr = QueueEntry.CompletionCallBackPtr;
        SPIFLASH_jobs[0].State = 0;

        HAL_SPI_Transmit_DMA(&hspi1, SPIFLASH_jobs[0].Header, SPIFLASH_jobs[0].HeaderLen);

        SpiFlashFsmState = SPIFLASH_FSM_STATE_HEADER_WRITE_WAIT;
        // Inner loop
    	while(SpiFlashFsmState != SPIFLASH_FSM_STATE_IDLE)
    	{
    	    if (xQueueReceive(SpiFlashLowLevelReqQ, &QueueEntry, 3000) == pdTRUE)
    	    {
				switch(SpiFlashFsmState)
				{
					case SPIFLASH_FSM_STATE_IDLE:
						break;
					case  SPIFLASH_FSM_STATE_HEADER_WRITE_WAIT:
						SpiRequestDescriptor = &SPIFLASH_jobs[0];
						switch (SpiRequestDescriptor->mode)
						{
						case SPIFLASH_CMD_READ:
						  // header sent. Now clock the receive
						  waitForNBusy();
						  HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)SPIFLASH_jobs[0].RxBuffer, SPIFLASH_jobs[0].Count);
						  enaIrqRx;
						  SpiFlashFsmState = SPIFLASH_FSM_STATE_PAYLOAD_WAIT;
						  break;

						case SPIFLASH_CMD_PROG_VIA_BUFF_1:
						case SPIFLASH_CMD_BUFF_2_WRITE:
						  // header sent. Now clock the transmit
							waitForNBusy();
							MyStatus = HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)SpiRequestDescriptor->TxBuffer, SpiRequestDescriptor->Count);
							if(MyStatus==HAL_BUSY)
								MyStatusBusy++;
							enaIrqTx;
							SpiFlashFsmState = SPIFLASH_FSM_STATE_PAYLOAD_WAIT;
						  break;

						case SPIFLASH_CMD_PAGE_CONFIGURE:
						case SPIFLASH_CMD_DO_NOTHING:
						case SPIFLASH_CMD_CHIP_ERASE:
						case SPIFLASH_CMD_PAGE_ERASE:
						  // command complete. wait to settle and finish
						  waitForWriteEnd();
						  waitForNBusy();
						  finish(SpiRequestDescriptor);
						  // Suspend the next queue entry processing till the spi operation ends, We assume that "Page Erase and Programming" take max time of 35 mSec
						  if(TaskMsecDelay)
							  vTaskDelay( TaskMsecDelay );
						  SpiFlashFsmState = SPIFLASH_FSM_STATE_IDLE;
						  break;
						default:
						  break;
						}
						break;

					case  SPIFLASH_FSM_STATE_PAYLOAD_WAIT:
				   	    // Turn off the SPI FLASH device
				    	    SPIFLASH_CS_HIGH;
					   	  // Suspend the next queue entry processing till the spi operation ends, We assume that "Page Erase and Programming" take max time of 35 mSec
					      if(TaskMsecDelay)
					          vTaskDelay( TaskMsecDelay );
					      // End of complete session
					      // Check if the command queue entry requested to free the TxBuffer and free the buffer
					      if(SPIFLASH_jobs[0].FreeTxBufferFlag)
					        if(SPIFLASH_jobs[0].TxBuffer)
					          vPortFree(SPIFLASH_jobs[0].TxBuffer);

					      // If the spi command request asked for returned message, send the message to the queue handle stored in the job descriptor
					      // The queue entry type must be genericQueueEntryT  !!!!!
					      if(SPIFLASH_jobs[0].CompletionCallBackPtr)
					      {
					        SPIFLASH_jobs[0].CompletionCallBackPtr();
					      }
					      SpiFlashFsmState = SPIFLASH_FSM_STATE_IDLE;
						break;
					case  SPIFLASH_FSM_STATE_FLASH_ERASE_WRITE_WAIT:
						break;
				}// switch
    	    } // Inner queue if
    	    else
    	    	 SpiFlashFsmState = SPIFLASH_FSM_STATE_IDLE;
    	} // Inner while


    } // Outer queue
    else
    {
      // Queue Rx error 
    }
  } // Outer while
  
}



volatile uint8_t MySpiErrorCounter;

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{

	MySpiErrorCounter++;
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	spiflashReqQueueEntry_T Request;
	BeforeCnt++;
	if(hspi == &hspi1)
	{

		AfterCnt++;

		Request.SpiRequestType		  = SPIFLASH_CMD_LOW_LEVEL_COMPLETED;
		Request.FreeTxBufferFlag      = 0;

		// send the request to spi flash
		xQueueSendFromISR(SpiFlashLowLevelReqQ, &Request, 0);

	}
}





void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	spiflashReqQueueEntry_T Request;

	if(hspi == &hspi1)
	{
		disIrqRx;

		SpiRequestDescriptor = &SPIFLASH_jobs[0];
		if (SpiRequestDescriptor->mode == SPIFLASH_CMD_READ)
		{
		  // receive finished
		  waitForNBusy();
		  //      if (SpiRequestDescriptor->mode == SPIFLASH_READ_ELOG) do it outside --- giora
		  //        advance
		  finish(SpiRequestDescriptor);

		  Request.SpiRequestType        = SPIFLASH_CMD_LOW_LEVEL_COMPLETED;
		  Request.FreeTxBufferFlag      = 0;

		  // send the request to spi flash
		  xQueueSendFromISR(SpiFlashLowLevelReqQ, &Request, 0);
		}
	}
}


#if 0
/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Stream0_IRQHandler(void) // DMA Rx stream irq handler
{
  volatile SPIFLASH_Job_Type *SpiRequestDescriptor;
  if ( __HAL_SPI_GET_FLAG(&hdma_spi1_rx, DMA_IT_TCIF0))
  {
    disIrqRx;
    /* Clear DMA Transfer Complete interrupt pending bit */
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);    
    
    SpiRequestDescriptor = &SPIFLASH_jobs[0];
    if (SpiRequestDescriptor->mode == SPIFLASH_CMD_READ)
    {
      // receive finished     
      waitForNBusy();
      //      if (SpiRequestDescriptor->mode == SPIFLASH_READ_ELOG) do it outside --- giora
      //        advance
      finish(SpiRequestDescriptor);
    }
  }
}  

#endif




/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
void finish(volatile SPIFLASH_Job_Type *SpiRequestDescriptor)
{
  
  // disable all interrupts just to be safe
  disIrqTx;
  disIrqRx;
 
  switch(SpiRequestDescriptor->mode)
  {
  case SPIFLASH_CMD_READ:
    break;
  case SPIFLASH_CMD_PROG_VIA_BUFF_1:
    if((void *)SpiRequestDescriptor->FreeTxBufferFlag && (void *)SpiRequestDescriptor->TxBuffer)
      vPortFree((void *)SpiRequestDescriptor->TxBuffer);
    break;
  case SPIFLASH_CMD_DO_NOTHING:
    break;
  default:
	  break;
    }
    // Turn off the SPI FLASH device
//    SPIFLASH_CS_HIGH;
    // Send indication to the spiflashTask that the spi command transaction has ended
 //   xSemaphoreGiveFromISR(spiflashTransactionSemaphore, NULL);
}



/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
void waitForNBusy(void)
{
  // bug: dma completes but spi remains busy for a while
//  while( __HAL_DMA_GET_FLAG(&hdma_spi1_tx, SPI_FLAG_BSY)); // must wait until spi !busy
while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

}

/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/


void waitForWriteEnd(void)
{
  volatile uint8_t opcode = SPIFLASH_AT45DB641E_CMD_RDSR;
  volatile uint8_t reg;
  
  SPIFLASH_CS_HIGH;
  while(1)
  {
    
    disIrqTx;
    SPIFLASH_CS_LOW;
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&opcode, 1, 10 );
    while( HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
//    TM_SPI_DMA_Transmit(SPI1, (uint8_t *)&opcode, NULL, 1);
//    while(TM_SPI_DMA_Working(SPI1));
    
    HAL_SPI_Receive(&hspi1, (uint8_t *)&reg, 1, 10);

//    TM_SPI_DMA_Transmit(SPI1, NULL, (uint8_t *)&reg, 1);
//    while(TM_SPI_DMA_Working(SPI1)); // must wait until spi !busy
    
    SPIFLASH_CS_HIGH;
    if ((reg & SPIFLASH_AT45DB641E_NRDY_FLAG) != 0)
      break;
  }
}	

volatile uint8_t testCount;

#if 1
/******************************************************************************
* @brief ReturnCode_T spiflashReqEnqueue(SpiflashReq_T OpCode, uint32_t SpiFlashAddress, uint8_t *RxDestinationPtr, uint8_t *TxSourcePtr, uint16_t OperationBytesCount, QueueHandle_t *CompletionHandlerFunctionPtr, uint8_t FreeTxBufferFlag)

* @param  
* @retval 
******************************************************************************/
ReturnCode_T spiflashReqEnqueue(SpiflashReq_T OpCode, uint32_t SpiFlashAddress, uint8_t *RxDestinationPtr, uint8_t *TxSourcePtr, uint16_t OperationBytesCount, void(*CompletionCallBackPtr)(), uint8_t FreeTxBufferFlag)
{
  spiflashReqQueueEntry_T Request;

#if 0 
  printf("Address %d \r", SpiFlashAddress);
  for(i=0; i<OperationBytesCount; i++)
    printf("%02x ", *(TxSourcePtr+i));
#endif
  
  // Send spi read request 
  Request.SpiFlashAddress       = SpiFlashAddress;
  Request.RxBuffer              = RxDestinationPtr;
  Request.TxBuffer              = TxSourcePtr;
  Request.Count                 = OperationBytesCount;
  Request.SpiRequestType        = OpCode;
  Request.CompletionCallBackPtr = CompletionCallBackPtr;     
  Request.FreeTxBufferFlag      = FreeTxBufferFlag;

  if(SpiFlashAddress < 0x400)
    testCount++;
  
  // send the request to spi flash 
  xQueueSend(spiflashReqQ, &Request, 0);
  
  return RETURNCODE_OK;
}  

#endif


uint8_t Header[4];
void spiflashPvdBuff2WriteCmd()
{
  SPIFLASH_CS_LOW;
  uint32_t WriteAddr = SPIFLASH_SYSTEM_START;
  Header[1] = Header[2] = Header[3] = 0;
  Header[3] = WriteAddr % SPIFLASH_SPI_PAGESIZE;                                // Byte addressing  support 
  Header[2] = (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x007f) << 1);   	        // Page Size = 264, See AT45DB641E: Table 15-7. Detailed Bit-level Addressing Sequence for Standard DataFlash Page Size (264 bytes)
  Header[2] |= (((WriteAddr % SPIFLASH_SPI_PAGESIZE) & 0x0100)>>8); 
  Header[1] =  (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x0080) >> 7);             //                      PA14  Header[1]     Header[2] PA9    Header[3] PA0
  Header[1] |= (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x7f00) >> 7);             // 01h 0 0 0 0 0 0 0 1  P P P P P P P P   P P P P P P P B    B B B B B B B B
                                                                                // 02h 0 0 0 0 0 0 1 0  P P P P P P P P   P P P P P P P B    B B B B B B B B
                                                                                // 03h 0 0 0 0 0 0 1 1  P P P P P P P P   P P P P P P P B    B B B B B B B B
  Header[0] = SPIFLASH_AT45DB641E_CMD_BUFF2_TO_MAIN_NOERASE;
  // TODO: Replace with HAL
  //TM_SPI_DMA_Transmit(SPI1, Header, 0, 4);
}




void spiflashBlockingRead( uint32_t SpiFlashAddress, uint16_t ReadLength, uint8_t* ReturnedDataPtr)
{
  uint8_t Header[4];
  
  SPIFLASH_CS_LOW;
  uint32_t WriteAddr = SpiFlashAddress;
  Header[1] = Header[2] = Header[3] = 0;
  Header[3] = WriteAddr % SPIFLASH_SPI_PAGESIZE;                                // Byte addressing  support 
  Header[2] = (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x007f) << 1);   	        // Page Size = 264, See AT45DB641E: Table 15-7. Detailed Bit-level Addressing Sequence for Standard DataFlash Page Size (264 bytes)
  Header[2] |= (((WriteAddr % SPIFLASH_SPI_PAGESIZE) & 0x0100)>>8); 
  Header[1] =  (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x0080) >> 7);             //                      PA14  Header[1]     Header[2] PA9    Header[3] PA0
  Header[1] |= (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x7f00) >> 7);             // 01h 0 0 0 0 0 0 0 1  P P P P P P P P   P P P P P P P B    B B B B B B B B
                                                                               // 03h 0 0 0 0 0 0 1 1  P P P P P P P P   P P P P P P P B    B B B B B B B B
  Header[0] = SPIFLASH_AT45DB641E_CMD_READ;


  HAL_SPI_Transmit(&hspi1, Header, sizeof(Header), 20);
  HAL_SPI_Receive(&hspi1, ReturnedDataPtr, ReadLength, 10);

#if 0
  TM_SPI_DMA_Transmit(SPI1, Header, ReturnedDataPtr, sizeof(Header));
  while(TM_SPI_DMA_Working(SPI1));
  TM_SPI_DMA_Transmit(SPI1, 0, ReturnedDataPtr,  ReadLength);
  while(TM_SPI_DMA_Working(SPI1));
#endif
  SPIFLASH_CS_HIGH;
}

void spiflashBlockingWriteWithoutErase( uint32_t SpiFlashAddress, uint16_t WriteLength, uint8_t* WriteDataPtr)
{
  uint8_t Header[4];
  
  SPIFLASH_CS_LOW;
  uint32_t WriteAddr = SpiFlashAddress;
  Header[1] = Header[2] = Header[3] = 0;
  Header[3] = WriteAddr % SPIFLASH_SPI_PAGESIZE;                                // Byte addressing  support 
  Header[2] = (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x007f) << 1);   	        // Page Size = 264, See AT45DB641E: Table 15-7. Detailed Bit-level Addressing Sequence for Standard DataFlash Page Size (264 bytes)
  Header[2] |= (((WriteAddr % SPIFLASH_SPI_PAGESIZE) & 0x0100)>>8); 
  Header[1] =  (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x0080) >> 7);             //                      PA14  Header[1]     Header[2] PA9    Header[3] PA0
  Header[1] |= (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x7f00) >> 7);             // 01h 0 0 0 0 0 0 0 1  P P P P P P P P   P P P P P P P B    B B B B B B B B
  // 02h 0 0 0 0 0 0 1 0  P P P P P P P P   P P P P P P P B    B B B B B B B B
  Header[0] = SPIFLASH_AT45DB641E_CMD_WRITE;
  

  HAL_SPI_Transmit(&hspi1, Header, sizeof(Header), 20);
 HAL_SPI_Transmit(&hspi1, WriteDataPtr, WriteLength, 20);



#if 0
  TM_SPI_DMA_Transmit(SPI1, Header, /*WriteDataPtr*/ 0, sizeof(Header));
  while(TM_SPI_DMA_Working(SPI1));
  
  TM_SPI_DMA_Transmit(SPI1, WriteDataPtr, NULL,  WriteLength);
  while(TM_SPI_DMA_Working(SPI1));
#endif
  
  SPIFLASH_CS_HIGH;
}

void spiflashBlockingWrite( uint32_t SpiFlashAddress, uint16_t WriteLength, uint8_t* WriteDataPtr)
{
  uint8_t Header[4];
  
  SPIFLASH_CS_LOW;
  uint32_t WriteAddr = SpiFlashAddress;
  Header[1] = Header[2] = Header[3] = 0;
  Header[3] = WriteAddr % SPIFLASH_SPI_PAGESIZE;                                // Byte addressing  support 
  Header[2] = (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x007f) << 1);   	        // Page Size = 264, See AT45DB641E: Table 15-7. Detailed Bit-level Addressing Sequence for Standard DataFlash Page Size (264 bytes)
  Header[2] |= (((WriteAddr % SPIFLASH_SPI_PAGESIZE) & 0x0100)>>8); 
  Header[1] =  (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x0080) >> 7);             //                      PA14  Header[1]     Header[2] PA9    Header[3] PA0
  Header[1] |= (((WriteAddr/SPIFLASH_SPI_PAGESIZE) & 0x7f00) >> 7);             // 01h 0 0 0 0 0 0 0 1  P P P P P P P P   P P P P P P P B    B B B B B B B B
                                                                                // 02h 0 0 0 0 0 0 1 0  P P P P P P P P   P P P P P P P B    B B B B B B B B
  Header[0] = SPIFLASH_AT45DB641E_CMD_ERASE_WRITE_VIA_BUFF_1;

  HAL_SPI_Transmit(&hspi1, Header, sizeof(Header), 20);
#if 0
  TM_SPI_DMA_Transmit(SPI1, Header, /*WriteDataPtr*/ 0, sizeof(Header));
  while(TM_SPI_DMA_Working(SPI1));
#endif

  HAL_SPI_Transmit(&hspi1, WriteDataPtr, WriteLength, 20);
#if 0
  TM_SPI_DMA_Transmit(SPI1, WriteDataPtr, NULL,  WriteLength);
  while(TM_SPI_DMA_Working(SPI1));
#endif

  SPIFLASH_CS_HIGH;
}

