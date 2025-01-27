#pragma once

#include "common.h"

// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 
typedef enum{SPIFLASH_FSM_STATE_IDLE, SPIFLASH_FSM_STATE_HEADER_WRITE_WAIT, SPIFLASH_FSM_STATE_PAYLOAD_WAIT, SPIFLASH_FSM_STATE_FLASH_ERASE_WRITE_WAIT} SpiflashFsmState_T;
typedef enum{SPIFLASH_CMD_READ, SPIFLASH_CMD_PROG_VIA_BUFF_1, SPIFLASH_CMD_PROG_BUFF_2_TO_MAIN, SPIFLASH_CMD_BUFF_2_WRITE, SPIFLASH_CMD_DO_NOTHING, SPIFLASH_CMD_PAGE_CONFIGURE, SPIFLASH_CMD_CHIP_ERASE, SPIFLASH_CMD_PAGE_ERASE, SPIFLASH_CMD_LOW_LEVEL_COMPLETED} SpiflashReq_T;;

#define SPIFLASH_CS_LOW      HAL_GPIO_WritePin(GPIOA, SPIFLASH_SPI_CS_N_Pin, GPIO_PIN_RESET); // spi flash chip-select low
#define SPIFLASH_CS_HIGH     HAL_GPIO_WritePin(GPIOA, SPIFLASH_SPI_CS_N_Pin, GPIO_PIN_SET); 	// spi-flash chip-select high




#define enaIrqRx  //__HAL_DMA_ENABLE_IT(&hdma_spi1_rx, DMA_IT_TC  );
#define enaIrqTx  //__HAL_DMA_ENABLE_IT(&hdma_spi1_tx, DMA_IT_TC  );
#define disIrqRx  //__HAL_DMA_DISABLE_IT(&hdma_spi1_rx, DMA_IT_TC );
#define disIrqTx  //__HAL_DMA_DISABLE_IT(&hdma_spi1_tx, DMA_IT_TC );


#if 0
#define enaIrqRx  DMA_ITConfig(SPI1_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
#define enaIrqTx  DMA_ITConfig(SPI1_DMA_TX_STREAM, DMA_IT_TC, ENABLE);
#define disIrqRx  DMA_ITConfig(SPI1_DMA_RX_STREAM, DMA_IT_TC, DISABLE);
#define disIrqTx  DMA_ITConfig(SPI1_DMA_TX_STREAM, DMA_IT_TC, DISABLE);
#endif




#pragma pack(1)
typedef  struct
{
  SpiflashReq_T SpiRequestType; 
  uint8_t *TxBuffer;
  uint8_t *RxBuffer;
  uint32_t SpiFlashAddress;
  uint16_t Count;
  void (*CompletionCallBackPtr)();
  uint8_t       FreeTxBufferFlag;
} spiflashReqQueueEntry_T;


typedef struct{
  SpiflashReq_T mode; 
  uint8_t *Header;
  uint16_t HeaderLen;
  uint8_t *TxBuffer;
  uint8_t *RxBuffer;
  uint16_t Count;
  void (*CompletionCallBackPtr)();
  uint8_t       FreeTxBufferFlag;
  int State;
}SPIFLASH_Job_Type;

#if 0
#define enaIrqRx  DMA_ITConfig(SPI1_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
#define enaIrqTx  DMA_ITConfig(SPI1_DMA_TX_STREAM, DMA_IT_TC, ENABLE);
#define disIrqRx  DMA_ITConfig(SPI1_DMA_RX_STREAM, DMA_IT_TC, DISABLE);
#define disIrqTx  DMA_ITConfig(SPI1_DMA_TX_STREAM, DMA_IT_TC, DISABLE);
#endif

/* Adesto AT45DB641E  FLASH SPI Interface pins  

   8 megabytes  = 64-Mbit DataFlash (with Extra 2-Mbits)
   page size = 0x100

   DS-45DB641E-027.pdf
*/
#define SPIFLASH_AT45DB641E_CMD_WRITE                      0x02  /* Write to Memory instruction */
#define SPIFLASH_AT45DB641E_CMD_WRITE_BUFF_2               0x87  /* Buffer 2 Write  */
#define SPIFLASH_AT45DB641E_CMD_ERASE_PAGE                 0x81  /* Page Erase */       
#define SPIFLASH_AT45DB641E_CMD_ERASE_WRITE_VIA_BUFF_1     0x82  /* Main Memory Page Program through Buffer 1 with Built-In Erase */
#define SPIFLASH_AT45DB641E_CMD_ERASE_WRITE_VIA_BUFF_2     0x85  /* Main Memory Page Program through Buffer 2 with Built-In Erase */
#define SPIFLASH_AT45DB641E_CMD_BUFF2_TO_MAIN_NOERASE      0x89  /* Buffer 2 to Main Memory Page Program without Built-In Erase*/
#define SPIFLASH_AT45DB641E_CMD_ERASE_WRITE_BUFF_2         0x86  /* Buffer 2 to Main Memory Page Program with Built-In Erase */
#define SPIFLASH_AT45DB641E_CMD_WRSR                       0x01  /* Write Status Register instruction */
#define SPIFLASH_AT45DB641E_CMD_WREN                       0x06  /* Write enable instruction */
#define SPIFLASH_AT45DB641E_CMD_READ                       0x03  /* Read from Memory instruction */
#define SPIFLASH_AT45DB641E_CMD_RDSR                       0xD7  /* Read Status Register instruction  */
#define SPIFLASH_AT45DB641E_CMD_RDID                       0x9F  /* Read identification */
#define SPIFLASH_AT45DB641E_CMD_SE                         0x7C  /* Sector Erase instruction */
#define SPIFLASH_AT45DB641E_CMD_PE                         0x81  /* Page Erase instruction */
#define SPIFLASH_AT45DB641E_CMD_BE                         0xC7  /* Bulk Erase instruction */

#define SPIFLASH_AT45DB641E_CMD_CONFIG                     0x3d    // Configuration command , used for page configuration
#define SPIFLASH_AT45DB641E_CMD_WRITE_TO_BUFFER2           0x87  /* Write to buffer2 instruction */

#define SPIFLASH_AT45DB641E_CMD_CHIP_ERASE                 0xc7;

#define SPIFLASH_AT45DB641E_NRDY_FLAG                      0x80  /* n-ready flag */

#define SPIFLASH__AT45DB641E_DUMMY_BYTE                    0xA5

#define SPIFLASH_SPI_PAGESIZE                   264
#define SPIFLASH_TOTAL_NUMBER_OF_PAGES          32768     
#define SPIFLASH_FLASH_SIZE                     (SPIFLASH_SPI_PAGESIZE * SPIFLASH_TOTAL_NUMBER_OF_PAGES) // 8650752,840000 for 264 page size


#define PAGE_ADDR(x) (x & 0xffffff00)

//
// flash zones, in pages
//
     
#define SPIFLASH_NUMBEROFPAGES_SYSTEM (1)               //          
#define SPIFLASH_NUMBEROFPAGES_PRODUCTION_DB (1)        //          
#define SPIFLASH_NUMBEROFPAGES_FWUPGRADECONTROL (1)     // 
#define SPIFLASH_NUMBEROFPAGES_CONFIG (1988)            // 
#define SPIFLASH_NUMBEROFPAGES_BACKUP_IMAGE (3972)      // 
#define SPIFLASH_NUMBEROFPAGES_UPGRADE_IMAGE (3972)     // 
#define SPIFLASH_NUMBEROFPAGES_EVENTS (22833)           // 

#if ((SPIFLASH_NUMBEROFPAGES_EVENTS + SPIFLASH_NUMBEROFPAGES_UPGRADE_IMAGE + SPIFLASH_NUMBEROFPAGES_BACKUP_IMAGE + SPIFLASH_NUMBEROFPAGES_CONFIG + SPIFLASH_NUMBEROFPAGES_FWUPGRADECONTROL + SPIFLASH_NUMBEROFPAGES_PRODUCTION_DB + SPIFLASH_NUMBEROFPAGES_SYSTEM) > SPIFLASH_TOTAL_NUMBER_OF_PAGES)
 #error "spiflash.h: Page allocation error"
#endif

#if ((SPIFLASH_NUMBEROFPAGES_SYSTEM + SPIFLASH_NUMBEROFPAGES_PRODUCTION_DB + SPIFLASH_NUMBEROFPAGES_FWUPGRADECONTROL + SPIFLASH_NUMBEROFPAGES_CONFIG  + SPIFLASH_NUMBEROFPAGES_BACKUP_IMAGE + SPIFLASH_NUMBEROFPAGES_UPGRADE_IMAGE + SPIFLASH_NUMBEROFPAGES_EVENTS) < SPIFLASH_TOTAL_NUMBER_OF_PAGES)
  #error "spiflash.h: SPI Flash not fully allocated"
#endif
     

// System page section
#define SPIFLASH_SYSTEM_START (SPIFLASH_PRODUCTION_END)
#define SPIFLASH_SYSTEM_SIZE  (SPIFLASH_NUMBEROFPAGES_SYSTEM * SPIFLASH_SPI_PAGESIZE)            
#define SPIFLASH_SYSTEM_END   (SPIFLASH_SYSTEM_START + SPIFLASH_SYSTEM_SIZE)        

// Production section
#define SPIFLASH_PRODUCTION_START (SPIFLASH_FWUPGRADECONTROL_END)
#define SPIFLASH_PRODUCTION_SIZE  (SPIFLASH_NUMBEROFPAGES_PRODUCTION_DB * SPIFLASH_SPI_PAGESIZE)            
#define SPIFLASH_PRODUCTION_END   (SPIFLASH_PRODUCTION_START + SPIFLASH_PRODUCTION_SIZE)        

// Fw Upgrade Control Section
#define SPIFLASH_FWUPGRADECONTROL_START (SPIFLASH_CONFIG_END)
#define SPIFLASH_FWUPGRADECONTROL_SIZE  (SPIFLASH_NUMBEROFPAGES_FWUPGRADECONTROL * SPIFLASH_SPI_PAGESIZE)
#define SPIFLASH_FWUPGRADECONTROL_END   (SPIFLASH_FWUPGRADECONTROL_START + SPIFLASH_FWUPGRADECONTROL_SIZE)

// Config Section 
#define SPIFLASH_CONFIG_START (SPIFLASH_VANILLA_END)
#define SPIFLASH_CONFIG_SIZE  (SPIFLASH_NUMBEROFPAGES_CONFIG * SPIFLASH_SPI_PAGESIZE)            
#define SPIFLASH_CONFIG_END   (SPIFLASH_CONFIG_START + SPIFLASH_CONFIG_SIZE)        

// Backup image section 
#define SPIFLASH_VANILLA_START (SPIFLASH_UPGRADE_END)
#define SPIFLASH_VANILLA_SIZE  (SPIFLASH_NUMBEROFPAGES_BACKUP_IMAGE * SPIFLASH_SPI_PAGESIZE)
#define SPIFLASH_VANILLA_END   (SPIFLASH_VANILLA_START + SPIFLASH_VANILLA_SIZE)

// Upgrade Section
#define SPIFLASH_UPGRADE_START (SPIFLASH_ZONE_EVENTS_END)
#define SPIFLASH_UPGRADE_SIZE  (SPIFLASH_NUMBEROFPAGES_UPGRADE_IMAGE * SPIFLASH_SPI_PAGESIZE)
#define SPIFLASH_UPGRADE_END   (SPIFLASH_UPGRADE_START + SPIFLASH_UPGRADE_SIZE)

// Events Section
#define SPIFLASH_ZONE_EVENTS_START (0x00000000)
#define SPIFLASH_ZONE_EVENTS_SIZE  (SPIFLASH_NUMBEROFPAGES_EVENTS * SPIFLASH_SPI_PAGESIZE)            
#define SPIFLASH_ZONE_EVENTS_END   (SPIFLASH_ZONE_EVENTS_START + SPIFLASH_ZONE_EVENTS_SIZE)        


//#define SPIFLASH_SYSTEM_PRODUCTION_START (SPIFLASH_CONFIG_END)
//#define SPIFLASH_SYSTEM_PRODUCTION_SIZE  (SPIFLASH_NUMBEROFPAGES_PRODUCTION_DB * SPIFLASH_SPI_PAGESIZE)            
//#define SPIFLASH_SYSTEM_PRODUCTION_END   (SPIFLASH_SYSTEM_PRODUCTION_START + SPIFLASH_SYSTEM_PRODUCTION_SIZE)        
   

#if (SPIFLASH_SYSTEM_END >  (SPIFLASH_TOTAL_NUMBER_OF_PAGES*SPIFLASH_SPI_PAGESIZE))
#error "spiflash.h: SPI Flash Allocation overflow"
#endif


#define SPIFLASH_SYNC_PATTERN           0x89ABCDEF
#define SPIFLASH_SYNC_PATTERN_SIZE      4
// NVM entry header     
#pragma pack(1)
typedef     struct {
  uint32_t SyncPattern;         // Should hold the pattern SPIFLASH_SYNC_PATTERN
  uint32_t EntrySize;           // 
  uint32_t PrevEntryAddress;    // 
  uint32_t CRC32;
} spiflashLogMemoryHeaderT;
     



//
// defines for TM_library
//



// L O C A L    T Y P E S   A N D    D E F I N I T I O N S  

// G L O B A L  P R O T O T Y P E S 
ReturnCode_T spiflashInit(void);
ReturnCode_T spiflashCommandWrite(SpiflashReq_T spiFlashRequest, uint32_t spiAddress, uint8_t* RxPtr, uint8_t* TxPtr, uint16_t Length);
ReturnCode_T spiflashReqEnqueue(SpiflashReq_T OpCode, uint32_t SpiFlashAddress, uint8_t *RxDestinationPtr, uint8_t *TxSourcePtr, uint16_t OperationBytesCount, void(*CompletionCallBackPtr)(), uint8_t FreeTxBufferFlag);
void spiflashBlockingRead( uint32_t SpiFlashAddress, uint16_t ReadLength, uint8_t* ReturnedDataPtr);
void spiflashBlockingWrite( uint32_t SpiFlashAddress, uint16_t WriteLength, uint8_t* WriteDataPtr);
void spiflashBlockingWriteWithoutErase( uint32_t SpiFlashAddress, uint16_t WriteLength, uint8_t* WriteDataPtr);




