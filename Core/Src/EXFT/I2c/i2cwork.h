 
#pragma once

#include "vlapConfig.h"
#include "tracer.h"



typedef enum {I2CWORK_TRANSACTION_CMD_IDLE, I2CWORK_TRANSACTION_CMD_WRITE, I2CWORK_TRANSACTION_CMD_READ, I2CWORK_TRANSACTION_CMD_SKIP } I2cTransactionCmd_T;
typedef enum {I2CWORK_QUEUE_TIME_EVENT, I2CWORK_QUEUE_I2C_COMPLETION_EVENT, I2CWORK_QUEUE_I2C_CALLSELF_EVENT} i2cworkQueueEvent_T;
typedef enum {I2CWORK_SCHEDULER_STATE_TIME_WAIT, I2CWORK_SCHEDULER_STATE_SCAN, I2CWORK_SCHEDULER_STATE_I2C_COMPLETION_WAIT, I2CWORK_SCHEDULER_STATE_I2C_FAULT_PS_OFF_WAIT, I2CWORK_SCHEDULER_STATE_I2C_FAULT_PS_ON_WAIT} i2cworkSchedulerState_T;
typedef enum {I2CWORK_OTHER_REQUEST, I2CWORK_LEDS_REQUEST} i2cworkType_T;
      
typedef enum{ I2CWORK_REQMODE_PERIODIC, I2CWORK_REQMODE_ONESHOT} i2cworkRequestMode_T;

typedef enum{I2CWORK_NOT_DURING_MESSAGE, I2CWORK_DURING_MESSAGE} i2cworkDuringMessageState_T;

#pragma pack(1)
typedef  struct
{
  I2cTransactionCmd_T   Cmd;
  uint8_t               I2cSlaveAddress;
  uint8_t               SlaveRegAddress;
  uint8_t               TranscationLength;
  uint8_t*              TransactionDataPtr;
} i2cworkSchedulerFunctionParameter_T;




#pragma pack(1)
typedef    struct
{
  i2cworkRequestMode_T          Mode;
  i2cworkType_T                 workType;
  uint8_t                       LedIndex;
  uint8_t                       Period;
  ReturnCode_T (*I2cWorkerPreTranscationCallBackPtr)(uint8_t LedIndex, i2cworkSchedulerFunctionParameter_T* ReturnedCommandPtr);
  ReturnCode_T (*I2cWorkerPostTranscationCallBackPtr)(uint8_t LedIndex, i2cworkSchedulerFunctionParameter_T* PostFunctionPtr);
} i2cworkSchedulerEntry_T;

#pragma pack(1)
typedef    struct
{
  uint8_t                 Counter;
  i2cworkSchedulerEntry_T MyEntry;
} i2cworkSchedulerDataBaseEntry_T;


#pragma pack(1)
typedef  struct
{
  i2cworkQueueEvent_T Evnet;
} i2cworlSchedulerQueueEntry_T;

typedef struct{
  volatile uint8_t address; //device address for this job
  volatile uint8_t direction;//direction (I2C_Direction_Transmitter or I2C_Direction_Receiver)
  volatile uint8_t bytes; //number of bytes to read/write
  volatile uint8_t subaddress;//register subaddress for the device - set to 0xFF if no subaddress used (direct read/write)
  volatile uint8_t* data_pointer; //points to the data
  volatile I2CREQ *i2cReq;
} I2C_Job_Type;

void i2cworkConfig(void);

void i2cworkBufferRead(I2C_TypeDef *I2Cx,  I2CREQ *i2cReq);
void i2cworkBufferWrite(I2C_TypeDef *I2Cx, I2CREQ *i2cReqPtr);

void i2c1EndTransactionFromISR(void);
void i2cRespFree(I2CRESP *i2cResp);
void i2cRespFreeFromISR(I2CRESP *i2cResp);
uint8_t i2cRespValid(I2CRESP *i2cResp);
ReturnCode_T i2cworkSchedulerInit();
i2cworkDuringMessageState_T i2cworkDuringMessageStateGet();
portTASK_FUNCTION(i2cworkSchedulerTask, pvParameters );
ReturnCode_T i2cworkSchedulerRegister( i2cworkSchedulerEntry_T* SchedulerEntryPtr);
uint8_t i2cworkSyncByteWrite(I2C_TypeDef *I2Cx, uint8_t *pBuffer, uint8_t deviceAddress, uint8_t WriteAddr);


extern QueueHandle_t i2cworlSchedulerQueueHandle;




#define I2C_DUTYCYCLE             I2C_DutyCycle_2
#define I2C1_SPEED                10000


#define I2C_SHORT_TIMEOUT         ((uint32_t)1000) //0x1000)
#define I2C_LONG_TIMEOUT          ((uint32_t)(10 * I2C_SHORT_TIMEOUT))

/* I2C STOP mask */
#define CR1_STOP_Set            ((uint16_t)0x0200)
#define CR1_STOP_Reset          ((uint16_t)0xFDFF)

/* I2C ACK mask */
#define CR1_ACK_Set             ((uint16_t)0x0400)
#define CR1_ACK_Reset           ((uint16_t)0xFBFF)

/* I2C POS mask */
#define CR1_POS_Set             ((uint16_t)0x0800)
#define CR1_POS_Reset           ((uint16_t)0xF7FF)


#define if_event(EVENT) (event == EVENT)?  #EVENT: 
#define EVENT                                                           \
  if_event(I2C_EVENT_MASTER_BYTE_RECEIVED)                              \
  if_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED)                           \
  if_event(I2C_EVENT_MASTER_BYTE_TRANSMITTING)                          \
  if_event(I2C_EVENT_MASTER_MODE_ADDRESS10)                             \
  if_event(I2C_EVENT_MASTER_MODE_SELECT)                                \
  if_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)                     \
  if_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)                  \
  if_event(I2C_EVENT_SLAVE_ACK_FAILURE)                                 \
  if_event(I2C_EVENT_SLAVE_BYTE_RECEIVED)                               \
  if_event(I2C_EVENT_SLAVE_BYTE_TRANSMITTED)                            \
  if_event(I2C_EVENT_SLAVE_BYTE_TRANSMITTING)                           \
  if_event(I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED)                  \
  if_event(I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED)                    \
  if_event(I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED)              \
  if_event(I2C_EVENT_SLAVE_STOP_DETECTED)                               \
  if_event(I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED)                 \
  if_event(I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED)  "UNKNOWN"





