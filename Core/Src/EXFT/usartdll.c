#include <hwdrivers.h>
#include "usartdll.h"
#include "pccommapplayer.h"

TimerHandle_t usartFsmTimerHandler;
static void usartRxFsmTimerCallback(TimerHandle_t pxTimer);

uartDllState_T  uartRxDllState = UART_RX_DLL_IDLE;
uint16_t uartBufferIndex;
uint16_t uartRxPayLoadCount;
uint8_t uartRxCheckSum;
uint16_t MessageLength;
uint8_t uartCommand;

uint16_t usartErrorCounter = 0;

void usartInit()
{
  usartFsmTimerHandler =  xTimerCreate("UsartDllTimer", portTICK_PERIOD_MS, pdFALSE, (void *)0, usartRxFsmTimerCallback);
  xTimerStart(usartFsmTimerHandler, 100);
}
   
void USART_FSM_STATE_CHANGE(uartDllState_T NextState, uint16_t Timeout)
{
  uartRxDllState = NextState;
  xTimerChangePeriod( usartFsmTimerHandler, Timeout, 100);
}

/********************************************************************
 * ReturnCode_T usartDllRxFsm( uint8_t Char, usartdllFsmStimuli_t stimuli)
 *
 *********************************************************************/
ReturnCode_T usartDllRxFsm(uint8_t Char, usartdllFsmStimuli_t stimuli)
{
  // Stop the timeout timer
  xTimerStop(usartFsmTimerHandler, 0);
  
  switch( uartRxDllState)
  {
  case  UART_RX_DLL_IDLE:
    switch(stimuli)
    {
    case USARTDLL_FSM_STIMULI_INCOMING_CHAR:
      switch(Char)
      {
      case 'R':
      case 'F':
      case 'W':
      case 'B':
      case 'S':
      case 'M':
      case 'L':  
        uartRxCheckSum = Char;
        uartBufferIndex = 0;
        UartRxBuffer[uartBufferIndex] = Char;
        uartCommand = Char;
        uartBufferIndex++;
        USART_FSM_STATE_CHANGE(UART_RX_DLL_ADDRESS_MSB_WAIT, USARTDLL_RX_TIMEOUT);
        //uartRxDllState = UART_RX_DLL_ADDRESS_MSB_WAIT;
      } 
      break;
    case USARTDLL_FSM_STIMULI_TIMEOUT:
      // In case of timeout on idle - do nothing
      break;
    }
    break;
  case UART_RX_DLL_ADDRESS_MSB_WAIT:
    switch(stimuli)
    {
    case USARTDLL_FSM_STIMULI_INCOMING_CHAR:
      UartRxBuffer[uartBufferIndex] = Char;
      uartBufferIndex++;
      uartRxCheckSum += Char;
      USART_FSM_STATE_CHANGE(UART_RX_DLL_ADDRESS_LSB_WAIT, USARTDLL_RX_TIMEOUT);
      //uartRxDllState = UART_RX_DLL_ADDRESS_LSB_WAIT;
      break;
    case USARTDLL_FSM_STIMULI_TIMEOUT:
      usartTimeoutHandler();
      break;
    }
    break;
  case UART_RX_DLL_ADDRESS_LSB_WAIT:
    switch(stimuli)
    {
    case USARTDLL_FSM_STIMULI_INCOMING_CHAR:
      UartRxBuffer[uartBufferIndex] = Char;
      uartBufferIndex++;
      uartRxCheckSum += Char;
      MessageLength = 0;
      // Special case - there are two bytes for length
      if((uartCommand == 'M') || (uartCommand == 'L'))
      {
        USART_FSM_STATE_CHANGE(UART_RX_DLL_LENGTH_MSB_WAIT, USARTDLL_RX_TIMEOUT);
        //uartRxDllState = UART_RX_DLL_LENGTH_MSB_WAIT;
      }
      else
      {
        USART_FSM_STATE_CHANGE(UART_RX_DLL_LENGTH_WAIT, USARTDLL_RX_TIMEOUT);
        //uartRxDllState = UART_RX_DLL_LENGTH_WAIT;
      }
      break;
    case USARTDLL_FSM_STIMULI_TIMEOUT:
      usartTimeoutHandler();
      break;
    }
    break;
  case UART_RX_DLL_LENGTH_MSB_WAIT:
    switch(stimuli)
    {
    case USARTDLL_FSM_STIMULI_INCOMING_CHAR:
      UartRxBuffer[uartBufferIndex] = Char;
      uartBufferIndex++;
      uartRxCheckSum += Char;
      MessageLength += 256 * Char;
      // If there was a problem with the message length - go back to idle and ignore this message
      if(MessageLength > UART_RX_BUFFER_LENGTH)
      {
        // Stop the timeout timer due to length problem
        xTimerStop(usartFsmTimerHandler, 1);
        uartRxDllState = UART_RX_DLL_IDLE;
        uartBufferIndex = 0;
        usartErrorCounter++;
        return RETURNCODE_OUT_OF_RANGE;
      }
      USART_FSM_STATE_CHANGE(UART_RX_DLL_LENGTH_WAIT, USARTDLL_RX_TIMEOUT);
      //uartRxDllState = UART_RX_DLL_LENGTH_WAIT;
      break;
    case USARTDLL_FSM_STIMULI_TIMEOUT:
      usartTimeoutHandler();
      break;
    }
    break;
  case UART_RX_DLL_LENGTH_WAIT:
    switch(stimuli)
    {
    case USARTDLL_FSM_STIMULI_INCOMING_CHAR:
      UartRxBuffer[uartBufferIndex] = Char;
      uartBufferIndex++;
      uartRxCheckSum += Char;
      MessageLength += Char;
      switch(uartCommand)
      {
      case 'R':
      case 'F':
      case 'W':
      case 'S':
        USART_FSM_STATE_CHANGE(UART_RX_DLL_CHECKSUM_WAIT, USARTDLL_RX_TIMEOUT);
        //uartRxDllState = UART_RX_DLL_CHECKSUM_WAIT;
        break;
      case 'B':
      case 'M':
      case 'L':  
        // If there was a problem with the message length - go back to idle and ignore this message
        if((MessageLength > UART_RX_BUFFER_LENGTH) || (MessageLength <= 0))
        {
          xTimerStop(usartFsmTimerHandler, 1);
          uartRxDllState = UART_RX_DLL_IDLE;
          uartBufferIndex = 0;
          usartErrorCounter++;
          return RETURNCODE_OUT_OF_RANGE;
        }
        uartRxPayLoadCount = MessageLength;
        USART_FSM_STATE_CHANGE(UART_RX_DLL_PAYLOAD_WAIT, USARTDLL_RX_TIMEOUT);
        //uartRxDllState = UART_RX_DLL_PAYLOAD_WAIT;
        break;
      } 
      break;
    case USARTDLL_FSM_STIMULI_TIMEOUT:
      usartTimeoutHandler();
      break;
    }
    break;
  case UART_RX_DLL_PAYLOAD_WAIT:
    switch(stimuli)
    {
    case USARTDLL_FSM_STIMULI_INCOMING_CHAR:
      UartRxBuffer[uartBufferIndex] = Char;
      uartBufferIndex++;
      uartRxCheckSum += Char;
      if(!--uartRxPayLoadCount)
        USART_FSM_STATE_CHANGE(UART_RX_DLL_CHECKSUM_WAIT, USARTDLL_RX_TIMEOUT);
        //uartRxDllState = UART_RX_DLL_CHECKSUM_WAIT;
      else
        USART_FSM_STATE_CHANGE(UART_RX_DLL_PAYLOAD_WAIT, USARTDLL_RX_TIMEOUT);
      break;
    case USARTDLL_FSM_STIMULI_TIMEOUT:
      usartTimeoutHandler();
      break;
    }
    break;
  case UART_RX_DLL_CHECKSUM_WAIT:
    switch(stimuli)
    {
    case USARTDLL_FSM_STIMULI_INCOMING_CHAR:
      xTimerStop(usartFsmTimerHandler, 1);
      if(Char == uartRxCheckSum)
        pcccommAppLayerProcessing(UartRxBuffer, MessageLength);
      uartRxDllState = UART_RX_DLL_IDLE;
      break;
    case USARTDLL_FSM_STIMULI_TIMEOUT:
      usartTimeoutHandler();
      break;
    }

    break;  
  }
  
  return RETURNCODE_OK;
}

void usartTimeoutHandler()
{
  usartErrorCounter++;
  // Go back to idle
  uartRxDllState = UART_RX_DLL_IDLE;
}

static void usartRxFsmTimerCallback(TimerHandle_t pxTimer)
{
  if(uartRxDllState != UART_RX_DLL_IDLE)
  {
    uint8_t MyBuffer[100]; 
    sprintf(MyBuffer, "usartdll timeout, State: %d", uartRxDllState);
    vlapmainDebugLog(MyBuffer);
    usartDllRxFsm(0, USARTDLL_FSM_STIMULI_TIMEOUT);
  }
}
