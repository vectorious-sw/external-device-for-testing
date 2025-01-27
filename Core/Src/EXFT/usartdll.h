#ifndef USARTDLL_H
#define USARTDLL_H

#include "stdint.h"

#include "common.h"

#define USARTDLL_RX_TIMEOUT 200

// uartdll fsm global vars and definitions
typedef enum {USARTDLL_FSM_STIMULI_INCOMING_CHAR, USARTDLL_FSM_STIMULI_TIMEOUT } usartdllFsmStimuli_t;

typedef enum { 
  UART_RX_DLL_IDLE,
  UART_RX_DLL_ADDRESS_MSB_WAIT,
  UART_RX_DLL_ADDRESS_LSB_WAIT,
  UART_RX_DLL_LENGTH_MSB_WAIT,
  UART_RX_DLL_LENGTH_WAIT, 
  UART_RX_DLL_PAYLOAD_WAIT, 
  UART_RX_DLL_CHECKSUM_WAIT
} uartDllState_T; 

// P R O T O T Y P E S 
ReturnCode_T usartDllRxFsm( uint8_t Char, usartdllFsmStimuli_t stimuli);
void usartInit();
void usartTimeoutHandler();

                                                                            \
  
  
#endif
  
