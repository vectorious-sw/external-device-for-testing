
#ifndef _GENERAL_H_
#define _GENERAL_H_

/*----------------------------------------------------------------------------
*        Headers
*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
*        Consts
*----------------------------------------------------------------------------*/

#define NON_INITIALIZED_VAL	-1

#define WORD_16_BITS	unsigned short
#define WORD_32_BITS	unsigned long

#define INT_16_BITS	short
#define INT_32_BITS	long

#define NOF_ELEMENTS(array) (sizeof(array)/sizeof(array[0]))

#define SUCCESSOR(val,modulo)   ((val == (modulo - 1) ? 0 : val + 1))
#define PREDECESSOR(val,modulo) ((val == 0) ? modulo - 1 : val - 1)

//#define MIN(a,b)        ((a<b)?a:b)

#define DEVELOPMENT_ENV
#ifdef DEVELOPMENT_ENV
  #define LOG_MSG(msg) hwdriversUartDmaTx( msg, strlen(msg) ) 
#else
  #define LOG_MSG(msg)  
#endif
/*----------------------------------------------------------------------------
*        Enumarations
*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
*        Types
*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
*        Exported functions
*----------------------------------------------------------------------------*/

#endif /* #ifndef _VMIC_MODEM_XFACE_H_ */

