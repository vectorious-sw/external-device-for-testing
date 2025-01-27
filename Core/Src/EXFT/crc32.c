#include "crc32.h"

// G L O B A L S 
uint32_t  Crc32Value;

volatile uint32_t CrcCounter; 

/****************************************************************************
 *** uuint32_t crc32ByteCalc(uint32_t crc32Base, uint8_t NewDataByte)
 * This function receives the current CRC32 value and new data byte and generates new 
 * CRC32 result
 *
 *
 *
 *****************************************************************************/
uint32_t crc32ByteCalc(uint32_t crc32Base, uint8_t NewDataByte)
{
  int i, j;
  unsigned int byte, crc, mask;
  int nBytes =1;
  i = 0;
  //crc = 0xFFFFFFFF;
  crc = crc32Base;
  for(i=0; i<nBytes; i++)
  {
    byte = NewDataByte;//message[i];            // Get next byte.
    crc = crc ^ byte;
    for (j = 7; j >= 0; j--) {    // Do eight times.
      mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return crc; //~crc;
}


/****************************************************************************
 *** void crc32BuffCalc()
 *
 *
 *
 *
 *
 *****************************************************************************/
uint32_t crc32BuffCalc(uint8_t * Message, uint32_t Offset, uint32_t  nBytes)
{
  int i, j;
  unsigned int byte, crc, mask;
  uint8_t* MsgPtr =   Message + Offset;
        
  i = 0;
  crc = 0xFFFFFFFF;
  for(i=0; i<nBytes; i++)
  {
    byte = MsgPtr[i];            // Get next byte.
    crc = crc ^ byte;
    for (j = 7; j >= 0; j--) {    // Do eight times.
      mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return ~crc;
}




/****************************************************************************
 *** vuint32_t crc32BuffAccumulate(uint32_t crc32Base, uint8_t * Message, uint32_t Offset, uint32_t  nBytes)
 *
 *
 *
 *
 *
 *****************************************************************************/
uint32_t crc32BuffAccumulate(uint32_t crc32Base, uint8_t * Message, uint32_t Offset, uint32_t  nBytes)
{
#if 0
  int i, j;
  unsigned int byte, crc, mask;
  uint8_t* MsgPtr =   Message + Offset;
        
  CrcCounter += nBytes;
  i = 0;
  crc = crc32Base;
  for(i=0; i<nBytes; i++)
  {
    byte = MsgPtr[i];            // Get next byte.
    crc = crc ^ byte;
    for (j = 7; j >= 0; j--) {    // Do eight times.
      mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return crc;
#endif
  
  int i, j;
  unsigned int byte, crc, mask;
  uint8_t* MsgPtr =   Message + Offset;
  crc = crc32Base;
 
  for(i=0; i<nBytes; i++)
  {
    byte = MsgPtr[i];
    crc = crc ^ byte;
    for (j = 7; j >= 0; j--) {    // Do eight times.
      mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return crc; 

  
  
}





/****************************************************************************
 *** uint32_t crc32ByteInit()
 *
 *
 *
 *
 *
 *****************************************************************************/
uint32_t crc32Init()
{
  CrcCounter = 0;
  // Return the initial CRC32 value
  return(0xFFFFFFFF);
}
