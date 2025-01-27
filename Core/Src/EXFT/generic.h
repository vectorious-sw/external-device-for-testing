#ifndef GENERIC_H
#define	GENERIC_H
#include "common.h"

// G E N E R I C   T Y P E S   F O R     F R E E   R T O S    I N T E R T A S K   C O M M U N I C A T I O N 

// To be used when intertack communication has no predefined types (for example: spiflash completion message can communicated with different clients, all of must support tge generic Queue Entry in their 
// input queue 
typedef struct 
{
  uint16_t              Type;
  // Optional data ptr
  uint8_t               *DataPtr;
  uint16_t              DataLength;
} genericQueueEntryT;



// G L O B A L  P R O T O T Y P E S 


#endif
