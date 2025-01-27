#ifndef AESCBC_H
#define AESCBC_H
#include "config.h"
#include "crypto.h"


// D E F I N I T I O N S  
#define AESCBC_ENCRYPT_TBD_FIELD_LENGTH 31
#define AESCBC_DECRYPT_MAX_LENGTH 5000

// Encryption header  - 70 bytes
#pragma pack(1)
typedef  struct   {
 uint8_t VlapExUniqId[12];
 uint8_t IvArray[16];
 uint16_t  EncryptedMsgLength;
 uint32_t  OriginalMsgCrc32;           // MSByte first, Not to be confused with Dll CRC32
 uint32_t  TransmissionTimeStamp;                  // Transmission TimeStamp (Number of seconds since Epoch, 1970-01-01 00:00:00 +0000 (UTC))
 uint8_t  TransmissionTimeStamp10mSec;            // Transmission TimeStamp (0-99 Tenth of a Second Resolution for the LinuxTimeOfDate seconds variable) 
 uint8_t  EncryptedTbd[AESCBC_ENCRYPT_TBD_FIELD_LENGTH];
} aescbcProtcolHeader_t;



// P RO T O T Y P E S 

void aescbc();
void aescbcInit();
ReturnCode_T aescbcEncrypt(uint8_t* MessagePtr, uint16_t MessageLength, uint16_t* ReturnedEncryptedMsgLengthPtr);
ReturnCode_T aescbcDecrypt(uint8_t* EncryptedMsgPtr, uint8_t* DecryptedMsgPtr, uint16_t MessageLengthr, uint8_t* key, uint8_t verifyCRC);
ReturnCode_T aescbcEncryptConfig(configNvramConfigurationEncryptedDb_t* EncryptConfigPtr);
ReturnCode_T aescbcDecryptConfig(configNvramConfigurationEncryptedDb_t* EncryptConfigPtr);
ReturnCode_T aescbcEncryptByChunks(uint8_t* MessagePtr, uint16_t MessageLength, uint16_t*  ReturnedEncryptedMsgLengthPtr);
void aescbcTestInit();
void aescbcTest();

#endif
