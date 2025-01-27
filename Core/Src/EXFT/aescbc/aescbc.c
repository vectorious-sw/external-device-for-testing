
//#include "types.h"
//#include <aesConfig.h>
#include "stm32h7xx.h"
#include <stdlib.h>
#include "FreeRTOS.h"
#include "crypto.h"
#include "config.h"
#include "uartdll.h"
#include "aescbc.h"
#include "protocolapp.h"
#include "inet.h"
#include "crc32.h"
#include "cmox_crypto.h"

// D E F I N I T I O N S  

#define AESCBC_ENCRYPTION_CHUNK         16

// P R O T O T Y P E S 
void aescbcInit();
ReturnCode_T aescbcEncryptByChunksTest(uint8_t* MessagePtr, uint16_t MessageLength, uint16_t*  ReturnedEncryptedMsgLengthPtr);

int32_t STM32_AES_CBC_Decrypt(uint8_t* InputMessage,
                              uint32_t InputMessageLength,
                              uint8_t  *AES256_Key,
                              uint8_t  *InitializationVector,
                              uint32_t  IvLength,
                              uint8_t  *OutputMessage,
                              uint32_t *OutputMessageLength);


// G L O B A L S 
volatile uint32_t aescbcCounter = 0;
// Structure that will keep the random state 
RNGstate_stt RNGstate;
// Structure for the parmeters of initialization
RNGinitInput_stt RNGinit_st;
// Nonce
uint8_t nonce[4] = {0,1,2,3};
// IV
uint8_t IV[16];





// String of entropy
uint8_t entropy_data[32]={0x9d,0x20,0x1a,0x18,0x9b,0x6d,0x1a,0xa7,0x0e,0x79,0x57,0x6f,0x36,0xb6,0xaa,0x88,0x55,0xfd,0x4a,0x7f,0x97,0xe9,0x71,0x69,0xb6,0x60,0x88,0x78,0xe1,0x9c,0x8b,0xa5};

uint32_t OutputEncryptedMessageLength = 0;
uint32_t OutputMessageLength = 0;




volatile int32_t RngInitStatus;

void aescbcInit()
{
  /* DeInitialize STM32 Cryptographic Library */
  //Crypto_DeInit();
  
  // RNG_Cmd(ENABLE);
  
  // Initialize the RNGinit structure 
  RNGinit_st.pmEntropyData = entropy_data;
  RNGinit_st.mEntropyDataSize = sizeof(entropy_data);
  RNGinit_st.pmNonce = nonce;
  RNGinit_st.mNonceSize = sizeof(nonce);
  // There is no personalization data in this case
  RNGinit_st.mPersDataSize = 0;
  RNGinit_st.pmPersData = NULL;
  // Init the random engine
   RngInitStatus = RNGinit(&RNGinit_st, &RNGstate);



}



RNGaddInput_stt  OptionalAditionalInput; 




/****************************************************************************
*** ReturnCode_T aescbcEncryptByChunks(uint8_t* MessagePtr, uint16_t MessageLength, uint16_t*  ReturnedEncryptedMsgLengthPtr)
* The source buffer (The original non encrypted buffer has no encryption header, it is just the SPI event
* The decrypted buffer will be over written by the encrypted buffer
*
*****************************************************************************/
ReturnCode_T aescbcEncryptByChunks(uint8_t* MessagePtr, uint16_t MessageLength, uint16_t*  ReturnedEncryptedMsgLengthPtr)
{
  // This function assumes MessagePtr points to protocol message (ie, to the begining of the DLLHeader and that the message has prealocated encryption header between the Dll and the application layer 
  ReturnCode_T MyReturn = RETURNCODE_ERROR;
  uint8_t* StartPtr;
  uint8_t* SourceDataPtr;
  uint8_t* EncryptedMessagePtr;
  uint8_t* EncryptedChunkPtr = 0;
  volatile uint32_t EncryptMsgPayloadLength;
  uint32_t OriginalMsgCrc32; 
  
  AESCBCctx_stt AESctx;
  uint32_t error_status = AES_SUCCESS;
  int32_t outputLength = 0;
  uint32_t i;
  
  
  // Generate IV 
  if ( RNGgenBytes(&RNGstate,0, (uint8_t*)IV,sizeof(IV)) == RNG_SUCCESS)
  {
    // MEssage length 
    EncryptMsgPayloadLength = MessageLength;
    // Calculate the CRC32 of the original message before padding and encryption 
    OriginalMsgCrc32  = crc32BuffCalc(MessagePtr/*+sizeof(DllHeader_t)*/+sizeof(aescbcProtcolHeader_t), 0, MessageLength);
    
    // Pad message to %16 
    if(EncryptMsgPayloadLength % 16)
      EncryptMsgPayloadLength = ((EncryptMsgPayloadLength/16) + 1)* 16;
    else
      EncryptMsgPayloadLength = EncryptMsgPayloadLength;
    
    // Set flag field to default value 
    AESctx.mFlags = E_SK_DEFAULT;
    // Set key size to 32 (corresponding to AES-256) 
    AESctx.mKeySize =  CRL_AES256_KEY;
    // Set iv size field to IvLength
    AESctx.mIvSize = 16;
    // Initialize the operation, by passing the key.
    // Third parameter is NULL because CBC doesn't use any IV 
    error_status = AES_CBC_Encrypt_Init(&AESctx, configProductionDb.UpStreamAesCbcKey, IV);
    
    EncryptedChunkPtr = pvPortMalloc(AESCBC_ENCRYPTION_CHUNK);
    if(EncryptedChunkPtr && (error_status == AES_SUCCESS))
    {
      // sets the payload data ptr, we assume the message 
      
      StartPtr = MessagePtr/*+sizeof(DllHeader_t)*/+sizeof(aescbcProtcolHeader_t); 
      SourceDataPtr = StartPtr; 
      EncryptedMessagePtr = MessagePtr/*+sizeof(DllHeader_t)*/+sizeof(aescbcProtcolHeader_t);
      // Go over the buffer chunk by chunk, encrypt into the chunk size allocated memory and copy back to the original buffer
      for(i=0; i<EncryptMsgPayloadLength; i+=AESCBC_ENCRYPTION_CHUNK, SourceDataPtr+=AESCBC_ENCRYPTION_CHUNK, EncryptedMessagePtr+=AESCBC_ENCRYPTION_CHUNK)
      {
        // Encrypt chunk 
        error_status = AES_CBC_Encrypt_Append(&AESctx, SourceDataPtr, AESCBC_ENCRYPTION_CHUNK, EncryptedChunkPtr, &outputLength);
        if (error_status == AES_SUCCESS)
          memcpy(EncryptedMessagePtr, EncryptedChunkPtr, AESCBC_ENCRYPTION_CHUNK);
        else
          break;
      }
      // Check if we got break before the for loop completed
      if(i==EncryptMsgPayloadLength)
      {
        // Do the Finalization */
        error_status = AES_CBC_Encrypt_Finish(&AESctx, StartPtr + outputLength, &outputLength);
        // Add data written to the information to be returned */
        *ReturnedEncryptedMsgLengthPtr += outputLength;
        
        // Fill the encryption header
        aescbcProtcolHeader_t* EncryptionHeaderPtr = (aescbcProtcolHeader_t*)(MessagePtr/*+sizeof(DllHeader_t)*/);
        // TODO: change 12 to general definition
        memcpy(EncryptionHeaderPtr->VlapExUniqId, protocolappUniqueIdPtrGet(), 12);
        // Fill the IV
        memcpy(EncryptionHeaderPtr->IvArray, IV, sizeof(IV));
        // Fill the 
        // TODO: TBD field currently filled with zeros
        memset(EncryptionHeaderPtr->EncryptedTbd, 0, AESCBC_ENCRYPT_TBD_FIELD_LENGTH);
        // Transmission timestamp will be over written when the transmission process gets the entry out of the log memory
        EncryptionHeaderPtr->TransmissionTimeStamp = HTONL(rtcEpochGet());
        EncryptionHeaderPtr->TransmissionTimeStamp10mSec = 0;
        // Fill the original message CRC32 to the encryption header
        EncryptionHeaderPtr->OriginalMsgCrc32 = HTONL(OriginalMsgCrc32);
       // Fill the actual payload length
        EncryptionHeaderPtr->EncryptedMsgLength = HTONS(MessageLength);
        *ReturnedEncryptedMsgLengthPtr = (uint16_t)(/*sizeof(DllHeader_t)*/ + sizeof(aescbcProtcolHeader_t) + EncryptMsgPayloadLength /*+ sizeof(DllEndOfMessage_t)*/);
        
        MyReturn = RETURNCODE_OK;
      }
      else
        MyReturn = RETURNCODE_ERROR;
      
    }
    else
      MyReturn = RETURNCODE_ERROR;
    
  }
  else
    MyReturn = RETURNCODE_ERROR;
  
  if(EncryptedChunkPtr)
    vPortFree(EncryptedChunkPtr);
  return(MyReturn);
}
    
/****************************************************************************
*** ReturnCode_T aescbcEncryptByChunks(uint8_t* MessagePtr, uint16_t MessageLength, uint16_t*  ReturnedEncryptedMsgLengthPtr)
* The source buffer (The original non encrypted buffer has no encryption header, it is just the SPI event
* The decrypted buffer will be over written by the encrypted buffer
*
*****************************************************************************/
ReturnCode_T aescbcEncryptByChunksTest(uint8_t* MessagePtr, uint16_t MessageLength, uint16_t*  ReturnedEncryptedMsgLengthPtr)
{
  // This function assumes MessagePtr points to protocol message (ie, to the begining of the DLLHeader and that the message has prealocated encryption header between the Dll and the application layer
  ReturnCode_T MyReturn = RETURNCODE_ERROR;
  uint8_t* StartPtr;
  uint8_t* SourceDataPtr;
  uint8_t* EncryptedMessagePtr;
  uint8_t* EncryptedChunkPtr = 0;
  volatile uint32_t EncryptMsgPayloadLength;
  uint32_t OriginalMsgCrc32;

  AESCBCctx_stt AESctx;
  uint32_t error_status = AES_SUCCESS;
  int32_t outputLength = 0;
  uint32_t i;


  // Generate IV
  if ( RNGgenBytes(&RNGstate,0, (uint8_t*)IV,sizeof(IV)) == RNG_SUCCESS)
  {
    // MEssage length
    EncryptMsgPayloadLength = MessageLength;

    // Pad message to %16
    if(EncryptMsgPayloadLength % 16)
      EncryptMsgPayloadLength = ((EncryptMsgPayloadLength/16) + 1)* 16;
    else
      EncryptMsgPayloadLength = EncryptMsgPayloadLength;

    // Set flag field to default value
    AESctx.mFlags = E_SK_DEFAULT;
    // Set key size to 32 (corresponding to AES-256)
    AESctx.mKeySize =  CRL_AES256_KEY;
    // Set iv size field to IvLength
    AESctx.mIvSize = 16;
    // Initialize the operation, by passing the key.
    // Third parameter is NULL because CBC doesn't use any IV
    error_status = AES_CBC_Encrypt_Init(&AESctx, configProductionDb.UpStreamAesCbcKey, IV);

    EncryptedChunkPtr = pvPortMalloc(AESCBC_ENCRYPTION_CHUNK);
    if(EncryptedChunkPtr && (error_status == AES_SUCCESS))
    {
      // sets the payload data ptr, we assume the message

      SourceDataPtr = MessagePtr;
      EncryptedMessagePtr = MessagePtr;
      // Go over the buffer chunk by chunk, encrypt into the chunk size allocated memory and copy back to the original buffer
      for(i=0; i<EncryptMsgPayloadLength; i+=AESCBC_ENCRYPTION_CHUNK, SourceDataPtr+=AESCBC_ENCRYPTION_CHUNK, EncryptedMessagePtr+=AESCBC_ENCRYPTION_CHUNK)
      {
        // Encrypt chunk
        error_status = AES_CBC_Encrypt_Append(&AESctx, SourceDataPtr, AESCBC_ENCRYPTION_CHUNK, EncryptedChunkPtr, &outputLength);
        if (error_status == AES_SUCCESS)
          memcpy(EncryptedMessagePtr, EncryptedChunkPtr, AESCBC_ENCRYPTION_CHUNK);
        else
          break;
      }
      // Check if we got break before the for loop completed
      if(i==EncryptMsgPayloadLength)
      {
        // Do the Finalization */
        error_status = AES_CBC_Encrypt_Finish(&AESctx, StartPtr + outputLength, &outputLength);
        // Add data written to the information to be returned */
        *ReturnedEncryptedMsgLengthPtr += outputLength;

        // Fill the encryption header
        *ReturnedEncryptedMsgLengthPtr = (uint16_t)(EncryptMsgPayloadLength);

        MyReturn = RETURNCODE_OK;
      }
      else
        MyReturn = RETURNCODE_ERROR;

    }
    else
      MyReturn = RETURNCODE_ERROR;

  }
  else
    MyReturn = RETURNCODE_ERROR;

  if(EncryptedChunkPtr)
    vPortFree(EncryptedChunkPtr);
  return(MyReturn);
}





/****************************************************************************
ReturnCode_T aescbcDecrypt(uint8_t* EncryptedMsgPtr, uint8_t* DecryptedMsgPtr, uint16_t MessageLength, uint8_t* key, uint8_t verifyCRC)
*
*
*****************************************************************************/
ReturnCode_T aescbcDecrypt(uint8_t* EncryptedMsgPtr, uint8_t* DecryptedMsgPtr, uint16_t MessageLength, uint8_t* key, uint8_t verifyCRC)
{
  // This function assumes MessagePtr points to protocol message (ie, to the beginning of the DLLHeader and that the message has prealocated encryption header between the Dll and the application layer
  
  // Point to encryption header start
  aescbcProtcolHeader_t* EncryptionHeaderPtr = (aescbcProtcolHeader_t*)(EncryptedMsgPtr);
  uint16_t decryptLength = NTOHS(EncryptionHeaderPtr->EncryptedMsgLength);
  uint32_t originalMsgCrc32 = NTOHL(EncryptionHeaderPtr->OriginalMsgCrc32);
  
  STM32_AES_CBC_Decrypt(EncryptedMsgPtr+sizeof(aescbcProtcolHeader_t), MessageLength, key, EncryptionHeaderPtr->IvArray, 16, DecryptedMsgPtr, &OutputMessageLength);
  
  if(verifyCRC)
  {
    if(decryptLength > AESCBC_DECRYPT_MAX_LENGTH)
      return RETURNCODE_ERROR;
    
    uint32_t decryptedCRC = crc32BuffCalc((uint8_t*) DecryptedMsgPtr, 0, decryptLength);
    if(decryptedCRC != originalMsgCrc32)
    {
      return RETURNCODE_ERROR;
    }
  }
  
  return(RETURNCODE_OK);
}






/**
* @brief  AES CBC Decryption example.
* @param  InputMessage: pointer to input message to be decrypted.
* @param  InputMessageLength: input data message length in byte.
* @param  AES256_Key: pointer to the AES key to be used in the operation
* @param  InitializationVector: pointer to the Initialization Vector (IV)
* @param  IvLength: IV length in bytes.
* @param  OutputMessage: pointer to output parameter that will handle the decrypted message
* @param  OutputMessageLength: pointer to decrypted message length.
* @retval error status: can be AES_SUCCESS if success or one of
*         AES_ERR_BAD_INPUT_SIZE, AES_ERR_BAD_OPERATION, AES_ERR_BAD_CONTEXT
*         AES_ERR_BAD_PARAMETER if error occured.
*/
int32_t STM32_AES_CBC_Decrypt(uint8_t* InputMessage,
                              uint32_t InputMessageLength,
                              uint8_t  *AES256_Key,
                              uint8_t  *InitializationVector,
                              uint32_t  IvLength,
                              uint8_t  *OutputMessage,
                              uint32_t *OutputMessageLength)
{
  AESCBCctx_stt AESctx;

  uint32_t error_status = AES_SUCCESS;

  int32_t outputLength = 0;

  /* Set flag field to default value */
  AESctx.mFlags = E_SK_DEFAULT;

  /* Set key size to 32 (corresponding to AES-256) */
  AESctx.mKeySize = 32;

  /* Set iv size field to IvLength*/
  AESctx.mIvSize = IvLength;

  /* Initialize the operation, by passing the key.
  * Third parameter is NULL because CBC doesn't use any IV */
  error_status = AES_CBC_Decrypt_Init(&AESctx, AES256_Key, InitializationVector );

  /* check for initialization errors */
  if (error_status == AES_SUCCESS)
  {
    /* Decrypt Data */
    error_status = AES_CBC_Decrypt_Append(&AESctx,
                                          InputMessage,
                                          InputMessageLength,
                                          OutputMessage,
                                          &outputLength);

    if (error_status == AES_SUCCESS)
    {
      /* Write the number of data written*/
      *OutputMessageLength = outputLength;
      /* Do the Finalization */
      error_status = AES_CBC_Decrypt_Finish(&AESctx, OutputMessage + *OutputMessageLength, &outputLength);
      /* Add data written to the information to be returned */
      *OutputMessageLength += outputLength;
    }
  }

  return error_status;
}

/****************************************************************************
*** ReturnCode_T aescbcEncryptConfig(configNvramConfigurationEncryptedDb_t* EncryptConfigPtr)
*
*
*****************************************************************************/
ReturnCode_T aescbcEncryptConfig(configNvramConfigurationEncryptedDb_t* EncryptConfigPtr)
{
  ReturnCode_T MyReturn = RETURNCODE_ERROR;
  uint8_t* StartPtr;
  uint8_t* SourceDataPtr = 0;
  uint8_t* EncryptedMessagePtr = 0;
  uint8_t* EncryptedChunkPtr = 0;
  volatile uint32_t EncryptMsgPayloadLength;
  
  AESCBCctx_stt AESctx;
  uint32_t error_status = AES_SUCCESS;
  int32_t outputLength = 0;
  uint32_t i;
   




  // Generate IV 
  int32_t RngGenStatus = RNGgenBytes(&RNGstate,0, (uint8_t*)IV,sizeof(IV));
  if (RngGenStatus == RNG_SUCCESS)
  {    
    // Encrypt the configuration Db(including the CRC)
    EncryptMsgPayloadLength = sizeof(configNvramConfigurationDb_t);
    
    // Pad message to %16 
    if(EncryptMsgPayloadLength % 16)
      EncryptMsgPayloadLength = ((EncryptMsgPayloadLength/16) + 1)* 16;
    else
      EncryptMsgPayloadLength = EncryptMsgPayloadLength;
    
    // Calculate the number of padded bytes
    EncryptConfigPtr->paddingSize = EncryptMsgPayloadLength - sizeof(configNvramConfigurationDb_t);
    // Fill the IV
    memcpy(EncryptConfigPtr->IvArray, IV, sizeof(IV));   
    
    // Set flag field to default value 
    AESctx.mFlags = E_SK_DEFAULT;
    // Set key size to 32 (corresponding to AES-256) 
    AESctx.mKeySize =  CRL_AES256_KEY;
    // Set iv size field to IvLength
    AESctx.mIvSize = 16;
    // Initialize the operation, by passing the key.
    // Third parameter is NULL because CBC doesn't use any IV 
    error_status = AES_CBC_Encrypt_Init(&AESctx, configProductionDb.DownStreamAesCbcKey, IV);
    
    EncryptedChunkPtr = pvPortMalloc(AESCBC_ENCRYPTION_CHUNK);
    if(EncryptedChunkPtr && (error_status == AES_SUCCESS))
    {
      // sets the payload data ptr to the beginning of the config sturct 
      StartPtr = (uint8_t*)EncryptConfigPtr; 
      SourceDataPtr = StartPtr; 
      EncryptedMessagePtr = (uint8_t*)StartPtr;
      // Go over the buffer chunk by chunk, encrypt into the chunk size allocated memory and copy back to the original buffer
      for(i=0; i<EncryptMsgPayloadLength; i+=AESCBC_ENCRYPTION_CHUNK, SourceDataPtr+=AESCBC_ENCRYPTION_CHUNK, EncryptedMessagePtr+=AESCBC_ENCRYPTION_CHUNK)
      {
        // Encrypt chunk 
        error_status = AES_CBC_Encrypt_Append(&AESctx, SourceDataPtr, AESCBC_ENCRYPTION_CHUNK, EncryptedChunkPtr, &outputLength);
        if (error_status == AES_SUCCESS)
          memcpy(EncryptedMessagePtr, EncryptedChunkPtr, AESCBC_ENCRYPTION_CHUNK);
        else
          break;
      }
      // Check if we got break before the for loop completed
      if(i==EncryptMsgPayloadLength)
      {
        // Do the Finalization */
        error_status = AES_CBC_Encrypt_Finish(&AESctx, StartPtr + outputLength, &outputLength);     
        MyReturn = RETURNCODE_OK;
      }
      else
        MyReturn = RETURNCODE_ERROR;
      
    }
    else
      MyReturn = RETURNCODE_ERROR;
    
  }
//  else
//    MyReturn = RETURNCODE_ERROR;
  
  if(EncryptedChunkPtr)
    vPortFree(EncryptedChunkPtr);
  return(MyReturn);  
}

/****************************************************************************
*** ReturnCode_T aescbcDecryptConfig(configNvramConfigurationEncryptedDb_t* EncryptConfigPtr)
*
*
*****************************************************************************/
ReturnCode_T aescbcDecryptConfig(configNvramConfigurationEncryptedDb_t* EncryptConfigPtr)
{
  ReturnCode_T MyReturn = RETURNCODE_ERROR;
  uint8_t* StartPtr;
  uint8_t* SourceDataPtr;
  uint8_t* DecryptedMessagePtr;
  uint8_t* DecryptedChunkPtr;
  volatile uint32_t EncryptMsgPayloadLength;

  AESCBCctx_stt AESctx;
  uint32_t error_status = AES_SUCCESS;
  int32_t outputLength = 0;
  uint32_t i;
   
  // Decrypt length
  EncryptMsgPayloadLength = sizeof(configNvramConfigurationDb_t) + EncryptConfigPtr->paddingSize;
  
  // Set flag field to default value 
  AESctx.mFlags = E_SK_DEFAULT;
  // Set key size to 32 (corresponding to AES-256) 
  AESctx.mKeySize =  CRL_AES256_KEY;
  // Set iv size field to IvLength
  AESctx.mIvSize = 16;
  // Initialize the operation, by passing the key.
  error_status = AES_CBC_Decrypt_Init(&AESctx, configProductionDb.DownStreamAesCbcKey, EncryptConfigPtr->IvArray);
  
  DecryptedChunkPtr = pvPortMalloc(AESCBC_ENCRYPTION_CHUNK);
  if(DecryptedChunkPtr && (error_status == AES_SUCCESS))
  {
    // sets the payload data ptr to the beginning of the config sturct 
    StartPtr = (uint8_t*)EncryptConfigPtr; 
    SourceDataPtr = StartPtr; 
    DecryptedMessagePtr = (uint8_t*)StartPtr;
    // Go over the buffer chunk by chunk, encrypt into the chunk size allocated memory and copy back to the original buffer
    for(i=0; i<EncryptMsgPayloadLength; i+=AESCBC_ENCRYPTION_CHUNK, SourceDataPtr+=AESCBC_ENCRYPTION_CHUNK, DecryptedMessagePtr+=AESCBC_ENCRYPTION_CHUNK)
    {
      // Encrypt chunk 
      error_status = AES_CBC_Decrypt_Append(&AESctx, SourceDataPtr, AESCBC_ENCRYPTION_CHUNK, DecryptedChunkPtr, &outputLength);
      if (error_status == AES_SUCCESS)
        memcpy(DecryptedMessagePtr, DecryptedChunkPtr, AESCBC_ENCRYPTION_CHUNK);
      else
        break;
    }
    // Check if we got break before the for loop completed
    if(i==EncryptMsgPayloadLength)
    {
      // Do the Finalization */
      error_status = AES_CBC_Decrypt_Finish(&AESctx, StartPtr + outputLength, &outputLength);
    }
    else
      MyReturn = RETURNCODE_ERROR; 
  }
  else
    MyReturn = RETURNCODE_ERROR;
  
  
  if(DecryptedChunkPtr)
    vPortFree(DecryptedChunkPtr);
  return(MyReturn);  
}




uint8_t Key[CRL_AES256_KEY] =
{
  0x8e, 0x73, 0xb0, 0xf7, 0xda, 0x0e, 0x64, 0x52,
  0xc8, 0x10, 0xf3, 0x2b, 0x80, 0x90, 0x79, 0xe5,
  0x62, 0xf8, 0xea, 0xd2, 0x52, 0x2c, 0x6b, 0x7b,
  0x64, 0x54, 0xac, 0x54, 0x99, 0x1d, 0xed, 0x7b,
};


uint8_t Plaintext[] =
{
  0x6c, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
  0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
  0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
  0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
  0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
  0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
  0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
  0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10,

};


uint8_t Plaintext1[] =
{
  0x6c, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
  0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
  0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
  0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
  0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
  0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
  0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
  0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10,

};


uint32_t ReturnedLength;
uint8_t OutGG[50];

void aescbcTest()
{


	aescbcEncryptByChunksTest(&Plaintext1, sizeof(Plaintext1), (uint16_t*)&ReturnedLength);


}




#if 0

/* Buffer to store the output data */
uint8_t OutputEncryptMessage[sizeof(Plaintext)];
uint8_t OutputMessage[sizeof(Plaintext)];


// String of entropy
uint8_t entropy_data[32]={0x9d,0x20,0x1a,0x18,0x9b,0x6d,0x1a,0xa7,0x0e,0x79,0x57,0x6f,0x36,0xb6,0xaa,0x88,0x55,0xfd,0x4a,0x7f,0x97,0xe9,0x71,0x69,0xb6,0x60,0x88,0x78,0xe1,0x9c,0x8b,0xa5};
// Nonce
uint8_t nonce[4] = {0,1,2,3};

uint8_t IV[16];

/* Key to be used for AES encryption/decryption */
uint8_t Key[CRL_AES256_KEY] =
{
  0x8e, 0x73, 0xb0, 0xf7, 0xda, 0x0e, 0x64, 0x52,
  0xc8, 0x10, 0xf3, 0x2b, 0x80, 0x90, 0x79, 0xe5,
  0x62, 0xf8, 0xea, 0xd2, 0x52, 0x2c, 0x6b, 0x7b,
  0x64, 0x54, 0xac, 0x54, 0x99, 0x1d, 0xed, 0x7b,
};


const uint8_t Plaintext[] =
{
  0x6c, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
  0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
  0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
  0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
  0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
  0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
  0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
  0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10,
  
};


/* Initialization Vector */
uint8_t IV1[CRL_AES_BLOCK] =
{
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
};



/** @addtogroup STM32_Crypto_Examples
* @{
*/

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* Private define ------------------------------------------------------------*/
#define PLAINTEXT_LENGTH 128 + 32
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint8_t Plaintext[] =
{
  0x6c, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
  0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
  0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
  0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
  0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
  0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
  0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
  0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10,
  
};

/* Key to be used for AES encryption/decryption */
uint8_t Key[CRL_AES256_KEY] =
{
  0x8e, 0x73, 0xb0, 0xf7, 0xda, 0x0e, 0x64, 0x52,
  0xc8, 0x10, 0xf3, 0x2b, 0x80, 0x90, 0x79, 0xe5,
  0x62, 0xf8, 0xea, 0xd2, 0x52, 0x2c, 0x6b, 0x7b,
  0x64, 0x54, 0xac, 0x54, 0x99, 0x1d, 0xed, 0x7b,
};

/* Initialization Vector */
uint8_t IV1[CRL_AES_BLOCK] =
{
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
};


/* Buffer to store the output data */
uint8_t OutputEncryptMessage[sizeof(Plaintext)];
uint8_t OutputMessage[sizeof(Plaintext)];

/* Size of the output data */
uint32_t OutputEncryptedMessageLength = 0;
uint32_t OutputMessageLength = 0;

const uint8_t Expected_Ciphertext[] =
{
  0x4f, 0x02, 0x1d, 0xb2, 0x43, 0xbc, 0x63, 0x3d,
  0x71, 0x78, 0x18, 0x3a, 0x9f, 0xa0, 0x71, 0xe8,
  0xb4, 0xd9, 0xad, 0xa9, 0xad, 0x7d, 0xed, 0xf4,
  0xe5, 0xe7, 0x38, 0x76, 0x3f, 0x69, 0x14, 0x5a,
  0x57, 0x1b, 0x24, 0x20, 0x12, 0xfb, 0x7a, 0xe0,
  0x7f, 0xa9, 0xba, 0xac, 0x3d, 0xf1, 0x02, 0xe0,
  0x08, 0xb0, 0xe2, 0x79, 0x88, 0x59, 0x88, 0x81,
  0xd9, 0x20, 0xa9, 0xe6, 0x4f, 0x56, 0x15, 0xcd,
};



//// Random Bytes generation
/* Structure that will keep the random state */
RNGstate_stt RNGstate;
/* Structure for the parmeters of initialization */
RNGinitInput_stt RNGinit_st;
/* String of entropy */
//entropy_data[32]={0x9d,0x20,0x1a,0x18,0x9b,0x6d,0x1a,0xa7,0x0e,0x79,0x57,0x6f,0x36,0xb6,0xaa,0x88,0x55,0xfd,0x4a,0x7f,0x97,0xe9,0x71,0x69,0xb6,0x60,0x88,0x78,0xe1,0x9c,0x8b,0xa5};
/* Nonce */
// uint8_t nonce[4] = {0,1,2,3};
/* array to keep the returned random bytes */
uint8_t IV[16];




/* Private function prototypes -----------------------------------------------*/
int32_t ttSTM32_AES_CBC_Encrypt(uint8_t*  InputMessage,
                                uint32_t  InputMessageLength,
                                uint8_t  *AES256_Key,
                                uint8_t  *InitializationVector,
                                uint32_t  IvLength,
                                uint8_t  *OutputEncryptMessage,
                                uint32_t *OutputMessageLength);

int32_t ttSTM32_AES_CBC_Decrypt(uint8_t*  InputMessage,
                                uint32_t  InputMessageLength,
                                uint8_t  *AES256_Key,
                                uint8_t  *InitializationVector,
                                uint32_t  IvLength,
                                uint8_t  *OutputMessage,
                                uint32_t *OutputMessageLength);

TestStatus Buffercmp(const uint8_t* pBuffer,
                     uint8_t* pBuffer1,
                     uint16_t BufferLength);
/* Private functions ---------------------------------------------------------*/






void aescbc(void)
{
  int32_t status = AES_SUCCESS;
  
  //Generate
  RNGgenBytes(&RNGstate,0,IV,sizeof(IV));
  
  
  /* Encrypt DATA with AES in CBC mode */
  status = ttSTM32_AES_CBC_Encrypt( (uint8_t *) Plaintext, sizeof(Plaintext), Key, IV, sizeof(IV), OutputEncryptMessage,
                                   &OutputEncryptedMessageLength);
  if (status == AES_SUCCESS)
  {
    if (Buffercmp(Expected_Ciphertext, OutputMessage, sizeof(Plaintext)) == PASSED)
    {
      /* add application traintment in case of AES CBC encrption is passed */   
      aescbcCounter++;   
    }
    else
    {
      /* add application traintment in case of AES CBC encrption is failed */
    }
  }
  else
  {
    /* Add application traintment in case of encryption/decryption not success possible values
    *  of status:
    * AES_ERR_BAD_CONTEXT, AES_ERR_BAD_PARAMETER, AES_ERR_BAD_INPUT_SIZE, AES_ERR_BAD_OPERATION
    */
  }
  /* Decrypt DATA with AES in CBC mode */
  status = ttSTM32_AES_CBC_Decrypt( (uint8_t *) OutputEncryptMessage, sizeof(Plaintext), Key, IV, sizeof(IV), OutputMessage,
                                   &OutputMessageLength);
  if (status == AES_SUCCESS)
  {
    if (Buffercmp(Plaintext, OutputMessage, sizeof(Plaintext)) == PASSED)
    {
      /* add application traintment in case of AES CBC encrption is passed */ 
      aescbcCounter++;   
      
      
    }
    else
    {
      /* add application traintment in case of AES CBC encrption is failed */
    }
  }
  else
  {
    /* Add application traintment in case of encryption/decryption not success possible values
    *  of status:
    * AES_ERR_BAD_CONTEXT, AES_ERR_BAD_PARAMETER, AES_ERR_BAD_INPUT_SIZE, AES_ERR_BAD_OPERATION
    */
  }
}

/**
* @brief  AES CBC Encryption example.
* @param  InputMessage: pointer to input message to be encrypted.
* @param  InputMessageLength: input data message length in byte.
* @param  AES256_Key: pointer to the AES key to be used in the operation
* @param  InitializationVector: pointer to the Initialization Vector (IV)
* @param  IvLength: IV length in bytes.
* @param  OutputMessage: pointer to output parameter that will handle the encrypted message
* @param  OutputMessageLength: pointer to encrypted message length.
* @retval error status: can be AES_SUCCESS if success or one of
*         AES_ERR_BAD_INPUT_SIZE, AES_ERR_BAD_OPERATION, AES_ERR_BAD_CONTEXT
*         AES_ERR_BAD_PARAMETER if error occured.
*/
int32_t ttSTM32_AES_CBC_Encrypt(uint8_t* InputMessage,
                                uint32_t InputMessageLength,
                                uint8_t  *AES256_Key,
                                uint8_t  *InitializationVector,
                                uint32_t  IvLength,
                                uint8_t  *OutputMessage,
                                uint32_t *OutputMessageLength)
{
  AESCBCctx_stt AESctx;
  
  uint32_t error_status = AES_SUCCESS;
  
  int32_t outputLength = 0;
  
  /* Set flag field to default value */
  AESctx.mFlags = E_SK_DEFAULT;
  
  /* Set key size to 32 (corresponding to AES-256) */
  AESctx.mKeySize =  CRL_AES256_KEY;
  
  /* Set iv size field to IvLength*/
  AESctx.mIvSize = IvLength;
  
  /* Initialize the operation, by passing the key.
  * Third parameter is NULL because CBC doesn't use any IV */
  error_status = AES_CBC_Encrypt_Init(&AESctx, AES256_Key, InitializationVector );
  
  /* check for initialization errors */
  if (error_status == AES_SUCCESS)
  {
    /* Encrypt Data */
    error_status = AES_CBC_Encrypt_Append(&AESctx,
                                          InputMessage,
                                          InputMessageLength,
                                          OutputMessage,
                                          &outputLength);
    
    if (error_status == AES_SUCCESS)
    {
      /* Write the number of data written*/
      *OutputMessageLength = outputLength;
      /* Do the Finalization */
      error_status = AES_CBC_Encrypt_Finish(&AESctx, OutputMessage + *OutputMessageLength, &outputLength);
      /* Add data written to the information to be returned */
      *OutputMessageLength += outputLength;
    }
  }
  
  return error_status;
}


/**
* @brief  AES CBC Decryption example.
* @param  InputMessage: pointer to input message to be decrypted.
* @param  InputMessageLength: input data message length in byte.
* @param  AES256_Key: pointer to the AES key to be used in the operation
* @param  InitializationVector: pointer to the Initialization Vector (IV)
* @param  IvLength: IV length in bytes.
* @param  OutputMessage: pointer to output parameter that will handle the decrypted message
* @param  OutputMessageLength: pointer to decrypted message length.
* @retval error status: can be AES_SUCCESS if success or one of
*         AES_ERR_BAD_INPUT_SIZE, AES_ERR_BAD_OPERATION, AES_ERR_BAD_CONTEXT
*         AES_ERR_BAD_PARAMETER if error occured.
*/
int32_t ttSTM32_AES_CBC_Decrypt(uint8_t* InputMessage,
                                uint32_t InputMessageLength,
                                uint8_t  *AES256_Key,
                                uint8_t  *InitializationVector,
                                uint32_t  IvLength,
                                uint8_t  *OutputMessage,
                                uint32_t *OutputMessageLength)
{
  AESCBCctx_stt AESctx;
  
  uint32_t error_status = AES_SUCCESS;
  
  int32_t outputLength = 0;
  
  /* Set flag field to default value */
  AESctx.mFlags = E_SK_DEFAULT;
  
  /* Set key size to 32 (corresponding to AES-256) */
  AESctx.mKeySize = 32;
  
  /* Set iv size field to IvLength*/
  AESctx.mIvSize = IvLength;
  
  /* Initialize the operation, by passing the key.
  * Third parameter is NULL because CBC doesn't use any IV */
  error_status = AES_CBC_Decrypt_Init(&AESctx, AES256_Key, InitializationVector );
  
  /* check for initialization errors */
  if (error_status == AES_SUCCESS)
  {
    /* Decrypt Data */
    error_status = AES_CBC_Decrypt_Append(&AESctx,
                                          InputMessage,
                                          InputMessageLength,
                                          OutputMessage,
                                          &outputLength);
    
    if (error_status == AES_SUCCESS)
    {
      /* Write the number of data written*/
      *OutputMessageLength = outputLength;
      /* Do the Finalization */
      error_status = AES_CBC_Decrypt_Finish(&AESctx, OutputMessage + *OutputMessageLength, &outputLength);
      /* Add data written to the information to be returned */
      *OutputMessageLength += outputLength;
    }
  }
  
  return error_status;
}

/**
* @brief  Compares two buffers.
* @param  pBuffer, pBuffer1: buffers to be compared.
* @param  BufferLength: buffer's length
* @retval PASSED: pBuffer identical to pBuffer1
*         FAILED: pBuffer differs from pBuffer1
*/
TestStatus Buffercmp(const uint8_t* pBuffer, uint8_t* pBuffer1, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer != *pBuffer1)
    {
      return FAILED;
    }
    
    pBuffer++;
    pBuffer1++;
  }
  
  return PASSED;
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif
#endif 
