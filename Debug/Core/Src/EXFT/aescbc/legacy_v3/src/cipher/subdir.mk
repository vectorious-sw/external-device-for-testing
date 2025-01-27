################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cbc.c \
../Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ccm.c \
../Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cfb.c \
../Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ctr.c \
../Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ecb.c \
../Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_gcm.c \
../Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_keywrap.c \
../Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ofb.c \
../Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_xts.c \
../Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_chachapoly.c 

OBJS += \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cbc.o \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ccm.o \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cfb.o \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ctr.o \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ecb.o \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_gcm.o \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_keywrap.o \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ofb.o \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_xts.o \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_chachapoly.o 

C_DEPS += \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cbc.d \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ccm.d \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cfb.d \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ctr.d \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ecb.d \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_gcm.d \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_keywrap.d \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ofb.d \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_xts.d \
./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_chachapoly.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/EXFT/aescbc/legacy_v3/src/cipher/%.o Core/Src/EXFT/aescbc/legacy_v3/src/cipher/%.su Core/Src/EXFT/aescbc/legacy_v3/src/cipher/%.cyclo: ../Core/Src/EXFT/aescbc/legacy_v3/src/cipher/%.c Core/Src/EXFT/aescbc/legacy_v3/src/cipher/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H753xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-EXFT-2f-aescbc-2f-legacy_v3-2f-src-2f-cipher

clean-Core-2f-Src-2f-EXFT-2f-aescbc-2f-legacy_v3-2f-src-2f-cipher:
	-$(RM) ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cbc.cyclo ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cbc.d ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cbc.o ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cbc.su ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ccm.cyclo ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ccm.d ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ccm.o ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ccm.su ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cfb.cyclo ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cfb.d ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cfb.o ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_cfb.su ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ctr.cyclo ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ctr.d ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ctr.o ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ctr.su ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ecb.cyclo ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ecb.d ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ecb.o ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ecb.su ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_gcm.cyclo ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_gcm.d ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_gcm.o ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_gcm.su ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_keywrap.cyclo ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_keywrap.d ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_keywrap.o ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_keywrap.su ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ofb.cyclo ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ofb.d ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ofb.o ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_ofb.su ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_xts.cyclo ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_xts.d ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_xts.o ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_aes_xts.su ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_chachapoly.cyclo ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_chachapoly.d ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_chachapoly.o ./Core/Src/EXFT/aescbc/legacy_v3/src/cipher/legacy_v3_chachapoly.su

.PHONY: clean-Core-2f-Src-2f-EXFT-2f-aescbc-2f-legacy_v3-2f-src-2f-cipher

