################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP_Lib/Source/arm_correlate_fast_q15.c \
../Drivers/CMSIS/DSP_Lib/Source/arm_dot_prod_q15.c \
../Drivers/CMSIS/DSP_Lib/Source/arm_fill_q15.c \
../Drivers/CMSIS/DSP_Lib/Source/arm_max_q15.c \
../Drivers/CMSIS/DSP_Lib/Source/arm_max_q31.c \
../Drivers/CMSIS/DSP_Lib/Source/arm_min_q15.c \
../Drivers/CMSIS/DSP_Lib/Source/arm_min_q31.c 

OBJS += \
./Drivers/CMSIS/DSP_Lib/Source/arm_correlate_fast_q15.o \
./Drivers/CMSIS/DSP_Lib/Source/arm_dot_prod_q15.o \
./Drivers/CMSIS/DSP_Lib/Source/arm_fill_q15.o \
./Drivers/CMSIS/DSP_Lib/Source/arm_max_q15.o \
./Drivers/CMSIS/DSP_Lib/Source/arm_max_q31.o \
./Drivers/CMSIS/DSP_Lib/Source/arm_min_q15.o \
./Drivers/CMSIS/DSP_Lib/Source/arm_min_q31.o 

C_DEPS += \
./Drivers/CMSIS/DSP_Lib/Source/arm_correlate_fast_q15.d \
./Drivers/CMSIS/DSP_Lib/Source/arm_dot_prod_q15.d \
./Drivers/CMSIS/DSP_Lib/Source/arm_fill_q15.d \
./Drivers/CMSIS/DSP_Lib/Source/arm_max_q15.d \
./Drivers/CMSIS/DSP_Lib/Source/arm_max_q31.d \
./Drivers/CMSIS/DSP_Lib/Source/arm_min_q15.d \
./Drivers/CMSIS/DSP_Lib/Source/arm_min_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP_Lib/Source/%.o Drivers/CMSIS/DSP_Lib/Source/%.su Drivers/CMSIS/DSP_Lib/Source/%.cyclo: ../Drivers/CMSIS/DSP_Lib/Source/%.c Drivers/CMSIS/DSP_Lib/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H753xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Source

clean-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Source:
	-$(RM) ./Drivers/CMSIS/DSP_Lib/Source/arm_correlate_fast_q15.cyclo ./Drivers/CMSIS/DSP_Lib/Source/arm_correlate_fast_q15.d ./Drivers/CMSIS/DSP_Lib/Source/arm_correlate_fast_q15.o ./Drivers/CMSIS/DSP_Lib/Source/arm_correlate_fast_q15.su ./Drivers/CMSIS/DSP_Lib/Source/arm_dot_prod_q15.cyclo ./Drivers/CMSIS/DSP_Lib/Source/arm_dot_prod_q15.d ./Drivers/CMSIS/DSP_Lib/Source/arm_dot_prod_q15.o ./Drivers/CMSIS/DSP_Lib/Source/arm_dot_prod_q15.su ./Drivers/CMSIS/DSP_Lib/Source/arm_fill_q15.cyclo ./Drivers/CMSIS/DSP_Lib/Source/arm_fill_q15.d ./Drivers/CMSIS/DSP_Lib/Source/arm_fill_q15.o ./Drivers/CMSIS/DSP_Lib/Source/arm_fill_q15.su ./Drivers/CMSIS/DSP_Lib/Source/arm_max_q15.cyclo ./Drivers/CMSIS/DSP_Lib/Source/arm_max_q15.d ./Drivers/CMSIS/DSP_Lib/Source/arm_max_q15.o ./Drivers/CMSIS/DSP_Lib/Source/arm_max_q15.su ./Drivers/CMSIS/DSP_Lib/Source/arm_max_q31.cyclo ./Drivers/CMSIS/DSP_Lib/Source/arm_max_q31.d ./Drivers/CMSIS/DSP_Lib/Source/arm_max_q31.o ./Drivers/CMSIS/DSP_Lib/Source/arm_max_q31.su ./Drivers/CMSIS/DSP_Lib/Source/arm_min_q15.cyclo ./Drivers/CMSIS/DSP_Lib/Source/arm_min_q15.d ./Drivers/CMSIS/DSP_Lib/Source/arm_min_q15.o ./Drivers/CMSIS/DSP_Lib/Source/arm_min_q15.su ./Drivers/CMSIS/DSP_Lib/Source/arm_min_q31.cyclo ./Drivers/CMSIS/DSP_Lib/Source/arm_min_q31.d ./Drivers/CMSIS/DSP_Lib/Source/arm_min_q31.o ./Drivers/CMSIS/DSP_Lib/Source/arm_min_q31.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Source

