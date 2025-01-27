################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/EXFT/Sensors/sensor.c 

OBJS += \
./Core/Src/EXFT/Sensors/sensor.o 

C_DEPS += \
./Core/Src/EXFT/Sensors/sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/EXFT/Sensors/%.o Core/Src/EXFT/Sensors/%.su Core/Src/EXFT/Sensors/%.cyclo: ../Core/Src/EXFT/Sensors/%.c Core/Src/EXFT/Sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H753xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-EXFT-2f-Sensors

clean-Core-2f-Src-2f-EXFT-2f-Sensors:
	-$(RM) ./Core/Src/EXFT/Sensors/sensor.cyclo ./Core/Src/EXFT/Sensors/sensor.d ./Core/Src/EXFT/Sensors/sensor.o ./Core/Src/EXFT/Sensors/sensor.su

.PHONY: clean-Core-2f-Src-2f-EXFT-2f-Sensors

