################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/EXFT/audit.c \
../Core/Src/EXFT/autopower.c \
../Core/Src/EXFT/autoresonance.c \
../Core/Src/EXFT/ble.c \
../Core/Src/EXFT/buzzer.c \
../Core/Src/EXFT/charger.c \
../Core/Src/EXFT/comm.c \
../Core/Src/EXFT/config.c \
../Core/Src/EXFT/crc32.c \
../Core/Src/EXFT/dac.c \
../Core/Src/EXFT/events.c \
../Core/Src/EXFT/fwupgrade.c \
../Core/Src/EXFT/hwdrivers.c \
../Core/Src/EXFT/leds.c \
../Core/Src/EXFT/measure.c \
../Core/Src/EXFT/pccommAppLayer.c \
../Core/Src/EXFT/protocolApp.c \
../Core/Src/EXFT/rtc.c \
../Core/Src/EXFT/spiflash.c \
../Core/Src/EXFT/spiflashdisk.c \
../Core/Src/EXFT/tracer.c \
../Core/Src/EXFT/transQ.c \
../Core/Src/EXFT/uartdll.c \
../Core/Src/EXFT/usartdll.c \
../Core/Src/EXFT/vibrator.c \
../Core/Src/EXFT/vlapMain.c 

OBJS += \
./Core/Src/EXFT/audit.o \
./Core/Src/EXFT/autopower.o \
./Core/Src/EXFT/autoresonance.o \
./Core/Src/EXFT/ble.o \
./Core/Src/EXFT/buzzer.o \
./Core/Src/EXFT/charger.o \
./Core/Src/EXFT/comm.o \
./Core/Src/EXFT/config.o \
./Core/Src/EXFT/crc32.o \
./Core/Src/EXFT/dac.o \
./Core/Src/EXFT/events.o \
./Core/Src/EXFT/fwupgrade.o \
./Core/Src/EXFT/hwdrivers.o \
./Core/Src/EXFT/leds.o \
./Core/Src/EXFT/measure.o \
./Core/Src/EXFT/pccommAppLayer.o \
./Core/Src/EXFT/protocolApp.o \
./Core/Src/EXFT/rtc.o \
./Core/Src/EXFT/spiflash.o \
./Core/Src/EXFT/spiflashdisk.o \
./Core/Src/EXFT/tracer.o \
./Core/Src/EXFT/transQ.o \
./Core/Src/EXFT/uartdll.o \
./Core/Src/EXFT/usartdll.o \
./Core/Src/EXFT/vibrator.o \
./Core/Src/EXFT/vlapMain.o 

C_DEPS += \
./Core/Src/EXFT/audit.d \
./Core/Src/EXFT/autopower.d \
./Core/Src/EXFT/autoresonance.d \
./Core/Src/EXFT/ble.d \
./Core/Src/EXFT/buzzer.d \
./Core/Src/EXFT/charger.d \
./Core/Src/EXFT/comm.d \
./Core/Src/EXFT/config.d \
./Core/Src/EXFT/crc32.d \
./Core/Src/EXFT/dac.d \
./Core/Src/EXFT/events.d \
./Core/Src/EXFT/fwupgrade.d \
./Core/Src/EXFT/hwdrivers.d \
./Core/Src/EXFT/leds.d \
./Core/Src/EXFT/measure.d \
./Core/Src/EXFT/pccommAppLayer.d \
./Core/Src/EXFT/protocolApp.d \
./Core/Src/EXFT/rtc.d \
./Core/Src/EXFT/spiflash.d \
./Core/Src/EXFT/spiflashdisk.d \
./Core/Src/EXFT/tracer.d \
./Core/Src/EXFT/transQ.d \
./Core/Src/EXFT/uartdll.d \
./Core/Src/EXFT/usartdll.d \
./Core/Src/EXFT/vibrator.d \
./Core/Src/EXFT/vlapMain.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/EXFT/%.o Core/Src/EXFT/%.su Core/Src/EXFT/%.cyclo: ../Core/Src/EXFT/%.c Core/Src/EXFT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H753xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-EXFT

clean-Core-2f-Src-2f-EXFT:
	-$(RM) ./Core/Src/EXFT/audit.cyclo ./Core/Src/EXFT/audit.d ./Core/Src/EXFT/audit.o ./Core/Src/EXFT/audit.su ./Core/Src/EXFT/autopower.cyclo ./Core/Src/EXFT/autopower.d ./Core/Src/EXFT/autopower.o ./Core/Src/EXFT/autopower.su ./Core/Src/EXFT/autoresonance.cyclo ./Core/Src/EXFT/autoresonance.d ./Core/Src/EXFT/autoresonance.o ./Core/Src/EXFT/autoresonance.su ./Core/Src/EXFT/ble.cyclo ./Core/Src/EXFT/ble.d ./Core/Src/EXFT/ble.o ./Core/Src/EXFT/ble.su ./Core/Src/EXFT/buzzer.cyclo ./Core/Src/EXFT/buzzer.d ./Core/Src/EXFT/buzzer.o ./Core/Src/EXFT/buzzer.su ./Core/Src/EXFT/charger.cyclo ./Core/Src/EXFT/charger.d ./Core/Src/EXFT/charger.o ./Core/Src/EXFT/charger.su ./Core/Src/EXFT/comm.cyclo ./Core/Src/EXFT/comm.d ./Core/Src/EXFT/comm.o ./Core/Src/EXFT/comm.su ./Core/Src/EXFT/config.cyclo ./Core/Src/EXFT/config.d ./Core/Src/EXFT/config.o ./Core/Src/EXFT/config.su ./Core/Src/EXFT/crc32.cyclo ./Core/Src/EXFT/crc32.d ./Core/Src/EXFT/crc32.o ./Core/Src/EXFT/crc32.su ./Core/Src/EXFT/dac.cyclo ./Core/Src/EXFT/dac.d ./Core/Src/EXFT/dac.o ./Core/Src/EXFT/dac.su ./Core/Src/EXFT/events.cyclo ./Core/Src/EXFT/events.d ./Core/Src/EXFT/events.o ./Core/Src/EXFT/events.su ./Core/Src/EXFT/fwupgrade.cyclo ./Core/Src/EXFT/fwupgrade.d ./Core/Src/EXFT/fwupgrade.o ./Core/Src/EXFT/fwupgrade.su ./Core/Src/EXFT/hwdrivers.cyclo ./Core/Src/EXFT/hwdrivers.d ./Core/Src/EXFT/hwdrivers.o ./Core/Src/EXFT/hwdrivers.su ./Core/Src/EXFT/leds.cyclo ./Core/Src/EXFT/leds.d ./Core/Src/EXFT/leds.o ./Core/Src/EXFT/leds.su ./Core/Src/EXFT/measure.cyclo ./Core/Src/EXFT/measure.d ./Core/Src/EXFT/measure.o ./Core/Src/EXFT/measure.su ./Core/Src/EXFT/pccommAppLayer.cyclo ./Core/Src/EXFT/pccommAppLayer.d ./Core/Src/EXFT/pccommAppLayer.o ./Core/Src/EXFT/pccommAppLayer.su ./Core/Src/EXFT/protocolApp.cyclo ./Core/Src/EXFT/protocolApp.d ./Core/Src/EXFT/protocolApp.o ./Core/Src/EXFT/protocolApp.su ./Core/Src/EXFT/rtc.cyclo ./Core/Src/EXFT/rtc.d ./Core/Src/EXFT/rtc.o ./Core/Src/EXFT/rtc.su ./Core/Src/EXFT/spiflash.cyclo ./Core/Src/EXFT/spiflash.d ./Core/Src/EXFT/spiflash.o ./Core/Src/EXFT/spiflash.su ./Core/Src/EXFT/spiflashdisk.cyclo ./Core/Src/EXFT/spiflashdisk.d ./Core/Src/EXFT/spiflashdisk.o ./Core/Src/EXFT/spiflashdisk.su ./Core/Src/EXFT/tracer.cyclo ./Core/Src/EXFT/tracer.d ./Core/Src/EXFT/tracer.o ./Core/Src/EXFT/tracer.su ./Core/Src/EXFT/transQ.cyclo ./Core/Src/EXFT/transQ.d ./Core/Src/EXFT/transQ.o ./Core/Src/EXFT/transQ.su ./Core/Src/EXFT/uartdll.cyclo ./Core/Src/EXFT/uartdll.d ./Core/Src/EXFT/uartdll.o ./Core/Src/EXFT/uartdll.su ./Core/Src/EXFT/usartdll.cyclo ./Core/Src/EXFT/usartdll.d ./Core/Src/EXFT/usartdll.o ./Core/Src/EXFT/usartdll.su ./Core/Src/EXFT/vibrator.cyclo ./Core/Src/EXFT/vibrator.d ./Core/Src/EXFT/vibrator.o ./Core/Src/EXFT/vibrator.su ./Core/Src/EXFT/vlapMain.cyclo ./Core/Src/EXFT/vlapMain.d ./Core/Src/EXFT/vlapMain.o ./Core/Src/EXFT/vlapMain.su

.PHONY: clean-Core-2f-Src-2f-EXFT

