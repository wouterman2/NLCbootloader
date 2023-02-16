################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Github/nunc-coffee-embedded/Core\ shared/src/crc.c \
D:/Github/nunc-coffee-embedded/Core\ shared/src/eeprom.c \
D:/Github/nunc-coffee-embedded/Core\ shared/src/flash.c \
D:/Github/nunc-coffee-embedded/Core\ shared/src/led.c \
D:/Github/nunc-coffee-embedded/Core\ shared/src/serial.c \
D:/Github/nunc-coffee-embedded/Core\ shared/src/utils.c \
D:/Github/nunc-coffee-embedded/Core\ shared/src/ws2812.c 

OBJS += \
./Core\ shared/src/crc.o \
./Core\ shared/src/eeprom.o \
./Core\ shared/src/flash.o \
./Core\ shared/src/led.o \
./Core\ shared/src/serial.o \
./Core\ shared/src/utils.o \
./Core\ shared/src/ws2812.o 

C_DEPS += \
./Core\ shared/src/crc.d \
./Core\ shared/src/eeprom.d \
./Core\ shared/src/flash.d \
./Core\ shared/src/led.d \
./Core\ shared/src/serial.d \
./Core\ shared/src/utils.d \
./Core\ shared/src/ws2812.d 


# Each subdirectory must supply rules for building sources it contributes
Core\ shared/src/crc.o: D:/Github/nunc-coffee-embedded/Core\ shared/src/crc.c Core\ shared/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L151xC -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"../../nunc-coffee-embedded/Core shared/inc" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core shared/src/crc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core\ shared/src/eeprom.o: D:/Github/nunc-coffee-embedded/Core\ shared/src/eeprom.c Core\ shared/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L151xC -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"../../nunc-coffee-embedded/Core shared/inc" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core shared/src/eeprom.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core\ shared/src/flash.o: D:/Github/nunc-coffee-embedded/Core\ shared/src/flash.c Core\ shared/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L151xC -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"../../nunc-coffee-embedded/Core shared/inc" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core shared/src/flash.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core\ shared/src/led.o: D:/Github/nunc-coffee-embedded/Core\ shared/src/led.c Core\ shared/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L151xC -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"../../nunc-coffee-embedded/Core shared/inc" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core shared/src/led.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core\ shared/src/serial.o: D:/Github/nunc-coffee-embedded/Core\ shared/src/serial.c Core\ shared/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L151xC -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"../../nunc-coffee-embedded/Core shared/inc" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core shared/src/serial.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core\ shared/src/utils.o: D:/Github/nunc-coffee-embedded/Core\ shared/src/utils.c Core\ shared/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L151xC -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"../../nunc-coffee-embedded/Core shared/inc" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core shared/src/utils.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core\ shared/src/ws2812.o: D:/Github/nunc-coffee-embedded/Core\ shared/src/ws2812.c Core\ shared/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L151xC -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"../../nunc-coffee-embedded/Core shared/inc" -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core shared/src/ws2812.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-20-shared-2f-src

clean-Core-20-shared-2f-src:
	-$(RM) ./Core\ shared/src/crc.d ./Core\ shared/src/crc.o ./Core\ shared/src/crc.su ./Core\ shared/src/eeprom.d ./Core\ shared/src/eeprom.o ./Core\ shared/src/eeprom.su ./Core\ shared/src/flash.d ./Core\ shared/src/flash.o ./Core\ shared/src/flash.su ./Core\ shared/src/led.d ./Core\ shared/src/led.o ./Core\ shared/src/led.su ./Core\ shared/src/serial.d ./Core\ shared/src/serial.o ./Core\ shared/src/serial.su ./Core\ shared/src/utils.d ./Core\ shared/src/utils.o ./Core\ shared/src/utils.su ./Core\ shared/src/ws2812.d ./Core\ shared/src/ws2812.o ./Core\ shared/src/ws2812.su

.PHONY: clean-Core-20-shared-2f-src

