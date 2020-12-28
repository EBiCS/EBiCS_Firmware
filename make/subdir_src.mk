################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FOC.c \
../Src/display_bafang.c \
../Src/display_ebics.c \
../Src/display_kingmeter.c \
../Src/display_kunteng.c \
../Src/eeprom.c \
../Src/main.c \
../Src/print.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c 

OBJS += \
./Src/FOC.o \
./Src/display_bafang.o \
./Src/display_ebics.o \
./Src/display_kingmeter.o \
./Src/display_kunteng.o \
./Src/eeprom.o \
./Src/main.o \
./Src/print.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o 

C_DEPS += \
./Src/FOC.d \
./Src/display_bafang.d \
./Src/display_ebics.d \
./Src/display_kingmeter.d \
./Src/display_kunteng.d \
./Src/eeprom.d \
./Src/main.d \
./Src/print.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d 

INC_PATH = ../INC
DRIVERS_PATH = ../Drivers/STM32F1xx_HAL_Driver/Inc
LEGACY_PATH = ../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
DEVICE_PATH = ../Drivers/CMSIS/Device/ST/STM32F1xx/Include
CMSIS_PATH = ../Drivers/CMSIS/Include


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' -DARM_MATH_CM3 '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103x6 -I $(INC_PATH) -I $(DRIVERS_PATH) -I $(LEGACY_PATH) -I $(DEVICE_PATH) -I $(CMSIS_PATH) -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


