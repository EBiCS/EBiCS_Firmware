################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
Src/FOC.c \
Src/display_bafang.c \
Src/display_kingmeter.c \
Src/display_kunteng.c \
build/display_No_2.c \
Src/eeprom.c \
Src/main.c \
Src/print.c \
Src/stm32f1xx_hal_msp.c \
Src/stm32f1xx_it.c \
Src/system_stm32f1xx_Bootloader.c 

OBJS += \
build/FOC.o \
build/display_bafang.o \
build/display_kingmeter.o \
build/display_kunteng.o \
build/display_No_2.o \
build/eeprom.o \
build/main.o \
build/print.o \
build/stm32f1xx_hal_msp.o \
build/stm32f1xx_it.o \
build/system_stm32f1xx_Bootloader.o 

C_DEPS += \
build/FOC.d \
build/display_bafang.d \
build/display_kingmeter.d \
build/display_kunteng.d \
build/display_No_2.d \
build/eeprom.d \
build/main.d \
build/print.d \
build/stm32f1xx_hal_msp.d \
build/stm32f1xx_it.d \
build/system_stm32f1xx_Bootloader.d 

INC_PATH = Inc
DRIVERS_PATH = Drivers/STM32F1xx_HAL_Driver/Inc
LEGACY_PATH = Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
DEVICE_PATH = Drivers/CMSIS/Device/ST/STM32F1xx/Include
CMSIS_PATH = Drivers/CMSIS/Include


# Each subdirectory must supply rules for building sources it contributes
build/%.o: Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' -DARM_MATH_CM3 '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103x6 -I $(INC_PATH) -I $(DRIVERS_PATH) -I $(LEGACY_PATH) -I $(DEVICE_PATH) -I $(CMSIS_PATH) -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


