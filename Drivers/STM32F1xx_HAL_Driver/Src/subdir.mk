################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_iwdg.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c 

OBJS += \
build/stm32f1xx_hal.o \
build/stm32f1xx_hal_adc.o \
build/stm32f1xx_hal_adc_ex.o \
build/stm32f1xx_hal_cortex.o \
build/stm32f1xx_hal_dma.o \
build/stm32f1xx_hal_flash.o \
build/stm32f1xx_hal_flash_ex.o \
build/stm32f1xx_hal_gpio.o \
build/stm32f1xx_hal_gpio_ex.o \
build/stm32f1xx_hal_pwr.o \
build/stm32f1xx_hal_rcc.o \
build/stm32f1xx_hal_rcc_ex.o \
build/stm32f1xx_hal_tim.o \
build/stm32f1xx_hal_tim_ex.o \
build/stm32f1xx_hal_iwdg.o \
build/stm32f1xx_hal_uart.o 

C_DEPS += \
build/stm32f1xx_hal.d \
build/stm32f1xx_hal_adc.d \
build/stm32f1xx_hal_adc_ex.d \
build/stm32f1xx_hal_cortex.d \
build/stm32f1xx_hal_dma.d \
build/stm32f1xx_hal_flash.d \
build/stm32f1xx_hal_flash_ex.d \
build/stm32f1xx_hal_gpio.d \
build/stm32f1xx_hal_gpio_ex.d \
build/stm32f1xx_hal_pwr.d \
build/stm32f1xx_hal_rcc.d \
build/stm32f1xx_hal_rcc_ex.d \
build/stm32f1xx_hal_tim.d \
build/stm32f1xx_hal_tim_ex.d \
build/stm32f1xx_hal_iwdg.d \
build/stm32f1xx_hal_uart.d 

INC_PATH = Inc
DRIVERS_PATH = Drivers/STM32F1xx_HAL_Driver/Inc
LEGACY_PATH = Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
DEVICE_PATH = Drivers/CMSIS/Device/ST/STM32F1xx/Include
CMSIS_PATH = Drivers/CMSIS/Include


# Each subdirectory must supply rules for building sources it contributes
build/%.o: Drivers/STM32F1xx_HAL_Driver/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' -DARM_MATH_CM3 '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103x6 -I $(INC_PATH) -I $(DRIVERS_PATH) -I $(LEGACY_PATH) -I $(DEVICE_PATH) -I $(CMSIS_PATH) -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


