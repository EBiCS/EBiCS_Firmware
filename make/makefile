################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

SRC_PATH = ../Drivers/CMSIS

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: LishuiFOC_01.elf

# Tool invocations
LishuiFOC_01.elf: $(OBJS) $(USER_OBJS) ../STM32F103C6Tx_FLASH.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -L $(SRC_PATH) -specs=nosys.specs -specs=nano.specs -T"../STM32F103C6Tx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "LishuiFOC_01.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating hex and Printing size information:'
	arm-none-eabi-objcopy -O ihex "LishuiFOC_01.elf" "LishuiFOC_01.hex"
	arm-none-eabi-size "LishuiFOC_01.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
