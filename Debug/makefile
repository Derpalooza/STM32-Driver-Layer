################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include drivers/Src/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Driver\ Layer.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Driver\ Layer.list \

OBJCOPY_BIN += \
Driver\ Layer.bin \


# All Target
all: Driver\ Layer.elf secondary-outputs

# Tool invocations
Driver\ Layer.elf: $(OBJS) $(USER_OBJS) D:\Documents\Courses\MCU1_Fastbit\MCU1\Driver\ Layer\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "Driver Layer.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Documents\Courses\MCU1_Fastbit\MCU1\Driver Layer\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Driver Layer.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Driver\ Layer.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Driver Layer.list"
	@echo 'Finished building: $@'
	@echo ' '

Driver\ Layer.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Driver Layer.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
