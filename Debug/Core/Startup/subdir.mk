################################################################################
# Automatically-generated file. Do not edit!
<<<<<<< HEAD
# Toolchain: GNU Tools for STM32 (11.3.rel1)
=======
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
>>>>>>> 180f99fe2e7e7d46b51ba5f79c7da3100d5a8c27
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l496rgtx.s 

OBJS += \
./Core/Startup/startup_stm32l496rgtx.o 

S_DEPS += \
./Core/Startup/startup_stm32l496rgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32l496rgtx.d ./Core/Startup/startup_stm32l496rgtx.o

.PHONY: clean-Core-2f-Startup

