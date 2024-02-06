################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
../USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
../USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/list.c \
../USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
../USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
../USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
../USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/timers.c 

OBJS += \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/croutine.o \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/event_groups.o \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/list.o \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/queue.o \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.o \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/tasks.o \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/timers.o 

C_DEPS += \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/croutine.d \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/event_groups.d \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/list.d \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/queue.d \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.d \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/tasks.d \
./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/%.o USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/%.su: ../USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/%.c USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USB_DEVICE-2f-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source

clean-USB_DEVICE-2f-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source:
	-$(RM) ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/croutine.d ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/croutine.o ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/croutine.su ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/event_groups.d ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/event_groups.o ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/event_groups.su ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/list.d ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/list.o ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/list.su ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/queue.d ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/queue.o ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/queue.su ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.d ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.o ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.su ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/tasks.d ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/tasks.o ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/tasks.su ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/timers.d ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/timers.o ./USB_DEVICE/Middlewares/Third_Party/FreeRTOS/Source/timers.su

.PHONY: clean-USB_DEVICE-2f-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source

