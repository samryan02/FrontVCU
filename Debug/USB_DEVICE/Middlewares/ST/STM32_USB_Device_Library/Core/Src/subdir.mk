################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
../USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
../USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c 

OBJS += \
./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.o \
./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.o \
./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.o 

C_DEPS += \
./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.d \
./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.d \
./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.o USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.su: ../USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.c USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USB_DEVICE-2f-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Core-2f-Src

clean-USB_DEVICE-2f-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Core-2f-Src:
	-$(RM) ./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.d ./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.o ./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.su ./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.d ./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.o ./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.su ./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.d ./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.o ./USB_DEVICE/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.su

.PHONY: clean-USB_DEVICE-2f-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Core-2f-Src

