################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Plotter/src/TESTW02.c \
../Plotter/src/serial.c 

OBJS += \
./Plotter/src/TESTW02.o \
./Plotter/src/serial.o 

C_DEPS += \
./Plotter/src/TESTW02.d \
./Plotter/src/serial.d 


# Each subdirectory must supply rules for building sources it contributes
Plotter/src/%.o Plotter/src/%.su: ../Plotter/src/%.c Plotter/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Plotter/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Plotter-2f-src

clean-Plotter-2f-src:
	-$(RM) ./Plotter/src/TESTW02.d ./Plotter/src/TESTW02.o ./Plotter/src/TESTW02.su ./Plotter/src/serial.d ./Plotter/src/serial.o ./Plotter/src/serial.su

.PHONY: clean-Plotter-2f-src

