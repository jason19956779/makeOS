################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery.c 

OBJS += \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery.o 

C_DEPS += \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32F429I-Discovery/%.o: ../Drivers/BSP/STM32F429I-Discovery/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F429xx -I"D:/Desktop/WORK/TTT/Inc" -I"D:/Desktop/WORK/TTT/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/Desktop/WORK/TTT/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/Desktop/WORK/TTT/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Desktop/WORK/TTT/Drivers/CMSIS/Include" -I"D:/Desktop/WORK/TTT/Drivers/BSP/STM32F429I-Discovery"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


