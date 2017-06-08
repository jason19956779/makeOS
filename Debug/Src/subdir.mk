################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/malloc.c \
../Src/stm32f429i_discovery.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c \
../Src/threads.c 

OBJS += \
./Src/main.o \
./Src/malloc.o \
./Src/stm32f429i_discovery.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o \
./Src/threads.o 

C_DEPS += \
./Src/main.d \
./Src/malloc.d \
./Src/stm32f429i_discovery.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d \
./Src/threads.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F429xx -I"C:/Users/Jason/Desktop/makeOS/Inc" -I"C:/Users/Jason/Desktop/makeOS/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Jason/Desktop/makeOS/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Jason/Desktop/makeOS/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Jason/Desktop/makeOS/Drivers/CMSIS/Include"  -O0 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


