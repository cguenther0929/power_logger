################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FreeSans9pt7b.c \
../Core/Src/ad4681.c \
../Core/Src/fstfs_sd.c \
../Core/Src/main.c \
../Core/Src/oled_128b64.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/timer.c \
../Core/Src/uart.c 

OBJS += \
./Core/Src/FreeSans9pt7b.o \
./Core/Src/ad4681.o \
./Core/Src/fstfs_sd.o \
./Core/Src/main.o \
./Core/Src/oled_128b64.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/timer.o \
./Core/Src/uart.o 

C_DEPS += \
./Core/Src/FreeSans9pt7b.d \
./Core/Src/ad4681.d \
./Core/Src/fstfs_sd.d \
./Core/Src/main.d \
./Core/Src/oled_128b64.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/timer.d \
./Core/Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F105xC -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/FreeSans9pt7b.d ./Core/Src/FreeSans9pt7b.o ./Core/Src/ad4681.d ./Core/Src/ad4681.o ./Core/Src/fstfs_sd.d ./Core/Src/fstfs_sd.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/oled_128b64.d ./Core/Src/oled_128b64.o ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/timer.d ./Core/Src/timer.o ./Core/Src/uart.d ./Core/Src/uart.o

.PHONY: clean-Core-2f-Src

