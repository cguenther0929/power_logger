################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f105vbhx.s 

OBJS += \
./Core/Startup/startup_stm32f105vbhx.o 

S_DEPS += \
./Core/Startup/startup_stm32f105vbhx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32f105vbhx.o: ../Core/Startup/startup_stm32f105vbhx.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32f105vbhx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

