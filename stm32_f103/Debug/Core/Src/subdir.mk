################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ESC.c \
../Core/Src/ESP.c \
../Core/Src/MadgwickAHRS.c \
../Core/Src/aux_functions.c \
../Core/Src/imu.c \
../Core/Src/main.c \
../Core/Src/right_motor_encoder.c \
../Core/Src/run_course_hard_code.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/ultrasonic.c 

OBJS += \
./Core/Src/ESC.o \
./Core/Src/ESP.o \
./Core/Src/MadgwickAHRS.o \
./Core/Src/aux_functions.o \
./Core/Src/imu.o \
./Core/Src/main.o \
./Core/Src/right_motor_encoder.o \
./Core/Src/run_course_hard_code.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/ultrasonic.o 

C_DEPS += \
./Core/Src/ESC.d \
./Core/Src/ESP.d \
./Core/Src/MadgwickAHRS.d \
./Core/Src/aux_functions.d \
./Core/Src/imu.d \
./Core/Src/main.d \
./Core/Src/right_motor_encoder.d \
./Core/Src/run_course_hard_code.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/ultrasonic.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../FATFS/App -I../Core/Inc -I../FATFS/Target -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ESC.d ./Core/Src/ESC.o ./Core/Src/ESC.su ./Core/Src/ESP.d ./Core/Src/ESP.o ./Core/Src/ESP.su ./Core/Src/MadgwickAHRS.d ./Core/Src/MadgwickAHRS.o ./Core/Src/MadgwickAHRS.su ./Core/Src/aux_functions.d ./Core/Src/aux_functions.o ./Core/Src/aux_functions.su ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/right_motor_encoder.d ./Core/Src/right_motor_encoder.o ./Core/Src/right_motor_encoder.su ./Core/Src/run_course_hard_code.d ./Core/Src/run_course_hard_code.o ./Core/Src/run_course_hard_code.su ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/ultrasonic.d ./Core/Src/ultrasonic.o ./Core/Src/ultrasonic.su

.PHONY: clean-Core-2f-Src

