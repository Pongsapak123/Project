################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DPS/Source/ControllerFunctions/arm_pid_init_f32.c \
../DPS/Source/ControllerFunctions/arm_pid_init_q15.c \
../DPS/Source/ControllerFunctions/arm_pid_init_q31.c \
../DPS/Source/ControllerFunctions/arm_pid_reset_f32.c \
../DPS/Source/ControllerFunctions/arm_pid_reset_q15.c \
../DPS/Source/ControllerFunctions/arm_pid_reset_q31.c \
../DPS/Source/ControllerFunctions/arm_sin_cos_f32.c \
../DPS/Source/ControllerFunctions/arm_sin_cos_q31.c 

OBJS += \
./DPS/Source/ControllerFunctions/arm_pid_init_f32.o \
./DPS/Source/ControllerFunctions/arm_pid_init_q15.o \
./DPS/Source/ControllerFunctions/arm_pid_init_q31.o \
./DPS/Source/ControllerFunctions/arm_pid_reset_f32.o \
./DPS/Source/ControllerFunctions/arm_pid_reset_q15.o \
./DPS/Source/ControllerFunctions/arm_pid_reset_q31.o \
./DPS/Source/ControllerFunctions/arm_sin_cos_f32.o \
./DPS/Source/ControllerFunctions/arm_sin_cos_q31.o 

C_DEPS += \
./DPS/Source/ControllerFunctions/arm_pid_init_f32.d \
./DPS/Source/ControllerFunctions/arm_pid_init_q15.d \
./DPS/Source/ControllerFunctions/arm_pid_init_q31.d \
./DPS/Source/ControllerFunctions/arm_pid_reset_f32.d \
./DPS/Source/ControllerFunctions/arm_pid_reset_q15.d \
./DPS/Source/ControllerFunctions/arm_pid_reset_q31.d \
./DPS/Source/ControllerFunctions/arm_sin_cos_f32.d \
./DPS/Source/ControllerFunctions/arm_sin_cos_q31.d 


# Each subdirectory must supply rules for building sources it contributes
DPS/Source/ControllerFunctions/%.o DPS/Source/ControllerFunctions/%.su DPS/Source/ControllerFunctions/%.cyclo: ../DPS/Source/ControllerFunctions/%.c DPS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Work/Micro/Project/Project/DPS/ComputeLibrary/Include" -I"C:/Work/Micro/Project/Project/DPS/Include" -I"C:/Work/Micro/Project/Project/DPS/PrivateInclude" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DPS-2f-Source-2f-ControllerFunctions

clean-DPS-2f-Source-2f-ControllerFunctions:
	-$(RM) ./DPS/Source/ControllerFunctions/arm_pid_init_f32.cyclo ./DPS/Source/ControllerFunctions/arm_pid_init_f32.d ./DPS/Source/ControllerFunctions/arm_pid_init_f32.o ./DPS/Source/ControllerFunctions/arm_pid_init_f32.su ./DPS/Source/ControllerFunctions/arm_pid_init_q15.cyclo ./DPS/Source/ControllerFunctions/arm_pid_init_q15.d ./DPS/Source/ControllerFunctions/arm_pid_init_q15.o ./DPS/Source/ControllerFunctions/arm_pid_init_q15.su ./DPS/Source/ControllerFunctions/arm_pid_init_q31.cyclo ./DPS/Source/ControllerFunctions/arm_pid_init_q31.d ./DPS/Source/ControllerFunctions/arm_pid_init_q31.o ./DPS/Source/ControllerFunctions/arm_pid_init_q31.su ./DPS/Source/ControllerFunctions/arm_pid_reset_f32.cyclo ./DPS/Source/ControllerFunctions/arm_pid_reset_f32.d ./DPS/Source/ControllerFunctions/arm_pid_reset_f32.o ./DPS/Source/ControllerFunctions/arm_pid_reset_f32.su ./DPS/Source/ControllerFunctions/arm_pid_reset_q15.cyclo ./DPS/Source/ControllerFunctions/arm_pid_reset_q15.d ./DPS/Source/ControllerFunctions/arm_pid_reset_q15.o ./DPS/Source/ControllerFunctions/arm_pid_reset_q15.su ./DPS/Source/ControllerFunctions/arm_pid_reset_q31.cyclo ./DPS/Source/ControllerFunctions/arm_pid_reset_q31.d ./DPS/Source/ControllerFunctions/arm_pid_reset_q31.o ./DPS/Source/ControllerFunctions/arm_pid_reset_q31.su ./DPS/Source/ControllerFunctions/arm_sin_cos_f32.cyclo ./DPS/Source/ControllerFunctions/arm_sin_cos_f32.d ./DPS/Source/ControllerFunctions/arm_sin_cos_f32.o ./DPS/Source/ControllerFunctions/arm_sin_cos_f32.su ./DPS/Source/ControllerFunctions/arm_sin_cos_q31.cyclo ./DPS/Source/ControllerFunctions/arm_sin_cos_q31.d ./DPS/Source/ControllerFunctions/arm_sin_cos_q31.o ./DPS/Source/ControllerFunctions/arm_sin_cos_q31.su

.PHONY: clean-DPS-2f-Source-2f-ControllerFunctions

