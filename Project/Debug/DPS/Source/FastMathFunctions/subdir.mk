################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DPS/Source/FastMathFunctions/FastMathFunctionsF16.c \
../DPS/Source/FastMathFunctions/arm_atan2_f16.c \
../DPS/Source/FastMathFunctions/arm_atan2_f32.c \
../DPS/Source/FastMathFunctions/arm_atan2_q15.c \
../DPS/Source/FastMathFunctions/arm_atan2_q31.c \
../DPS/Source/FastMathFunctions/arm_cos_f32.c \
../DPS/Source/FastMathFunctions/arm_cos_q15.c \
../DPS/Source/FastMathFunctions/arm_cos_q31.c \
../DPS/Source/FastMathFunctions/arm_divide_q15.c \
../DPS/Source/FastMathFunctions/arm_divide_q31.c \
../DPS/Source/FastMathFunctions/arm_sin_f32.c \
../DPS/Source/FastMathFunctions/arm_sin_q15.c \
../DPS/Source/FastMathFunctions/arm_sin_q31.c \
../DPS/Source/FastMathFunctions/arm_sqrt_q15.c \
../DPS/Source/FastMathFunctions/arm_sqrt_q31.c \
../DPS/Source/FastMathFunctions/arm_vexp_f16.c \
../DPS/Source/FastMathFunctions/arm_vexp_f32.c \
../DPS/Source/FastMathFunctions/arm_vexp_f64.c \
../DPS/Source/FastMathFunctions/arm_vinverse_f16.c \
../DPS/Source/FastMathFunctions/arm_vlog_f16.c \
../DPS/Source/FastMathFunctions/arm_vlog_f32.c \
../DPS/Source/FastMathFunctions/arm_vlog_f64.c \
../DPS/Source/FastMathFunctions/arm_vlog_q15.c \
../DPS/Source/FastMathFunctions/arm_vlog_q31.c 

OBJS += \
./DPS/Source/FastMathFunctions/FastMathFunctionsF16.o \
./DPS/Source/FastMathFunctions/arm_atan2_f16.o \
./DPS/Source/FastMathFunctions/arm_atan2_f32.o \
./DPS/Source/FastMathFunctions/arm_atan2_q15.o \
./DPS/Source/FastMathFunctions/arm_atan2_q31.o \
./DPS/Source/FastMathFunctions/arm_cos_f32.o \
./DPS/Source/FastMathFunctions/arm_cos_q15.o \
./DPS/Source/FastMathFunctions/arm_cos_q31.o \
./DPS/Source/FastMathFunctions/arm_divide_q15.o \
./DPS/Source/FastMathFunctions/arm_divide_q31.o \
./DPS/Source/FastMathFunctions/arm_sin_f32.o \
./DPS/Source/FastMathFunctions/arm_sin_q15.o \
./DPS/Source/FastMathFunctions/arm_sin_q31.o \
./DPS/Source/FastMathFunctions/arm_sqrt_q15.o \
./DPS/Source/FastMathFunctions/arm_sqrt_q31.o \
./DPS/Source/FastMathFunctions/arm_vexp_f16.o \
./DPS/Source/FastMathFunctions/arm_vexp_f32.o \
./DPS/Source/FastMathFunctions/arm_vexp_f64.o \
./DPS/Source/FastMathFunctions/arm_vinverse_f16.o \
./DPS/Source/FastMathFunctions/arm_vlog_f16.o \
./DPS/Source/FastMathFunctions/arm_vlog_f32.o \
./DPS/Source/FastMathFunctions/arm_vlog_f64.o \
./DPS/Source/FastMathFunctions/arm_vlog_q15.o \
./DPS/Source/FastMathFunctions/arm_vlog_q31.o 

C_DEPS += \
./DPS/Source/FastMathFunctions/FastMathFunctionsF16.d \
./DPS/Source/FastMathFunctions/arm_atan2_f16.d \
./DPS/Source/FastMathFunctions/arm_atan2_f32.d \
./DPS/Source/FastMathFunctions/arm_atan2_q15.d \
./DPS/Source/FastMathFunctions/arm_atan2_q31.d \
./DPS/Source/FastMathFunctions/arm_cos_f32.d \
./DPS/Source/FastMathFunctions/arm_cos_q15.d \
./DPS/Source/FastMathFunctions/arm_cos_q31.d \
./DPS/Source/FastMathFunctions/arm_divide_q15.d \
./DPS/Source/FastMathFunctions/arm_divide_q31.d \
./DPS/Source/FastMathFunctions/arm_sin_f32.d \
./DPS/Source/FastMathFunctions/arm_sin_q15.d \
./DPS/Source/FastMathFunctions/arm_sin_q31.d \
./DPS/Source/FastMathFunctions/arm_sqrt_q15.d \
./DPS/Source/FastMathFunctions/arm_sqrt_q31.d \
./DPS/Source/FastMathFunctions/arm_vexp_f16.d \
./DPS/Source/FastMathFunctions/arm_vexp_f32.d \
./DPS/Source/FastMathFunctions/arm_vexp_f64.d \
./DPS/Source/FastMathFunctions/arm_vinverse_f16.d \
./DPS/Source/FastMathFunctions/arm_vlog_f16.d \
./DPS/Source/FastMathFunctions/arm_vlog_f32.d \
./DPS/Source/FastMathFunctions/arm_vlog_f64.d \
./DPS/Source/FastMathFunctions/arm_vlog_q15.d \
./DPS/Source/FastMathFunctions/arm_vlog_q31.d 


# Each subdirectory must supply rules for building sources it contributes
DPS/Source/FastMathFunctions/%.o DPS/Source/FastMathFunctions/%.su DPS/Source/FastMathFunctions/%.cyclo: ../DPS/Source/FastMathFunctions/%.c DPS/Source/FastMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Work/Micro/Project/Project/DPS/ComputeLibrary/Include" -I"C:/Work/Micro/Project/Project/DPS/Include" -I"C:/Work/Micro/Project/Project/DPS/PrivateInclude" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DPS-2f-Source-2f-FastMathFunctions

clean-DPS-2f-Source-2f-FastMathFunctions:
	-$(RM) ./DPS/Source/FastMathFunctions/FastMathFunctionsF16.cyclo ./DPS/Source/FastMathFunctions/FastMathFunctionsF16.d ./DPS/Source/FastMathFunctions/FastMathFunctionsF16.o ./DPS/Source/FastMathFunctions/FastMathFunctionsF16.su ./DPS/Source/FastMathFunctions/arm_atan2_f16.cyclo ./DPS/Source/FastMathFunctions/arm_atan2_f16.d ./DPS/Source/FastMathFunctions/arm_atan2_f16.o ./DPS/Source/FastMathFunctions/arm_atan2_f16.su ./DPS/Source/FastMathFunctions/arm_atan2_f32.cyclo ./DPS/Source/FastMathFunctions/arm_atan2_f32.d ./DPS/Source/FastMathFunctions/arm_atan2_f32.o ./DPS/Source/FastMathFunctions/arm_atan2_f32.su ./DPS/Source/FastMathFunctions/arm_atan2_q15.cyclo ./DPS/Source/FastMathFunctions/arm_atan2_q15.d ./DPS/Source/FastMathFunctions/arm_atan2_q15.o ./DPS/Source/FastMathFunctions/arm_atan2_q15.su ./DPS/Source/FastMathFunctions/arm_atan2_q31.cyclo ./DPS/Source/FastMathFunctions/arm_atan2_q31.d ./DPS/Source/FastMathFunctions/arm_atan2_q31.o ./DPS/Source/FastMathFunctions/arm_atan2_q31.su ./DPS/Source/FastMathFunctions/arm_cos_f32.cyclo ./DPS/Source/FastMathFunctions/arm_cos_f32.d ./DPS/Source/FastMathFunctions/arm_cos_f32.o ./DPS/Source/FastMathFunctions/arm_cos_f32.su ./DPS/Source/FastMathFunctions/arm_cos_q15.cyclo ./DPS/Source/FastMathFunctions/arm_cos_q15.d ./DPS/Source/FastMathFunctions/arm_cos_q15.o ./DPS/Source/FastMathFunctions/arm_cos_q15.su ./DPS/Source/FastMathFunctions/arm_cos_q31.cyclo ./DPS/Source/FastMathFunctions/arm_cos_q31.d ./DPS/Source/FastMathFunctions/arm_cos_q31.o ./DPS/Source/FastMathFunctions/arm_cos_q31.su ./DPS/Source/FastMathFunctions/arm_divide_q15.cyclo ./DPS/Source/FastMathFunctions/arm_divide_q15.d ./DPS/Source/FastMathFunctions/arm_divide_q15.o ./DPS/Source/FastMathFunctions/arm_divide_q15.su ./DPS/Source/FastMathFunctions/arm_divide_q31.cyclo ./DPS/Source/FastMathFunctions/arm_divide_q31.d ./DPS/Source/FastMathFunctions/arm_divide_q31.o ./DPS/Source/FastMathFunctions/arm_divide_q31.su ./DPS/Source/FastMathFunctions/arm_sin_f32.cyclo ./DPS/Source/FastMathFunctions/arm_sin_f32.d ./DPS/Source/FastMathFunctions/arm_sin_f32.o ./DPS/Source/FastMathFunctions/arm_sin_f32.su ./DPS/Source/FastMathFunctions/arm_sin_q15.cyclo ./DPS/Source/FastMathFunctions/arm_sin_q15.d ./DPS/Source/FastMathFunctions/arm_sin_q15.o ./DPS/Source/FastMathFunctions/arm_sin_q15.su ./DPS/Source/FastMathFunctions/arm_sin_q31.cyclo ./DPS/Source/FastMathFunctions/arm_sin_q31.d ./DPS/Source/FastMathFunctions/arm_sin_q31.o ./DPS/Source/FastMathFunctions/arm_sin_q31.su ./DPS/Source/FastMathFunctions/arm_sqrt_q15.cyclo ./DPS/Source/FastMathFunctions/arm_sqrt_q15.d ./DPS/Source/FastMathFunctions/arm_sqrt_q15.o ./DPS/Source/FastMathFunctions/arm_sqrt_q15.su ./DPS/Source/FastMathFunctions/arm_sqrt_q31.cyclo ./DPS/Source/FastMathFunctions/arm_sqrt_q31.d ./DPS/Source/FastMathFunctions/arm_sqrt_q31.o ./DPS/Source/FastMathFunctions/arm_sqrt_q31.su ./DPS/Source/FastMathFunctions/arm_vexp_f16.cyclo ./DPS/Source/FastMathFunctions/arm_vexp_f16.d ./DPS/Source/FastMathFunctions/arm_vexp_f16.o ./DPS/Source/FastMathFunctions/arm_vexp_f16.su ./DPS/Source/FastMathFunctions/arm_vexp_f32.cyclo ./DPS/Source/FastMathFunctions/arm_vexp_f32.d ./DPS/Source/FastMathFunctions/arm_vexp_f32.o ./DPS/Source/FastMathFunctions/arm_vexp_f32.su ./DPS/Source/FastMathFunctions/arm_vexp_f64.cyclo ./DPS/Source/FastMathFunctions/arm_vexp_f64.d ./DPS/Source/FastMathFunctions/arm_vexp_f64.o ./DPS/Source/FastMathFunctions/arm_vexp_f64.su ./DPS/Source/FastMathFunctions/arm_vinverse_f16.cyclo ./DPS/Source/FastMathFunctions/arm_vinverse_f16.d ./DPS/Source/FastMathFunctions/arm_vinverse_f16.o ./DPS/Source/FastMathFunctions/arm_vinverse_f16.su ./DPS/Source/FastMathFunctions/arm_vlog_f16.cyclo ./DPS/Source/FastMathFunctions/arm_vlog_f16.d ./DPS/Source/FastMathFunctions/arm_vlog_f16.o ./DPS/Source/FastMathFunctions/arm_vlog_f16.su ./DPS/Source/FastMathFunctions/arm_vlog_f32.cyclo ./DPS/Source/FastMathFunctions/arm_vlog_f32.d ./DPS/Source/FastMathFunctions/arm_vlog_f32.o ./DPS/Source/FastMathFunctions/arm_vlog_f32.su ./DPS/Source/FastMathFunctions/arm_vlog_f64.cyclo ./DPS/Source/FastMathFunctions/arm_vlog_f64.d ./DPS/Source/FastMathFunctions/arm_vlog_f64.o ./DPS/Source/FastMathFunctions/arm_vlog_f64.su ./DPS/Source/FastMathFunctions/arm_vlog_q15.cyclo ./DPS/Source/FastMathFunctions/arm_vlog_q15.d ./DPS/Source/FastMathFunctions/arm_vlog_q15.o ./DPS/Source/FastMathFunctions/arm_vlog_q15.su ./DPS/Source/FastMathFunctions/arm_vlog_q31.cyclo ./DPS/Source/FastMathFunctions/arm_vlog_q31.d ./DPS/Source/FastMathFunctions/arm_vlog_q31.o ./DPS/Source/FastMathFunctions/arm_vlog_q31.su

.PHONY: clean-DPS-2f-Source-2f-FastMathFunctions

