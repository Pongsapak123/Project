################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DPS/Source/InterpolationFunctions/InterpolationFunctionsF16.c \
../DPS/Source/InterpolationFunctions/arm_bilinear_interp_f16.c \
../DPS/Source/InterpolationFunctions/arm_bilinear_interp_f32.c \
../DPS/Source/InterpolationFunctions/arm_bilinear_interp_q15.c \
../DPS/Source/InterpolationFunctions/arm_bilinear_interp_q31.c \
../DPS/Source/InterpolationFunctions/arm_bilinear_interp_q7.c \
../DPS/Source/InterpolationFunctions/arm_linear_interp_f16.c \
../DPS/Source/InterpolationFunctions/arm_linear_interp_f32.c \
../DPS/Source/InterpolationFunctions/arm_linear_interp_q15.c \
../DPS/Source/InterpolationFunctions/arm_linear_interp_q31.c \
../DPS/Source/InterpolationFunctions/arm_linear_interp_q7.c \
../DPS/Source/InterpolationFunctions/arm_spline_interp_f32.c \
../DPS/Source/InterpolationFunctions/arm_spline_interp_init_f32.c 

OBJS += \
./DPS/Source/InterpolationFunctions/InterpolationFunctionsF16.o \
./DPS/Source/InterpolationFunctions/arm_bilinear_interp_f16.o \
./DPS/Source/InterpolationFunctions/arm_bilinear_interp_f32.o \
./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q15.o \
./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q31.o \
./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q7.o \
./DPS/Source/InterpolationFunctions/arm_linear_interp_f16.o \
./DPS/Source/InterpolationFunctions/arm_linear_interp_f32.o \
./DPS/Source/InterpolationFunctions/arm_linear_interp_q15.o \
./DPS/Source/InterpolationFunctions/arm_linear_interp_q31.o \
./DPS/Source/InterpolationFunctions/arm_linear_interp_q7.o \
./DPS/Source/InterpolationFunctions/arm_spline_interp_f32.o \
./DPS/Source/InterpolationFunctions/arm_spline_interp_init_f32.o 

C_DEPS += \
./DPS/Source/InterpolationFunctions/InterpolationFunctionsF16.d \
./DPS/Source/InterpolationFunctions/arm_bilinear_interp_f16.d \
./DPS/Source/InterpolationFunctions/arm_bilinear_interp_f32.d \
./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q15.d \
./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q31.d \
./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q7.d \
./DPS/Source/InterpolationFunctions/arm_linear_interp_f16.d \
./DPS/Source/InterpolationFunctions/arm_linear_interp_f32.d \
./DPS/Source/InterpolationFunctions/arm_linear_interp_q15.d \
./DPS/Source/InterpolationFunctions/arm_linear_interp_q31.d \
./DPS/Source/InterpolationFunctions/arm_linear_interp_q7.d \
./DPS/Source/InterpolationFunctions/arm_spline_interp_f32.d \
./DPS/Source/InterpolationFunctions/arm_spline_interp_init_f32.d 


# Each subdirectory must supply rules for building sources it contributes
DPS/Source/InterpolationFunctions/%.o DPS/Source/InterpolationFunctions/%.su: ../DPS/Source/InterpolationFunctions/%.c DPS/Source/InterpolationFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Work/Micro/Project/Project/DPS/ComputeLibrary/Include" -I"C:/Work/Micro/Project/Project/DPS/Include" -I"C:/Work/Micro/Project/Project/DPS/PrivateInclude" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DPS-2f-Source-2f-InterpolationFunctions

clean-DPS-2f-Source-2f-InterpolationFunctions:
	-$(RM) ./DPS/Source/InterpolationFunctions/InterpolationFunctionsF16.d ./DPS/Source/InterpolationFunctions/InterpolationFunctionsF16.o ./DPS/Source/InterpolationFunctions/InterpolationFunctionsF16.su ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_f16.d ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_f16.o ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_f16.su ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_f32.d ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_f32.o ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_f32.su ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q15.d ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q15.o ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q15.su ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q31.d ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q31.o ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q31.su ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q7.d ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q7.o ./DPS/Source/InterpolationFunctions/arm_bilinear_interp_q7.su ./DPS/Source/InterpolationFunctions/arm_linear_interp_f16.d ./DPS/Source/InterpolationFunctions/arm_linear_interp_f16.o ./DPS/Source/InterpolationFunctions/arm_linear_interp_f16.su ./DPS/Source/InterpolationFunctions/arm_linear_interp_f32.d ./DPS/Source/InterpolationFunctions/arm_linear_interp_f32.o ./DPS/Source/InterpolationFunctions/arm_linear_interp_f32.su ./DPS/Source/InterpolationFunctions/arm_linear_interp_q15.d ./DPS/Source/InterpolationFunctions/arm_linear_interp_q15.o ./DPS/Source/InterpolationFunctions/arm_linear_interp_q15.su ./DPS/Source/InterpolationFunctions/arm_linear_interp_q31.d ./DPS/Source/InterpolationFunctions/arm_linear_interp_q31.o ./DPS/Source/InterpolationFunctions/arm_linear_interp_q31.su ./DPS/Source/InterpolationFunctions/arm_linear_interp_q7.d ./DPS/Source/InterpolationFunctions/arm_linear_interp_q7.o ./DPS/Source/InterpolationFunctions/arm_linear_interp_q7.su ./DPS/Source/InterpolationFunctions/arm_spline_interp_f32.d ./DPS/Source/InterpolationFunctions/arm_spline_interp_f32.o ./DPS/Source/InterpolationFunctions/arm_spline_interp_f32.su ./DPS/Source/InterpolationFunctions/arm_spline_interp_init_f32.d ./DPS/Source/InterpolationFunctions/arm_spline_interp_init_f32.o ./DPS/Source/InterpolationFunctions/arm_spline_interp_init_f32.su

.PHONY: clean-DPS-2f-Source-2f-InterpolationFunctions

