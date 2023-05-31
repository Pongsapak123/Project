################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DPS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.c \
../DPS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.c \
../DPS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.c \
../DPS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.c \
../DPS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.c \
../DPS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.c \
../DPS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.c \
../DPS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.c 

OBJS += \
./DPS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.o \
./DPS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.o \
./DPS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.o \
./DPS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.o \
./DPS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.o \
./DPS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.o \
./DPS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.o \
./DPS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.o 

C_DEPS += \
./DPS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.d \
./DPS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.d \
./DPS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.d \
./DPS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.d \
./DPS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.d \
./DPS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.d \
./DPS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.d \
./DPS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.d 


# Each subdirectory must supply rules for building sources it contributes
DPS/Source/QuaternionMathFunctions/%.o DPS/Source/QuaternionMathFunctions/%.su DPS/Source/QuaternionMathFunctions/%.cyclo: ../DPS/Source/QuaternionMathFunctions/%.c DPS/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Work/Micro/Project/Project/DPS/ComputeLibrary/Include" -I"C:/Work/Micro/Project/Project/DPS/Include" -I"C:/Work/Micro/Project/Project/DPS/PrivateInclude" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DPS-2f-Source-2f-QuaternionMathFunctions

clean-DPS-2f-Source-2f-QuaternionMathFunctions:
	-$(RM) ./DPS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.cyclo ./DPS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.d ./DPS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.o ./DPS/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.su ./DPS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.cyclo ./DPS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.d ./DPS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.o ./DPS/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.su ./DPS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.cyclo ./DPS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.d ./DPS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.o ./DPS/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.su ./DPS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.cyclo ./DPS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.d ./DPS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.o ./DPS/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.su ./DPS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.cyclo ./DPS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.d ./DPS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.o ./DPS/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.su ./DPS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.cyclo ./DPS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.d ./DPS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.o ./DPS/Source/QuaternionMathFunctions/arm_quaternion_product_f32.su ./DPS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.cyclo ./DPS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.d ./DPS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.o ./DPS/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.su ./DPS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.cyclo ./DPS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.d ./DPS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.o ./DPS/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.su

.PHONY: clean-DPS-2f-Source-2f-QuaternionMathFunctions

