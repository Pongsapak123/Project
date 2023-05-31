################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DPS/Source/SupportFunctions/SupportFunctionsF16.c \
../DPS/Source/SupportFunctions/arm_barycenter_f16.c \
../DPS/Source/SupportFunctions/arm_barycenter_f32.c \
../DPS/Source/SupportFunctions/arm_bitonic_sort_f32.c \
../DPS/Source/SupportFunctions/arm_bubble_sort_f32.c \
../DPS/Source/SupportFunctions/arm_copy_f16.c \
../DPS/Source/SupportFunctions/arm_copy_f32.c \
../DPS/Source/SupportFunctions/arm_copy_f64.c \
../DPS/Source/SupportFunctions/arm_copy_q15.c \
../DPS/Source/SupportFunctions/arm_copy_q31.c \
../DPS/Source/SupportFunctions/arm_copy_q7.c \
../DPS/Source/SupportFunctions/arm_f16_to_f64.c \
../DPS/Source/SupportFunctions/arm_f16_to_float.c \
../DPS/Source/SupportFunctions/arm_f16_to_q15.c \
../DPS/Source/SupportFunctions/arm_f64_to_f16.c \
../DPS/Source/SupportFunctions/arm_f64_to_float.c \
../DPS/Source/SupportFunctions/arm_f64_to_q15.c \
../DPS/Source/SupportFunctions/arm_f64_to_q31.c \
../DPS/Source/SupportFunctions/arm_f64_to_q7.c \
../DPS/Source/SupportFunctions/arm_fill_f16.c \
../DPS/Source/SupportFunctions/arm_fill_f32.c \
../DPS/Source/SupportFunctions/arm_fill_f64.c \
../DPS/Source/SupportFunctions/arm_fill_q15.c \
../DPS/Source/SupportFunctions/arm_fill_q31.c \
../DPS/Source/SupportFunctions/arm_fill_q7.c \
../DPS/Source/SupportFunctions/arm_float_to_f16.c \
../DPS/Source/SupportFunctions/arm_float_to_f64.c \
../DPS/Source/SupportFunctions/arm_float_to_q15.c \
../DPS/Source/SupportFunctions/arm_float_to_q31.c \
../DPS/Source/SupportFunctions/arm_float_to_q7.c \
../DPS/Source/SupportFunctions/arm_heap_sort_f32.c \
../DPS/Source/SupportFunctions/arm_insertion_sort_f32.c \
../DPS/Source/SupportFunctions/arm_merge_sort_f32.c \
../DPS/Source/SupportFunctions/arm_merge_sort_init_f32.c \
../DPS/Source/SupportFunctions/arm_q15_to_f16.c \
../DPS/Source/SupportFunctions/arm_q15_to_f64.c \
../DPS/Source/SupportFunctions/arm_q15_to_float.c \
../DPS/Source/SupportFunctions/arm_q15_to_q31.c \
../DPS/Source/SupportFunctions/arm_q15_to_q7.c \
../DPS/Source/SupportFunctions/arm_q31_to_f64.c \
../DPS/Source/SupportFunctions/arm_q31_to_float.c \
../DPS/Source/SupportFunctions/arm_q31_to_q15.c \
../DPS/Source/SupportFunctions/arm_q31_to_q7.c \
../DPS/Source/SupportFunctions/arm_q7_to_f64.c \
../DPS/Source/SupportFunctions/arm_q7_to_float.c \
../DPS/Source/SupportFunctions/arm_q7_to_q15.c \
../DPS/Source/SupportFunctions/arm_q7_to_q31.c \
../DPS/Source/SupportFunctions/arm_quick_sort_f32.c \
../DPS/Source/SupportFunctions/arm_selection_sort_f32.c \
../DPS/Source/SupportFunctions/arm_sort_f32.c \
../DPS/Source/SupportFunctions/arm_sort_init_f32.c \
../DPS/Source/SupportFunctions/arm_weighted_sum_f16.c \
../DPS/Source/SupportFunctions/arm_weighted_sum_f32.c 

OBJS += \
./DPS/Source/SupportFunctions/SupportFunctionsF16.o \
./DPS/Source/SupportFunctions/arm_barycenter_f16.o \
./DPS/Source/SupportFunctions/arm_barycenter_f32.o \
./DPS/Source/SupportFunctions/arm_bitonic_sort_f32.o \
./DPS/Source/SupportFunctions/arm_bubble_sort_f32.o \
./DPS/Source/SupportFunctions/arm_copy_f16.o \
./DPS/Source/SupportFunctions/arm_copy_f32.o \
./DPS/Source/SupportFunctions/arm_copy_f64.o \
./DPS/Source/SupportFunctions/arm_copy_q15.o \
./DPS/Source/SupportFunctions/arm_copy_q31.o \
./DPS/Source/SupportFunctions/arm_copy_q7.o \
./DPS/Source/SupportFunctions/arm_f16_to_f64.o \
./DPS/Source/SupportFunctions/arm_f16_to_float.o \
./DPS/Source/SupportFunctions/arm_f16_to_q15.o \
./DPS/Source/SupportFunctions/arm_f64_to_f16.o \
./DPS/Source/SupportFunctions/arm_f64_to_float.o \
./DPS/Source/SupportFunctions/arm_f64_to_q15.o \
./DPS/Source/SupportFunctions/arm_f64_to_q31.o \
./DPS/Source/SupportFunctions/arm_f64_to_q7.o \
./DPS/Source/SupportFunctions/arm_fill_f16.o \
./DPS/Source/SupportFunctions/arm_fill_f32.o \
./DPS/Source/SupportFunctions/arm_fill_f64.o \
./DPS/Source/SupportFunctions/arm_fill_q15.o \
./DPS/Source/SupportFunctions/arm_fill_q31.o \
./DPS/Source/SupportFunctions/arm_fill_q7.o \
./DPS/Source/SupportFunctions/arm_float_to_f16.o \
./DPS/Source/SupportFunctions/arm_float_to_f64.o \
./DPS/Source/SupportFunctions/arm_float_to_q15.o \
./DPS/Source/SupportFunctions/arm_float_to_q31.o \
./DPS/Source/SupportFunctions/arm_float_to_q7.o \
./DPS/Source/SupportFunctions/arm_heap_sort_f32.o \
./DPS/Source/SupportFunctions/arm_insertion_sort_f32.o \
./DPS/Source/SupportFunctions/arm_merge_sort_f32.o \
./DPS/Source/SupportFunctions/arm_merge_sort_init_f32.o \
./DPS/Source/SupportFunctions/arm_q15_to_f16.o \
./DPS/Source/SupportFunctions/arm_q15_to_f64.o \
./DPS/Source/SupportFunctions/arm_q15_to_float.o \
./DPS/Source/SupportFunctions/arm_q15_to_q31.o \
./DPS/Source/SupportFunctions/arm_q15_to_q7.o \
./DPS/Source/SupportFunctions/arm_q31_to_f64.o \
./DPS/Source/SupportFunctions/arm_q31_to_float.o \
./DPS/Source/SupportFunctions/arm_q31_to_q15.o \
./DPS/Source/SupportFunctions/arm_q31_to_q7.o \
./DPS/Source/SupportFunctions/arm_q7_to_f64.o \
./DPS/Source/SupportFunctions/arm_q7_to_float.o \
./DPS/Source/SupportFunctions/arm_q7_to_q15.o \
./DPS/Source/SupportFunctions/arm_q7_to_q31.o \
./DPS/Source/SupportFunctions/arm_quick_sort_f32.o \
./DPS/Source/SupportFunctions/arm_selection_sort_f32.o \
./DPS/Source/SupportFunctions/arm_sort_f32.o \
./DPS/Source/SupportFunctions/arm_sort_init_f32.o \
./DPS/Source/SupportFunctions/arm_weighted_sum_f16.o \
./DPS/Source/SupportFunctions/arm_weighted_sum_f32.o 

C_DEPS += \
./DPS/Source/SupportFunctions/SupportFunctionsF16.d \
./DPS/Source/SupportFunctions/arm_barycenter_f16.d \
./DPS/Source/SupportFunctions/arm_barycenter_f32.d \
./DPS/Source/SupportFunctions/arm_bitonic_sort_f32.d \
./DPS/Source/SupportFunctions/arm_bubble_sort_f32.d \
./DPS/Source/SupportFunctions/arm_copy_f16.d \
./DPS/Source/SupportFunctions/arm_copy_f32.d \
./DPS/Source/SupportFunctions/arm_copy_f64.d \
./DPS/Source/SupportFunctions/arm_copy_q15.d \
./DPS/Source/SupportFunctions/arm_copy_q31.d \
./DPS/Source/SupportFunctions/arm_copy_q7.d \
./DPS/Source/SupportFunctions/arm_f16_to_f64.d \
./DPS/Source/SupportFunctions/arm_f16_to_float.d \
./DPS/Source/SupportFunctions/arm_f16_to_q15.d \
./DPS/Source/SupportFunctions/arm_f64_to_f16.d \
./DPS/Source/SupportFunctions/arm_f64_to_float.d \
./DPS/Source/SupportFunctions/arm_f64_to_q15.d \
./DPS/Source/SupportFunctions/arm_f64_to_q31.d \
./DPS/Source/SupportFunctions/arm_f64_to_q7.d \
./DPS/Source/SupportFunctions/arm_fill_f16.d \
./DPS/Source/SupportFunctions/arm_fill_f32.d \
./DPS/Source/SupportFunctions/arm_fill_f64.d \
./DPS/Source/SupportFunctions/arm_fill_q15.d \
./DPS/Source/SupportFunctions/arm_fill_q31.d \
./DPS/Source/SupportFunctions/arm_fill_q7.d \
./DPS/Source/SupportFunctions/arm_float_to_f16.d \
./DPS/Source/SupportFunctions/arm_float_to_f64.d \
./DPS/Source/SupportFunctions/arm_float_to_q15.d \
./DPS/Source/SupportFunctions/arm_float_to_q31.d \
./DPS/Source/SupportFunctions/arm_float_to_q7.d \
./DPS/Source/SupportFunctions/arm_heap_sort_f32.d \
./DPS/Source/SupportFunctions/arm_insertion_sort_f32.d \
./DPS/Source/SupportFunctions/arm_merge_sort_f32.d \
./DPS/Source/SupportFunctions/arm_merge_sort_init_f32.d \
./DPS/Source/SupportFunctions/arm_q15_to_f16.d \
./DPS/Source/SupportFunctions/arm_q15_to_f64.d \
./DPS/Source/SupportFunctions/arm_q15_to_float.d \
./DPS/Source/SupportFunctions/arm_q15_to_q31.d \
./DPS/Source/SupportFunctions/arm_q15_to_q7.d \
./DPS/Source/SupportFunctions/arm_q31_to_f64.d \
./DPS/Source/SupportFunctions/arm_q31_to_float.d \
./DPS/Source/SupportFunctions/arm_q31_to_q15.d \
./DPS/Source/SupportFunctions/arm_q31_to_q7.d \
./DPS/Source/SupportFunctions/arm_q7_to_f64.d \
./DPS/Source/SupportFunctions/arm_q7_to_float.d \
./DPS/Source/SupportFunctions/arm_q7_to_q15.d \
./DPS/Source/SupportFunctions/arm_q7_to_q31.d \
./DPS/Source/SupportFunctions/arm_quick_sort_f32.d \
./DPS/Source/SupportFunctions/arm_selection_sort_f32.d \
./DPS/Source/SupportFunctions/arm_sort_f32.d \
./DPS/Source/SupportFunctions/arm_sort_init_f32.d \
./DPS/Source/SupportFunctions/arm_weighted_sum_f16.d \
./DPS/Source/SupportFunctions/arm_weighted_sum_f32.d 


# Each subdirectory must supply rules for building sources it contributes
DPS/Source/SupportFunctions/%.o DPS/Source/SupportFunctions/%.su DPS/Source/SupportFunctions/%.cyclo: ../DPS/Source/SupportFunctions/%.c DPS/Source/SupportFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Work/Micro/Project/Project/DPS/ComputeLibrary/Include" -I"C:/Work/Micro/Project/Project/DPS/Include" -I"C:/Work/Micro/Project/Project/DPS/PrivateInclude" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DPS-2f-Source-2f-SupportFunctions

clean-DPS-2f-Source-2f-SupportFunctions:
	-$(RM) ./DPS/Source/SupportFunctions/SupportFunctionsF16.cyclo ./DPS/Source/SupportFunctions/SupportFunctionsF16.d ./DPS/Source/SupportFunctions/SupportFunctionsF16.o ./DPS/Source/SupportFunctions/SupportFunctionsF16.su ./DPS/Source/SupportFunctions/arm_barycenter_f16.cyclo ./DPS/Source/SupportFunctions/arm_barycenter_f16.d ./DPS/Source/SupportFunctions/arm_barycenter_f16.o ./DPS/Source/SupportFunctions/arm_barycenter_f16.su ./DPS/Source/SupportFunctions/arm_barycenter_f32.cyclo ./DPS/Source/SupportFunctions/arm_barycenter_f32.d ./DPS/Source/SupportFunctions/arm_barycenter_f32.o ./DPS/Source/SupportFunctions/arm_barycenter_f32.su ./DPS/Source/SupportFunctions/arm_bitonic_sort_f32.cyclo ./DPS/Source/SupportFunctions/arm_bitonic_sort_f32.d ./DPS/Source/SupportFunctions/arm_bitonic_sort_f32.o ./DPS/Source/SupportFunctions/arm_bitonic_sort_f32.su ./DPS/Source/SupportFunctions/arm_bubble_sort_f32.cyclo ./DPS/Source/SupportFunctions/arm_bubble_sort_f32.d ./DPS/Source/SupportFunctions/arm_bubble_sort_f32.o ./DPS/Source/SupportFunctions/arm_bubble_sort_f32.su ./DPS/Source/SupportFunctions/arm_copy_f16.cyclo ./DPS/Source/SupportFunctions/arm_copy_f16.d ./DPS/Source/SupportFunctions/arm_copy_f16.o ./DPS/Source/SupportFunctions/arm_copy_f16.su ./DPS/Source/SupportFunctions/arm_copy_f32.cyclo ./DPS/Source/SupportFunctions/arm_copy_f32.d ./DPS/Source/SupportFunctions/arm_copy_f32.o ./DPS/Source/SupportFunctions/arm_copy_f32.su ./DPS/Source/SupportFunctions/arm_copy_f64.cyclo ./DPS/Source/SupportFunctions/arm_copy_f64.d ./DPS/Source/SupportFunctions/arm_copy_f64.o ./DPS/Source/SupportFunctions/arm_copy_f64.su ./DPS/Source/SupportFunctions/arm_copy_q15.cyclo ./DPS/Source/SupportFunctions/arm_copy_q15.d ./DPS/Source/SupportFunctions/arm_copy_q15.o ./DPS/Source/SupportFunctions/arm_copy_q15.su ./DPS/Source/SupportFunctions/arm_copy_q31.cyclo ./DPS/Source/SupportFunctions/arm_copy_q31.d ./DPS/Source/SupportFunctions/arm_copy_q31.o ./DPS/Source/SupportFunctions/arm_copy_q31.su ./DPS/Source/SupportFunctions/arm_copy_q7.cyclo ./DPS/Source/SupportFunctions/arm_copy_q7.d ./DPS/Source/SupportFunctions/arm_copy_q7.o ./DPS/Source/SupportFunctions/arm_copy_q7.su ./DPS/Source/SupportFunctions/arm_f16_to_f64.cyclo ./DPS/Source/SupportFunctions/arm_f16_to_f64.d ./DPS/Source/SupportFunctions/arm_f16_to_f64.o ./DPS/Source/SupportFunctions/arm_f16_to_f64.su ./DPS/Source/SupportFunctions/arm_f16_to_float.cyclo ./DPS/Source/SupportFunctions/arm_f16_to_float.d ./DPS/Source/SupportFunctions/arm_f16_to_float.o ./DPS/Source/SupportFunctions/arm_f16_to_float.su ./DPS/Source/SupportFunctions/arm_f16_to_q15.cyclo ./DPS/Source/SupportFunctions/arm_f16_to_q15.d ./DPS/Source/SupportFunctions/arm_f16_to_q15.o ./DPS/Source/SupportFunctions/arm_f16_to_q15.su ./DPS/Source/SupportFunctions/arm_f64_to_f16.cyclo ./DPS/Source/SupportFunctions/arm_f64_to_f16.d ./DPS/Source/SupportFunctions/arm_f64_to_f16.o ./DPS/Source/SupportFunctions/arm_f64_to_f16.su ./DPS/Source/SupportFunctions/arm_f64_to_float.cyclo ./DPS/Source/SupportFunctions/arm_f64_to_float.d ./DPS/Source/SupportFunctions/arm_f64_to_float.o ./DPS/Source/SupportFunctions/arm_f64_to_float.su ./DPS/Source/SupportFunctions/arm_f64_to_q15.cyclo ./DPS/Source/SupportFunctions/arm_f64_to_q15.d ./DPS/Source/SupportFunctions/arm_f64_to_q15.o ./DPS/Source/SupportFunctions/arm_f64_to_q15.su ./DPS/Source/SupportFunctions/arm_f64_to_q31.cyclo ./DPS/Source/SupportFunctions/arm_f64_to_q31.d ./DPS/Source/SupportFunctions/arm_f64_to_q31.o ./DPS/Source/SupportFunctions/arm_f64_to_q31.su ./DPS/Source/SupportFunctions/arm_f64_to_q7.cyclo ./DPS/Source/SupportFunctions/arm_f64_to_q7.d ./DPS/Source/SupportFunctions/arm_f64_to_q7.o ./DPS/Source/SupportFunctions/arm_f64_to_q7.su ./DPS/Source/SupportFunctions/arm_fill_f16.cyclo ./DPS/Source/SupportFunctions/arm_fill_f16.d ./DPS/Source/SupportFunctions/arm_fill_f16.o ./DPS/Source/SupportFunctions/arm_fill_f16.su ./DPS/Source/SupportFunctions/arm_fill_f32.cyclo ./DPS/Source/SupportFunctions/arm_fill_f32.d ./DPS/Source/SupportFunctions/arm_fill_f32.o ./DPS/Source/SupportFunctions/arm_fill_f32.su ./DPS/Source/SupportFunctions/arm_fill_f64.cyclo ./DPS/Source/SupportFunctions/arm_fill_f64.d ./DPS/Source/SupportFunctions/arm_fill_f64.o ./DPS/Source/SupportFunctions/arm_fill_f64.su ./DPS/Source/SupportFunctions/arm_fill_q15.cyclo ./DPS/Source/SupportFunctions/arm_fill_q15.d ./DPS/Source/SupportFunctions/arm_fill_q15.o ./DPS/Source/SupportFunctions/arm_fill_q15.su ./DPS/Source/SupportFunctions/arm_fill_q31.cyclo ./DPS/Source/SupportFunctions/arm_fill_q31.d ./DPS/Source/SupportFunctions/arm_fill_q31.o ./DPS/Source/SupportFunctions/arm_fill_q31.su ./DPS/Source/SupportFunctions/arm_fill_q7.cyclo ./DPS/Source/SupportFunctions/arm_fill_q7.d ./DPS/Source/SupportFunctions/arm_fill_q7.o ./DPS/Source/SupportFunctions/arm_fill_q7.su ./DPS/Source/SupportFunctions/arm_float_to_f16.cyclo ./DPS/Source/SupportFunctions/arm_float_to_f16.d ./DPS/Source/SupportFunctions/arm_float_to_f16.o ./DPS/Source/SupportFunctions/arm_float_to_f16.su ./DPS/Source/SupportFunctions/arm_float_to_f64.cyclo ./DPS/Source/SupportFunctions/arm_float_to_f64.d ./DPS/Source/SupportFunctions/arm_float_to_f64.o ./DPS/Source/SupportFunctions/arm_float_to_f64.su ./DPS/Source/SupportFunctions/arm_float_to_q15.cyclo ./DPS/Source/SupportFunctions/arm_float_to_q15.d ./DPS/Source/SupportFunctions/arm_float_to_q15.o ./DPS/Source/SupportFunctions/arm_float_to_q15.su ./DPS/Source/SupportFunctions/arm_float_to_q31.cyclo ./DPS/Source/SupportFunctions/arm_float_to_q31.d ./DPS/Source/SupportFunctions/arm_float_to_q31.o ./DPS/Source/SupportFunctions/arm_float_to_q31.su ./DPS/Source/SupportFunctions/arm_float_to_q7.cyclo ./DPS/Source/SupportFunctions/arm_float_to_q7.d ./DPS/Source/SupportFunctions/arm_float_to_q7.o ./DPS/Source/SupportFunctions/arm_float_to_q7.su ./DPS/Source/SupportFunctions/arm_heap_sort_f32.cyclo ./DPS/Source/SupportFunctions/arm_heap_sort_f32.d ./DPS/Source/SupportFunctions/arm_heap_sort_f32.o
	-$(RM) ./DPS/Source/SupportFunctions/arm_heap_sort_f32.su ./DPS/Source/SupportFunctions/arm_insertion_sort_f32.cyclo ./DPS/Source/SupportFunctions/arm_insertion_sort_f32.d ./DPS/Source/SupportFunctions/arm_insertion_sort_f32.o ./DPS/Source/SupportFunctions/arm_insertion_sort_f32.su ./DPS/Source/SupportFunctions/arm_merge_sort_f32.cyclo ./DPS/Source/SupportFunctions/arm_merge_sort_f32.d ./DPS/Source/SupportFunctions/arm_merge_sort_f32.o ./DPS/Source/SupportFunctions/arm_merge_sort_f32.su ./DPS/Source/SupportFunctions/arm_merge_sort_init_f32.cyclo ./DPS/Source/SupportFunctions/arm_merge_sort_init_f32.d ./DPS/Source/SupportFunctions/arm_merge_sort_init_f32.o ./DPS/Source/SupportFunctions/arm_merge_sort_init_f32.su ./DPS/Source/SupportFunctions/arm_q15_to_f16.cyclo ./DPS/Source/SupportFunctions/arm_q15_to_f16.d ./DPS/Source/SupportFunctions/arm_q15_to_f16.o ./DPS/Source/SupportFunctions/arm_q15_to_f16.su ./DPS/Source/SupportFunctions/arm_q15_to_f64.cyclo ./DPS/Source/SupportFunctions/arm_q15_to_f64.d ./DPS/Source/SupportFunctions/arm_q15_to_f64.o ./DPS/Source/SupportFunctions/arm_q15_to_f64.su ./DPS/Source/SupportFunctions/arm_q15_to_float.cyclo ./DPS/Source/SupportFunctions/arm_q15_to_float.d ./DPS/Source/SupportFunctions/arm_q15_to_float.o ./DPS/Source/SupportFunctions/arm_q15_to_float.su ./DPS/Source/SupportFunctions/arm_q15_to_q31.cyclo ./DPS/Source/SupportFunctions/arm_q15_to_q31.d ./DPS/Source/SupportFunctions/arm_q15_to_q31.o ./DPS/Source/SupportFunctions/arm_q15_to_q31.su ./DPS/Source/SupportFunctions/arm_q15_to_q7.cyclo ./DPS/Source/SupportFunctions/arm_q15_to_q7.d ./DPS/Source/SupportFunctions/arm_q15_to_q7.o ./DPS/Source/SupportFunctions/arm_q15_to_q7.su ./DPS/Source/SupportFunctions/arm_q31_to_f64.cyclo ./DPS/Source/SupportFunctions/arm_q31_to_f64.d ./DPS/Source/SupportFunctions/arm_q31_to_f64.o ./DPS/Source/SupportFunctions/arm_q31_to_f64.su ./DPS/Source/SupportFunctions/arm_q31_to_float.cyclo ./DPS/Source/SupportFunctions/arm_q31_to_float.d ./DPS/Source/SupportFunctions/arm_q31_to_float.o ./DPS/Source/SupportFunctions/arm_q31_to_float.su ./DPS/Source/SupportFunctions/arm_q31_to_q15.cyclo ./DPS/Source/SupportFunctions/arm_q31_to_q15.d ./DPS/Source/SupportFunctions/arm_q31_to_q15.o ./DPS/Source/SupportFunctions/arm_q31_to_q15.su ./DPS/Source/SupportFunctions/arm_q31_to_q7.cyclo ./DPS/Source/SupportFunctions/arm_q31_to_q7.d ./DPS/Source/SupportFunctions/arm_q31_to_q7.o ./DPS/Source/SupportFunctions/arm_q31_to_q7.su ./DPS/Source/SupportFunctions/arm_q7_to_f64.cyclo ./DPS/Source/SupportFunctions/arm_q7_to_f64.d ./DPS/Source/SupportFunctions/arm_q7_to_f64.o ./DPS/Source/SupportFunctions/arm_q7_to_f64.su ./DPS/Source/SupportFunctions/arm_q7_to_float.cyclo ./DPS/Source/SupportFunctions/arm_q7_to_float.d ./DPS/Source/SupportFunctions/arm_q7_to_float.o ./DPS/Source/SupportFunctions/arm_q7_to_float.su ./DPS/Source/SupportFunctions/arm_q7_to_q15.cyclo ./DPS/Source/SupportFunctions/arm_q7_to_q15.d ./DPS/Source/SupportFunctions/arm_q7_to_q15.o ./DPS/Source/SupportFunctions/arm_q7_to_q15.su ./DPS/Source/SupportFunctions/arm_q7_to_q31.cyclo ./DPS/Source/SupportFunctions/arm_q7_to_q31.d ./DPS/Source/SupportFunctions/arm_q7_to_q31.o ./DPS/Source/SupportFunctions/arm_q7_to_q31.su ./DPS/Source/SupportFunctions/arm_quick_sort_f32.cyclo ./DPS/Source/SupportFunctions/arm_quick_sort_f32.d ./DPS/Source/SupportFunctions/arm_quick_sort_f32.o ./DPS/Source/SupportFunctions/arm_quick_sort_f32.su ./DPS/Source/SupportFunctions/arm_selection_sort_f32.cyclo ./DPS/Source/SupportFunctions/arm_selection_sort_f32.d ./DPS/Source/SupportFunctions/arm_selection_sort_f32.o ./DPS/Source/SupportFunctions/arm_selection_sort_f32.su ./DPS/Source/SupportFunctions/arm_sort_f32.cyclo ./DPS/Source/SupportFunctions/arm_sort_f32.d ./DPS/Source/SupportFunctions/arm_sort_f32.o ./DPS/Source/SupportFunctions/arm_sort_f32.su ./DPS/Source/SupportFunctions/arm_sort_init_f32.cyclo ./DPS/Source/SupportFunctions/arm_sort_init_f32.d ./DPS/Source/SupportFunctions/arm_sort_init_f32.o ./DPS/Source/SupportFunctions/arm_sort_init_f32.su ./DPS/Source/SupportFunctions/arm_weighted_sum_f16.cyclo ./DPS/Source/SupportFunctions/arm_weighted_sum_f16.d ./DPS/Source/SupportFunctions/arm_weighted_sum_f16.o ./DPS/Source/SupportFunctions/arm_weighted_sum_f16.su ./DPS/Source/SupportFunctions/arm_weighted_sum_f32.cyclo ./DPS/Source/SupportFunctions/arm_weighted_sum_f32.d ./DPS/Source/SupportFunctions/arm_weighted_sum_f32.o ./DPS/Source/SupportFunctions/arm_weighted_sum_f32.su

.PHONY: clean-DPS-2f-Source-2f-SupportFunctions

