################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DPS/Source/StatisticsFunctions/StatisticsFunctionsF16.c \
../DPS/Source/StatisticsFunctions/arm_absmax_f16.c \
../DPS/Source/StatisticsFunctions/arm_absmax_f32.c \
../DPS/Source/StatisticsFunctions/arm_absmax_f64.c \
../DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f16.c \
../DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f32.c \
../DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f64.c \
../DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q15.c \
../DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q31.c \
../DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q7.c \
../DPS/Source/StatisticsFunctions/arm_absmax_q15.c \
../DPS/Source/StatisticsFunctions/arm_absmax_q31.c \
../DPS/Source/StatisticsFunctions/arm_absmax_q7.c \
../DPS/Source/StatisticsFunctions/arm_absmin_f16.c \
../DPS/Source/StatisticsFunctions/arm_absmin_f32.c \
../DPS/Source/StatisticsFunctions/arm_absmin_f64.c \
../DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f16.c \
../DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f32.c \
../DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f64.c \
../DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q15.c \
../DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q31.c \
../DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q7.c \
../DPS/Source/StatisticsFunctions/arm_absmin_q15.c \
../DPS/Source/StatisticsFunctions/arm_absmin_q31.c \
../DPS/Source/StatisticsFunctions/arm_absmin_q7.c \
../DPS/Source/StatisticsFunctions/arm_accumulate_f16.c \
../DPS/Source/StatisticsFunctions/arm_accumulate_f32.c \
../DPS/Source/StatisticsFunctions/arm_accumulate_f64.c \
../DPS/Source/StatisticsFunctions/arm_entropy_f16.c \
../DPS/Source/StatisticsFunctions/arm_entropy_f32.c \
../DPS/Source/StatisticsFunctions/arm_entropy_f64.c \
../DPS/Source/StatisticsFunctions/arm_kullback_leibler_f16.c \
../DPS/Source/StatisticsFunctions/arm_kullback_leibler_f32.c \
../DPS/Source/StatisticsFunctions/arm_kullback_leibler_f64.c \
../DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f16.c \
../DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f32.c \
../DPS/Source/StatisticsFunctions/arm_logsumexp_f16.c \
../DPS/Source/StatisticsFunctions/arm_logsumexp_f32.c \
../DPS/Source/StatisticsFunctions/arm_max_f16.c \
../DPS/Source/StatisticsFunctions/arm_max_f32.c \
../DPS/Source/StatisticsFunctions/arm_max_f64.c \
../DPS/Source/StatisticsFunctions/arm_max_no_idx_f16.c \
../DPS/Source/StatisticsFunctions/arm_max_no_idx_f32.c \
../DPS/Source/StatisticsFunctions/arm_max_no_idx_f64.c \
../DPS/Source/StatisticsFunctions/arm_max_no_idx_q15.c \
../DPS/Source/StatisticsFunctions/arm_max_no_idx_q31.c \
../DPS/Source/StatisticsFunctions/arm_max_no_idx_q7.c \
../DPS/Source/StatisticsFunctions/arm_max_q15.c \
../DPS/Source/StatisticsFunctions/arm_max_q31.c \
../DPS/Source/StatisticsFunctions/arm_max_q7.c \
../DPS/Source/StatisticsFunctions/arm_mean_f16.c \
../DPS/Source/StatisticsFunctions/arm_mean_f32.c \
../DPS/Source/StatisticsFunctions/arm_mean_f64.c \
../DPS/Source/StatisticsFunctions/arm_mean_q15.c \
../DPS/Source/StatisticsFunctions/arm_mean_q31.c \
../DPS/Source/StatisticsFunctions/arm_mean_q7.c \
../DPS/Source/StatisticsFunctions/arm_min_f16.c \
../DPS/Source/StatisticsFunctions/arm_min_f32.c \
../DPS/Source/StatisticsFunctions/arm_min_f64.c \
../DPS/Source/StatisticsFunctions/arm_min_no_idx_f16.c \
../DPS/Source/StatisticsFunctions/arm_min_no_idx_f32.c \
../DPS/Source/StatisticsFunctions/arm_min_no_idx_f64.c \
../DPS/Source/StatisticsFunctions/arm_min_no_idx_q15.c \
../DPS/Source/StatisticsFunctions/arm_min_no_idx_q31.c \
../DPS/Source/StatisticsFunctions/arm_min_no_idx_q7.c \
../DPS/Source/StatisticsFunctions/arm_min_q15.c \
../DPS/Source/StatisticsFunctions/arm_min_q31.c \
../DPS/Source/StatisticsFunctions/arm_min_q7.c \
../DPS/Source/StatisticsFunctions/arm_mse_f16.c \
../DPS/Source/StatisticsFunctions/arm_mse_f32.c \
../DPS/Source/StatisticsFunctions/arm_mse_f64.c \
../DPS/Source/StatisticsFunctions/arm_mse_q15.c \
../DPS/Source/StatisticsFunctions/arm_mse_q31.c \
../DPS/Source/StatisticsFunctions/arm_mse_q7.c \
../DPS/Source/StatisticsFunctions/arm_power_f16.c \
../DPS/Source/StatisticsFunctions/arm_power_f32.c \
../DPS/Source/StatisticsFunctions/arm_power_f64.c \
../DPS/Source/StatisticsFunctions/arm_power_q15.c \
../DPS/Source/StatisticsFunctions/arm_power_q31.c \
../DPS/Source/StatisticsFunctions/arm_power_q7.c \
../DPS/Source/StatisticsFunctions/arm_rms_f16.c \
../DPS/Source/StatisticsFunctions/arm_rms_f32.c \
../DPS/Source/StatisticsFunctions/arm_rms_q15.c \
../DPS/Source/StatisticsFunctions/arm_rms_q31.c \
../DPS/Source/StatisticsFunctions/arm_std_f16.c \
../DPS/Source/StatisticsFunctions/arm_std_f32.c \
../DPS/Source/StatisticsFunctions/arm_std_f64.c \
../DPS/Source/StatisticsFunctions/arm_std_q15.c \
../DPS/Source/StatisticsFunctions/arm_std_q31.c \
../DPS/Source/StatisticsFunctions/arm_var_f16.c \
../DPS/Source/StatisticsFunctions/arm_var_f32.c \
../DPS/Source/StatisticsFunctions/arm_var_f64.c \
../DPS/Source/StatisticsFunctions/arm_var_q15.c \
../DPS/Source/StatisticsFunctions/arm_var_q31.c 

OBJS += \
./DPS/Source/StatisticsFunctions/StatisticsFunctionsF16.o \
./DPS/Source/StatisticsFunctions/arm_absmax_f16.o \
./DPS/Source/StatisticsFunctions/arm_absmax_f32.o \
./DPS/Source/StatisticsFunctions/arm_absmax_f64.o \
./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f16.o \
./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f32.o \
./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f64.o \
./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q15.o \
./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q31.o \
./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q7.o \
./DPS/Source/StatisticsFunctions/arm_absmax_q15.o \
./DPS/Source/StatisticsFunctions/arm_absmax_q31.o \
./DPS/Source/StatisticsFunctions/arm_absmax_q7.o \
./DPS/Source/StatisticsFunctions/arm_absmin_f16.o \
./DPS/Source/StatisticsFunctions/arm_absmin_f32.o \
./DPS/Source/StatisticsFunctions/arm_absmin_f64.o \
./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f16.o \
./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f32.o \
./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f64.o \
./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q15.o \
./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q31.o \
./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q7.o \
./DPS/Source/StatisticsFunctions/arm_absmin_q15.o \
./DPS/Source/StatisticsFunctions/arm_absmin_q31.o \
./DPS/Source/StatisticsFunctions/arm_absmin_q7.o \
./DPS/Source/StatisticsFunctions/arm_accumulate_f16.o \
./DPS/Source/StatisticsFunctions/arm_accumulate_f32.o \
./DPS/Source/StatisticsFunctions/arm_accumulate_f64.o \
./DPS/Source/StatisticsFunctions/arm_entropy_f16.o \
./DPS/Source/StatisticsFunctions/arm_entropy_f32.o \
./DPS/Source/StatisticsFunctions/arm_entropy_f64.o \
./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f16.o \
./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f32.o \
./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f64.o \
./DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f16.o \
./DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f32.o \
./DPS/Source/StatisticsFunctions/arm_logsumexp_f16.o \
./DPS/Source/StatisticsFunctions/arm_logsumexp_f32.o \
./DPS/Source/StatisticsFunctions/arm_max_f16.o \
./DPS/Source/StatisticsFunctions/arm_max_f32.o \
./DPS/Source/StatisticsFunctions/arm_max_f64.o \
./DPS/Source/StatisticsFunctions/arm_max_no_idx_f16.o \
./DPS/Source/StatisticsFunctions/arm_max_no_idx_f32.o \
./DPS/Source/StatisticsFunctions/arm_max_no_idx_f64.o \
./DPS/Source/StatisticsFunctions/arm_max_no_idx_q15.o \
./DPS/Source/StatisticsFunctions/arm_max_no_idx_q31.o \
./DPS/Source/StatisticsFunctions/arm_max_no_idx_q7.o \
./DPS/Source/StatisticsFunctions/arm_max_q15.o \
./DPS/Source/StatisticsFunctions/arm_max_q31.o \
./DPS/Source/StatisticsFunctions/arm_max_q7.o \
./DPS/Source/StatisticsFunctions/arm_mean_f16.o \
./DPS/Source/StatisticsFunctions/arm_mean_f32.o \
./DPS/Source/StatisticsFunctions/arm_mean_f64.o \
./DPS/Source/StatisticsFunctions/arm_mean_q15.o \
./DPS/Source/StatisticsFunctions/arm_mean_q31.o \
./DPS/Source/StatisticsFunctions/arm_mean_q7.o \
./DPS/Source/StatisticsFunctions/arm_min_f16.o \
./DPS/Source/StatisticsFunctions/arm_min_f32.o \
./DPS/Source/StatisticsFunctions/arm_min_f64.o \
./DPS/Source/StatisticsFunctions/arm_min_no_idx_f16.o \
./DPS/Source/StatisticsFunctions/arm_min_no_idx_f32.o \
./DPS/Source/StatisticsFunctions/arm_min_no_idx_f64.o \
./DPS/Source/StatisticsFunctions/arm_min_no_idx_q15.o \
./DPS/Source/StatisticsFunctions/arm_min_no_idx_q31.o \
./DPS/Source/StatisticsFunctions/arm_min_no_idx_q7.o \
./DPS/Source/StatisticsFunctions/arm_min_q15.o \
./DPS/Source/StatisticsFunctions/arm_min_q31.o \
./DPS/Source/StatisticsFunctions/arm_min_q7.o \
./DPS/Source/StatisticsFunctions/arm_mse_f16.o \
./DPS/Source/StatisticsFunctions/arm_mse_f32.o \
./DPS/Source/StatisticsFunctions/arm_mse_f64.o \
./DPS/Source/StatisticsFunctions/arm_mse_q15.o \
./DPS/Source/StatisticsFunctions/arm_mse_q31.o \
./DPS/Source/StatisticsFunctions/arm_mse_q7.o \
./DPS/Source/StatisticsFunctions/arm_power_f16.o \
./DPS/Source/StatisticsFunctions/arm_power_f32.o \
./DPS/Source/StatisticsFunctions/arm_power_f64.o \
./DPS/Source/StatisticsFunctions/arm_power_q15.o \
./DPS/Source/StatisticsFunctions/arm_power_q31.o \
./DPS/Source/StatisticsFunctions/arm_power_q7.o \
./DPS/Source/StatisticsFunctions/arm_rms_f16.o \
./DPS/Source/StatisticsFunctions/arm_rms_f32.o \
./DPS/Source/StatisticsFunctions/arm_rms_q15.o \
./DPS/Source/StatisticsFunctions/arm_rms_q31.o \
./DPS/Source/StatisticsFunctions/arm_std_f16.o \
./DPS/Source/StatisticsFunctions/arm_std_f32.o \
./DPS/Source/StatisticsFunctions/arm_std_f64.o \
./DPS/Source/StatisticsFunctions/arm_std_q15.o \
./DPS/Source/StatisticsFunctions/arm_std_q31.o \
./DPS/Source/StatisticsFunctions/arm_var_f16.o \
./DPS/Source/StatisticsFunctions/arm_var_f32.o \
./DPS/Source/StatisticsFunctions/arm_var_f64.o \
./DPS/Source/StatisticsFunctions/arm_var_q15.o \
./DPS/Source/StatisticsFunctions/arm_var_q31.o 

C_DEPS += \
./DPS/Source/StatisticsFunctions/StatisticsFunctionsF16.d \
./DPS/Source/StatisticsFunctions/arm_absmax_f16.d \
./DPS/Source/StatisticsFunctions/arm_absmax_f32.d \
./DPS/Source/StatisticsFunctions/arm_absmax_f64.d \
./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f16.d \
./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f32.d \
./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f64.d \
./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q15.d \
./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q31.d \
./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q7.d \
./DPS/Source/StatisticsFunctions/arm_absmax_q15.d \
./DPS/Source/StatisticsFunctions/arm_absmax_q31.d \
./DPS/Source/StatisticsFunctions/arm_absmax_q7.d \
./DPS/Source/StatisticsFunctions/arm_absmin_f16.d \
./DPS/Source/StatisticsFunctions/arm_absmin_f32.d \
./DPS/Source/StatisticsFunctions/arm_absmin_f64.d \
./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f16.d \
./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f32.d \
./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f64.d \
./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q15.d \
./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q31.d \
./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q7.d \
./DPS/Source/StatisticsFunctions/arm_absmin_q15.d \
./DPS/Source/StatisticsFunctions/arm_absmin_q31.d \
./DPS/Source/StatisticsFunctions/arm_absmin_q7.d \
./DPS/Source/StatisticsFunctions/arm_accumulate_f16.d \
./DPS/Source/StatisticsFunctions/arm_accumulate_f32.d \
./DPS/Source/StatisticsFunctions/arm_accumulate_f64.d \
./DPS/Source/StatisticsFunctions/arm_entropy_f16.d \
./DPS/Source/StatisticsFunctions/arm_entropy_f32.d \
./DPS/Source/StatisticsFunctions/arm_entropy_f64.d \
./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f16.d \
./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f32.d \
./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f64.d \
./DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f16.d \
./DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f32.d \
./DPS/Source/StatisticsFunctions/arm_logsumexp_f16.d \
./DPS/Source/StatisticsFunctions/arm_logsumexp_f32.d \
./DPS/Source/StatisticsFunctions/arm_max_f16.d \
./DPS/Source/StatisticsFunctions/arm_max_f32.d \
./DPS/Source/StatisticsFunctions/arm_max_f64.d \
./DPS/Source/StatisticsFunctions/arm_max_no_idx_f16.d \
./DPS/Source/StatisticsFunctions/arm_max_no_idx_f32.d \
./DPS/Source/StatisticsFunctions/arm_max_no_idx_f64.d \
./DPS/Source/StatisticsFunctions/arm_max_no_idx_q15.d \
./DPS/Source/StatisticsFunctions/arm_max_no_idx_q31.d \
./DPS/Source/StatisticsFunctions/arm_max_no_idx_q7.d \
./DPS/Source/StatisticsFunctions/arm_max_q15.d \
./DPS/Source/StatisticsFunctions/arm_max_q31.d \
./DPS/Source/StatisticsFunctions/arm_max_q7.d \
./DPS/Source/StatisticsFunctions/arm_mean_f16.d \
./DPS/Source/StatisticsFunctions/arm_mean_f32.d \
./DPS/Source/StatisticsFunctions/arm_mean_f64.d \
./DPS/Source/StatisticsFunctions/arm_mean_q15.d \
./DPS/Source/StatisticsFunctions/arm_mean_q31.d \
./DPS/Source/StatisticsFunctions/arm_mean_q7.d \
./DPS/Source/StatisticsFunctions/arm_min_f16.d \
./DPS/Source/StatisticsFunctions/arm_min_f32.d \
./DPS/Source/StatisticsFunctions/arm_min_f64.d \
./DPS/Source/StatisticsFunctions/arm_min_no_idx_f16.d \
./DPS/Source/StatisticsFunctions/arm_min_no_idx_f32.d \
./DPS/Source/StatisticsFunctions/arm_min_no_idx_f64.d \
./DPS/Source/StatisticsFunctions/arm_min_no_idx_q15.d \
./DPS/Source/StatisticsFunctions/arm_min_no_idx_q31.d \
./DPS/Source/StatisticsFunctions/arm_min_no_idx_q7.d \
./DPS/Source/StatisticsFunctions/arm_min_q15.d \
./DPS/Source/StatisticsFunctions/arm_min_q31.d \
./DPS/Source/StatisticsFunctions/arm_min_q7.d \
./DPS/Source/StatisticsFunctions/arm_mse_f16.d \
./DPS/Source/StatisticsFunctions/arm_mse_f32.d \
./DPS/Source/StatisticsFunctions/arm_mse_f64.d \
./DPS/Source/StatisticsFunctions/arm_mse_q15.d \
./DPS/Source/StatisticsFunctions/arm_mse_q31.d \
./DPS/Source/StatisticsFunctions/arm_mse_q7.d \
./DPS/Source/StatisticsFunctions/arm_power_f16.d \
./DPS/Source/StatisticsFunctions/arm_power_f32.d \
./DPS/Source/StatisticsFunctions/arm_power_f64.d \
./DPS/Source/StatisticsFunctions/arm_power_q15.d \
./DPS/Source/StatisticsFunctions/arm_power_q31.d \
./DPS/Source/StatisticsFunctions/arm_power_q7.d \
./DPS/Source/StatisticsFunctions/arm_rms_f16.d \
./DPS/Source/StatisticsFunctions/arm_rms_f32.d \
./DPS/Source/StatisticsFunctions/arm_rms_q15.d \
./DPS/Source/StatisticsFunctions/arm_rms_q31.d \
./DPS/Source/StatisticsFunctions/arm_std_f16.d \
./DPS/Source/StatisticsFunctions/arm_std_f32.d \
./DPS/Source/StatisticsFunctions/arm_std_f64.d \
./DPS/Source/StatisticsFunctions/arm_std_q15.d \
./DPS/Source/StatisticsFunctions/arm_std_q31.d \
./DPS/Source/StatisticsFunctions/arm_var_f16.d \
./DPS/Source/StatisticsFunctions/arm_var_f32.d \
./DPS/Source/StatisticsFunctions/arm_var_f64.d \
./DPS/Source/StatisticsFunctions/arm_var_q15.d \
./DPS/Source/StatisticsFunctions/arm_var_q31.d 


# Each subdirectory must supply rules for building sources it contributes
DPS/Source/StatisticsFunctions/%.o DPS/Source/StatisticsFunctions/%.su DPS/Source/StatisticsFunctions/%.cyclo: ../DPS/Source/StatisticsFunctions/%.c DPS/Source/StatisticsFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Work/Micro/Project/Project/DPS/ComputeLibrary/Include" -I"C:/Work/Micro/Project/Project/DPS/Include" -I"C:/Work/Micro/Project/Project/DPS/PrivateInclude" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DPS-2f-Source-2f-StatisticsFunctions

clean-DPS-2f-Source-2f-StatisticsFunctions:
	-$(RM) ./DPS/Source/StatisticsFunctions/StatisticsFunctionsF16.cyclo ./DPS/Source/StatisticsFunctions/StatisticsFunctionsF16.d ./DPS/Source/StatisticsFunctions/StatisticsFunctionsF16.o ./DPS/Source/StatisticsFunctions/StatisticsFunctionsF16.su ./DPS/Source/StatisticsFunctions/arm_absmax_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_absmax_f16.d ./DPS/Source/StatisticsFunctions/arm_absmax_f16.o ./DPS/Source/StatisticsFunctions/arm_absmax_f16.su ./DPS/Source/StatisticsFunctions/arm_absmax_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_absmax_f32.d ./DPS/Source/StatisticsFunctions/arm_absmax_f32.o ./DPS/Source/StatisticsFunctions/arm_absmax_f32.su ./DPS/Source/StatisticsFunctions/arm_absmax_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_absmax_f64.d ./DPS/Source/StatisticsFunctions/arm_absmax_f64.o ./DPS/Source/StatisticsFunctions/arm_absmax_f64.su ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f16.d ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f16.o ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f16.su ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f32.d ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f32.o ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f32.su ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f64.d ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f64.o ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_f64.su ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q15.d ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q15.o ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q15.su ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q31.d ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q31.o ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q31.su ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q7.cyclo ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q7.d ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q7.o ./DPS/Source/StatisticsFunctions/arm_absmax_no_idx_q7.su ./DPS/Source/StatisticsFunctions/arm_absmax_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_absmax_q15.d ./DPS/Source/StatisticsFunctions/arm_absmax_q15.o ./DPS/Source/StatisticsFunctions/arm_absmax_q15.su ./DPS/Source/StatisticsFunctions/arm_absmax_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_absmax_q31.d ./DPS/Source/StatisticsFunctions/arm_absmax_q31.o ./DPS/Source/StatisticsFunctions/arm_absmax_q31.su ./DPS/Source/StatisticsFunctions/arm_absmax_q7.cyclo ./DPS/Source/StatisticsFunctions/arm_absmax_q7.d ./DPS/Source/StatisticsFunctions/arm_absmax_q7.o ./DPS/Source/StatisticsFunctions/arm_absmax_q7.su ./DPS/Source/StatisticsFunctions/arm_absmin_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_absmin_f16.d ./DPS/Source/StatisticsFunctions/arm_absmin_f16.o ./DPS/Source/StatisticsFunctions/arm_absmin_f16.su ./DPS/Source/StatisticsFunctions/arm_absmin_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_absmin_f32.d ./DPS/Source/StatisticsFunctions/arm_absmin_f32.o ./DPS/Source/StatisticsFunctions/arm_absmin_f32.su ./DPS/Source/StatisticsFunctions/arm_absmin_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_absmin_f64.d ./DPS/Source/StatisticsFunctions/arm_absmin_f64.o ./DPS/Source/StatisticsFunctions/arm_absmin_f64.su ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f16.d ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f16.o ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f16.su ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f32.d ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f32.o ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f32.su ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f64.d ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f64.o ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_f64.su ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q15.d ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q15.o ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q15.su ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q31.d ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q31.o ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q31.su ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q7.cyclo ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q7.d ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q7.o ./DPS/Source/StatisticsFunctions/arm_absmin_no_idx_q7.su ./DPS/Source/StatisticsFunctions/arm_absmin_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_absmin_q15.d ./DPS/Source/StatisticsFunctions/arm_absmin_q15.o ./DPS/Source/StatisticsFunctions/arm_absmin_q15.su ./DPS/Source/StatisticsFunctions/arm_absmin_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_absmin_q31.d ./DPS/Source/StatisticsFunctions/arm_absmin_q31.o ./DPS/Source/StatisticsFunctions/arm_absmin_q31.su ./DPS/Source/StatisticsFunctions/arm_absmin_q7.cyclo ./DPS/Source/StatisticsFunctions/arm_absmin_q7.d ./DPS/Source/StatisticsFunctions/arm_absmin_q7.o ./DPS/Source/StatisticsFunctions/arm_absmin_q7.su ./DPS/Source/StatisticsFunctions/arm_accumulate_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_accumulate_f16.d ./DPS/Source/StatisticsFunctions/arm_accumulate_f16.o ./DPS/Source/StatisticsFunctions/arm_accumulate_f16.su ./DPS/Source/StatisticsFunctions/arm_accumulate_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_accumulate_f32.d ./DPS/Source/StatisticsFunctions/arm_accumulate_f32.o ./DPS/Source/StatisticsFunctions/arm_accumulate_f32.su ./DPS/Source/StatisticsFunctions/arm_accumulate_f64.cyclo
	-$(RM) ./DPS/Source/StatisticsFunctions/arm_accumulate_f64.d ./DPS/Source/StatisticsFunctions/arm_accumulate_f64.o ./DPS/Source/StatisticsFunctions/arm_accumulate_f64.su ./DPS/Source/StatisticsFunctions/arm_entropy_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_entropy_f16.d ./DPS/Source/StatisticsFunctions/arm_entropy_f16.o ./DPS/Source/StatisticsFunctions/arm_entropy_f16.su ./DPS/Source/StatisticsFunctions/arm_entropy_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_entropy_f32.d ./DPS/Source/StatisticsFunctions/arm_entropy_f32.o ./DPS/Source/StatisticsFunctions/arm_entropy_f32.su ./DPS/Source/StatisticsFunctions/arm_entropy_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_entropy_f64.d ./DPS/Source/StatisticsFunctions/arm_entropy_f64.o ./DPS/Source/StatisticsFunctions/arm_entropy_f64.su ./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f16.d ./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f16.o ./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f16.su ./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f32.d ./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f32.o ./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f32.su ./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f64.d ./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f64.o ./DPS/Source/StatisticsFunctions/arm_kullback_leibler_f64.su ./DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f16.d ./DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f16.o ./DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f16.su ./DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f32.d ./DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f32.o ./DPS/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f32.su ./DPS/Source/StatisticsFunctions/arm_logsumexp_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_logsumexp_f16.d ./DPS/Source/StatisticsFunctions/arm_logsumexp_f16.o ./DPS/Source/StatisticsFunctions/arm_logsumexp_f16.su ./DPS/Source/StatisticsFunctions/arm_logsumexp_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_logsumexp_f32.d ./DPS/Source/StatisticsFunctions/arm_logsumexp_f32.o ./DPS/Source/StatisticsFunctions/arm_logsumexp_f32.su ./DPS/Source/StatisticsFunctions/arm_max_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_max_f16.d ./DPS/Source/StatisticsFunctions/arm_max_f16.o ./DPS/Source/StatisticsFunctions/arm_max_f16.su ./DPS/Source/StatisticsFunctions/arm_max_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_max_f32.d ./DPS/Source/StatisticsFunctions/arm_max_f32.o ./DPS/Source/StatisticsFunctions/arm_max_f32.su ./DPS/Source/StatisticsFunctions/arm_max_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_max_f64.d ./DPS/Source/StatisticsFunctions/arm_max_f64.o ./DPS/Source/StatisticsFunctions/arm_max_f64.su ./DPS/Source/StatisticsFunctions/arm_max_no_idx_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_max_no_idx_f16.d ./DPS/Source/StatisticsFunctions/arm_max_no_idx_f16.o ./DPS/Source/StatisticsFunctions/arm_max_no_idx_f16.su ./DPS/Source/StatisticsFunctions/arm_max_no_idx_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_max_no_idx_f32.d ./DPS/Source/StatisticsFunctions/arm_max_no_idx_f32.o ./DPS/Source/StatisticsFunctions/arm_max_no_idx_f32.su ./DPS/Source/StatisticsFunctions/arm_max_no_idx_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_max_no_idx_f64.d ./DPS/Source/StatisticsFunctions/arm_max_no_idx_f64.o ./DPS/Source/StatisticsFunctions/arm_max_no_idx_f64.su ./DPS/Source/StatisticsFunctions/arm_max_no_idx_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_max_no_idx_q15.d ./DPS/Source/StatisticsFunctions/arm_max_no_idx_q15.o ./DPS/Source/StatisticsFunctions/arm_max_no_idx_q15.su ./DPS/Source/StatisticsFunctions/arm_max_no_idx_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_max_no_idx_q31.d ./DPS/Source/StatisticsFunctions/arm_max_no_idx_q31.o ./DPS/Source/StatisticsFunctions/arm_max_no_idx_q31.su ./DPS/Source/StatisticsFunctions/arm_max_no_idx_q7.cyclo ./DPS/Source/StatisticsFunctions/arm_max_no_idx_q7.d ./DPS/Source/StatisticsFunctions/arm_max_no_idx_q7.o ./DPS/Source/StatisticsFunctions/arm_max_no_idx_q7.su ./DPS/Source/StatisticsFunctions/arm_max_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_max_q15.d ./DPS/Source/StatisticsFunctions/arm_max_q15.o ./DPS/Source/StatisticsFunctions/arm_max_q15.su ./DPS/Source/StatisticsFunctions/arm_max_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_max_q31.d ./DPS/Source/StatisticsFunctions/arm_max_q31.o ./DPS/Source/StatisticsFunctions/arm_max_q31.su ./DPS/Source/StatisticsFunctions/arm_max_q7.cyclo ./DPS/Source/StatisticsFunctions/arm_max_q7.d ./DPS/Source/StatisticsFunctions/arm_max_q7.o ./DPS/Source/StatisticsFunctions/arm_max_q7.su ./DPS/Source/StatisticsFunctions/arm_mean_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_mean_f16.d ./DPS/Source/StatisticsFunctions/arm_mean_f16.o ./DPS/Source/StatisticsFunctions/arm_mean_f16.su ./DPS/Source/StatisticsFunctions/arm_mean_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_mean_f32.d ./DPS/Source/StatisticsFunctions/arm_mean_f32.o ./DPS/Source/StatisticsFunctions/arm_mean_f32.su ./DPS/Source/StatisticsFunctions/arm_mean_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_mean_f64.d ./DPS/Source/StatisticsFunctions/arm_mean_f64.o ./DPS/Source/StatisticsFunctions/arm_mean_f64.su ./DPS/Source/StatisticsFunctions/arm_mean_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_mean_q15.d ./DPS/Source/StatisticsFunctions/arm_mean_q15.o ./DPS/Source/StatisticsFunctions/arm_mean_q15.su ./DPS/Source/StatisticsFunctions/arm_mean_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_mean_q31.d ./DPS/Source/StatisticsFunctions/arm_mean_q31.o ./DPS/Source/StatisticsFunctions/arm_mean_q31.su ./DPS/Source/StatisticsFunctions/arm_mean_q7.cyclo
	-$(RM) ./DPS/Source/StatisticsFunctions/arm_mean_q7.d ./DPS/Source/StatisticsFunctions/arm_mean_q7.o ./DPS/Source/StatisticsFunctions/arm_mean_q7.su ./DPS/Source/StatisticsFunctions/arm_min_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_min_f16.d ./DPS/Source/StatisticsFunctions/arm_min_f16.o ./DPS/Source/StatisticsFunctions/arm_min_f16.su ./DPS/Source/StatisticsFunctions/arm_min_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_min_f32.d ./DPS/Source/StatisticsFunctions/arm_min_f32.o ./DPS/Source/StatisticsFunctions/arm_min_f32.su ./DPS/Source/StatisticsFunctions/arm_min_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_min_f64.d ./DPS/Source/StatisticsFunctions/arm_min_f64.o ./DPS/Source/StatisticsFunctions/arm_min_f64.su ./DPS/Source/StatisticsFunctions/arm_min_no_idx_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_min_no_idx_f16.d ./DPS/Source/StatisticsFunctions/arm_min_no_idx_f16.o ./DPS/Source/StatisticsFunctions/arm_min_no_idx_f16.su ./DPS/Source/StatisticsFunctions/arm_min_no_idx_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_min_no_idx_f32.d ./DPS/Source/StatisticsFunctions/arm_min_no_idx_f32.o ./DPS/Source/StatisticsFunctions/arm_min_no_idx_f32.su ./DPS/Source/StatisticsFunctions/arm_min_no_idx_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_min_no_idx_f64.d ./DPS/Source/StatisticsFunctions/arm_min_no_idx_f64.o ./DPS/Source/StatisticsFunctions/arm_min_no_idx_f64.su ./DPS/Source/StatisticsFunctions/arm_min_no_idx_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_min_no_idx_q15.d ./DPS/Source/StatisticsFunctions/arm_min_no_idx_q15.o ./DPS/Source/StatisticsFunctions/arm_min_no_idx_q15.su ./DPS/Source/StatisticsFunctions/arm_min_no_idx_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_min_no_idx_q31.d ./DPS/Source/StatisticsFunctions/arm_min_no_idx_q31.o ./DPS/Source/StatisticsFunctions/arm_min_no_idx_q31.su ./DPS/Source/StatisticsFunctions/arm_min_no_idx_q7.cyclo ./DPS/Source/StatisticsFunctions/arm_min_no_idx_q7.d ./DPS/Source/StatisticsFunctions/arm_min_no_idx_q7.o ./DPS/Source/StatisticsFunctions/arm_min_no_idx_q7.su ./DPS/Source/StatisticsFunctions/arm_min_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_min_q15.d ./DPS/Source/StatisticsFunctions/arm_min_q15.o ./DPS/Source/StatisticsFunctions/arm_min_q15.su ./DPS/Source/StatisticsFunctions/arm_min_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_min_q31.d ./DPS/Source/StatisticsFunctions/arm_min_q31.o ./DPS/Source/StatisticsFunctions/arm_min_q31.su ./DPS/Source/StatisticsFunctions/arm_min_q7.cyclo ./DPS/Source/StatisticsFunctions/arm_min_q7.d ./DPS/Source/StatisticsFunctions/arm_min_q7.o ./DPS/Source/StatisticsFunctions/arm_min_q7.su ./DPS/Source/StatisticsFunctions/arm_mse_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_mse_f16.d ./DPS/Source/StatisticsFunctions/arm_mse_f16.o ./DPS/Source/StatisticsFunctions/arm_mse_f16.su ./DPS/Source/StatisticsFunctions/arm_mse_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_mse_f32.d ./DPS/Source/StatisticsFunctions/arm_mse_f32.o ./DPS/Source/StatisticsFunctions/arm_mse_f32.su ./DPS/Source/StatisticsFunctions/arm_mse_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_mse_f64.d ./DPS/Source/StatisticsFunctions/arm_mse_f64.o ./DPS/Source/StatisticsFunctions/arm_mse_f64.su ./DPS/Source/StatisticsFunctions/arm_mse_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_mse_q15.d ./DPS/Source/StatisticsFunctions/arm_mse_q15.o ./DPS/Source/StatisticsFunctions/arm_mse_q15.su ./DPS/Source/StatisticsFunctions/arm_mse_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_mse_q31.d ./DPS/Source/StatisticsFunctions/arm_mse_q31.o ./DPS/Source/StatisticsFunctions/arm_mse_q31.su ./DPS/Source/StatisticsFunctions/arm_mse_q7.cyclo ./DPS/Source/StatisticsFunctions/arm_mse_q7.d ./DPS/Source/StatisticsFunctions/arm_mse_q7.o ./DPS/Source/StatisticsFunctions/arm_mse_q7.su ./DPS/Source/StatisticsFunctions/arm_power_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_power_f16.d ./DPS/Source/StatisticsFunctions/arm_power_f16.o ./DPS/Source/StatisticsFunctions/arm_power_f16.su ./DPS/Source/StatisticsFunctions/arm_power_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_power_f32.d ./DPS/Source/StatisticsFunctions/arm_power_f32.o ./DPS/Source/StatisticsFunctions/arm_power_f32.su ./DPS/Source/StatisticsFunctions/arm_power_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_power_f64.d ./DPS/Source/StatisticsFunctions/arm_power_f64.o ./DPS/Source/StatisticsFunctions/arm_power_f64.su ./DPS/Source/StatisticsFunctions/arm_power_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_power_q15.d ./DPS/Source/StatisticsFunctions/arm_power_q15.o ./DPS/Source/StatisticsFunctions/arm_power_q15.su ./DPS/Source/StatisticsFunctions/arm_power_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_power_q31.d ./DPS/Source/StatisticsFunctions/arm_power_q31.o ./DPS/Source/StatisticsFunctions/arm_power_q31.su ./DPS/Source/StatisticsFunctions/arm_power_q7.cyclo ./DPS/Source/StatisticsFunctions/arm_power_q7.d ./DPS/Source/StatisticsFunctions/arm_power_q7.o ./DPS/Source/StatisticsFunctions/arm_power_q7.su ./DPS/Source/StatisticsFunctions/arm_rms_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_rms_f16.d ./DPS/Source/StatisticsFunctions/arm_rms_f16.o ./DPS/Source/StatisticsFunctions/arm_rms_f16.su ./DPS/Source/StatisticsFunctions/arm_rms_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_rms_f32.d ./DPS/Source/StatisticsFunctions/arm_rms_f32.o ./DPS/Source/StatisticsFunctions/arm_rms_f32.su ./DPS/Source/StatisticsFunctions/arm_rms_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_rms_q15.d ./DPS/Source/StatisticsFunctions/arm_rms_q15.o ./DPS/Source/StatisticsFunctions/arm_rms_q15.su ./DPS/Source/StatisticsFunctions/arm_rms_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_rms_q31.d ./DPS/Source/StatisticsFunctions/arm_rms_q31.o ./DPS/Source/StatisticsFunctions/arm_rms_q31.su ./DPS/Source/StatisticsFunctions/arm_std_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_std_f16.d ./DPS/Source/StatisticsFunctions/arm_std_f16.o ./DPS/Source/StatisticsFunctions/arm_std_f16.su ./DPS/Source/StatisticsFunctions/arm_std_f32.cyclo
	-$(RM) ./DPS/Source/StatisticsFunctions/arm_std_f32.d ./DPS/Source/StatisticsFunctions/arm_std_f32.o ./DPS/Source/StatisticsFunctions/arm_std_f32.su ./DPS/Source/StatisticsFunctions/arm_std_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_std_f64.d ./DPS/Source/StatisticsFunctions/arm_std_f64.o ./DPS/Source/StatisticsFunctions/arm_std_f64.su ./DPS/Source/StatisticsFunctions/arm_std_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_std_q15.d ./DPS/Source/StatisticsFunctions/arm_std_q15.o ./DPS/Source/StatisticsFunctions/arm_std_q15.su ./DPS/Source/StatisticsFunctions/arm_std_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_std_q31.d ./DPS/Source/StatisticsFunctions/arm_std_q31.o ./DPS/Source/StatisticsFunctions/arm_std_q31.su ./DPS/Source/StatisticsFunctions/arm_var_f16.cyclo ./DPS/Source/StatisticsFunctions/arm_var_f16.d ./DPS/Source/StatisticsFunctions/arm_var_f16.o ./DPS/Source/StatisticsFunctions/arm_var_f16.su ./DPS/Source/StatisticsFunctions/arm_var_f32.cyclo ./DPS/Source/StatisticsFunctions/arm_var_f32.d ./DPS/Source/StatisticsFunctions/arm_var_f32.o ./DPS/Source/StatisticsFunctions/arm_var_f32.su ./DPS/Source/StatisticsFunctions/arm_var_f64.cyclo ./DPS/Source/StatisticsFunctions/arm_var_f64.d ./DPS/Source/StatisticsFunctions/arm_var_f64.o ./DPS/Source/StatisticsFunctions/arm_var_f64.su ./DPS/Source/StatisticsFunctions/arm_var_q15.cyclo ./DPS/Source/StatisticsFunctions/arm_var_q15.d ./DPS/Source/StatisticsFunctions/arm_var_q15.o ./DPS/Source/StatisticsFunctions/arm_var_q15.su ./DPS/Source/StatisticsFunctions/arm_var_q31.cyclo ./DPS/Source/StatisticsFunctions/arm_var_q31.d ./DPS/Source/StatisticsFunctions/arm_var_q31.o ./DPS/Source/StatisticsFunctions/arm_var_q31.su

.PHONY: clean-DPS-2f-Source-2f-StatisticsFunctions

