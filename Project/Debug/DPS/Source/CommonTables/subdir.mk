################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DPS/Source/CommonTables/CommonTablesF16.c \
../DPS/Source/CommonTables/arm_common_tables.c \
../DPS/Source/CommonTables/arm_common_tables_f16.c \
../DPS/Source/CommonTables/arm_const_structs.c \
../DPS/Source/CommonTables/arm_const_structs_f16.c \
../DPS/Source/CommonTables/arm_mve_tables.c \
../DPS/Source/CommonTables/arm_mve_tables_f16.c 

OBJS += \
./DPS/Source/CommonTables/CommonTablesF16.o \
./DPS/Source/CommonTables/arm_common_tables.o \
./DPS/Source/CommonTables/arm_common_tables_f16.o \
./DPS/Source/CommonTables/arm_const_structs.o \
./DPS/Source/CommonTables/arm_const_structs_f16.o \
./DPS/Source/CommonTables/arm_mve_tables.o \
./DPS/Source/CommonTables/arm_mve_tables_f16.o 

C_DEPS += \
./DPS/Source/CommonTables/CommonTablesF16.d \
./DPS/Source/CommonTables/arm_common_tables.d \
./DPS/Source/CommonTables/arm_common_tables_f16.d \
./DPS/Source/CommonTables/arm_const_structs.d \
./DPS/Source/CommonTables/arm_const_structs_f16.d \
./DPS/Source/CommonTables/arm_mve_tables.d \
./DPS/Source/CommonTables/arm_mve_tables_f16.d 


# Each subdirectory must supply rules for building sources it contributes
DPS/Source/CommonTables/%.o DPS/Source/CommonTables/%.su: ../DPS/Source/CommonTables/%.c DPS/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Work/Micro/Project/Project/DPS/ComputeLibrary/Include" -I"C:/Work/Micro/Project/Project/DPS/Include" -I"C:/Work/Micro/Project/Project/DPS/PrivateInclude" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DPS-2f-Source-2f-CommonTables

clean-DPS-2f-Source-2f-CommonTables:
	-$(RM) ./DPS/Source/CommonTables/CommonTablesF16.d ./DPS/Source/CommonTables/CommonTablesF16.o ./DPS/Source/CommonTables/CommonTablesF16.su ./DPS/Source/CommonTables/arm_common_tables.d ./DPS/Source/CommonTables/arm_common_tables.o ./DPS/Source/CommonTables/arm_common_tables.su ./DPS/Source/CommonTables/arm_common_tables_f16.d ./DPS/Source/CommonTables/arm_common_tables_f16.o ./DPS/Source/CommonTables/arm_common_tables_f16.su ./DPS/Source/CommonTables/arm_const_structs.d ./DPS/Source/CommonTables/arm_const_structs.o ./DPS/Source/CommonTables/arm_const_structs.su ./DPS/Source/CommonTables/arm_const_structs_f16.d ./DPS/Source/CommonTables/arm_const_structs_f16.o ./DPS/Source/CommonTables/arm_const_structs_f16.su ./DPS/Source/CommonTables/arm_mve_tables.d ./DPS/Source/CommonTables/arm_mve_tables.o ./DPS/Source/CommonTables/arm_mve_tables.su ./DPS/Source/CommonTables/arm_mve_tables_f16.d ./DPS/Source/CommonTables/arm_mve_tables_f16.o ./DPS/Source/CommonTables/arm_mve_tables_f16.su

.PHONY: clean-DPS-2f-Source-2f-CommonTables

