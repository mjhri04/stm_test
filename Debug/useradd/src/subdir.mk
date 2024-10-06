################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../useradd/src/mpu6050.c 

OBJS += \
./useradd/src/mpu6050.o 

C_DEPS += \
./useradd/src/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
useradd/src/%.o useradd/src/%.su useradd/src/%.cyclo: ../useradd/src/%.c useradd/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/ju/STM32CubeIDE/workspace_1.16.1/test/useradd/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-useradd-2f-src

clean-useradd-2f-src:
	-$(RM) ./useradd/src/mpu6050.cyclo ./useradd/src/mpu6050.d ./useradd/src/mpu6050.o ./useradd/src/mpu6050.su

.PHONY: clean-useradd-2f-src

