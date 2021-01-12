################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LIS2DH/lis2dh12_reg.c 

OBJS += \
./Drivers/LIS2DH/lis2dh12_reg.o 

C_DEPS += \
./Drivers/LIS2DH/lis2dh12_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LIS2DH/lis2dh12_reg.o: ../Drivers/LIS2DH/lis2dh12_reg.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Core/Inc -I"../Drivers/SHTC3" -I"../Drivers/DIGIPOT_MCP4222" -I"../Drivers/LIS2DH" -I"../Drivers/HTS221_SPI" -I"../Drivers/STM32F3xx_HAL_Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/LIS2DH/lis2dh12_reg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

