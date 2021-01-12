################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/HTS221_SPI/HTS221_SPI.c 

OBJS += \
./Drivers/HTS221_SPI/HTS221_SPI.o 

C_DEPS += \
./Drivers/HTS221_SPI/HTS221_SPI.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/HTS221_SPI/HTS221_SPI.o: ../Drivers/HTS221_SPI/HTS221_SPI.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Core/Inc -I"../Drivers/SHTC3" -I"../Drivers/DIGIPOT_MCP4222" -I"../Drivers/LIS2DH" -I"../Drivers/HTS221_SPI" -I"../Drivers/STM32F3xx_HAL_Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/HTS221_SPI/HTS221_SPI.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

