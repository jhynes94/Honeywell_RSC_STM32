################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DrvSPI/DrvSPI.c 

OBJS += \
./Core/Src/DrvSPI/DrvSPI.o 

C_DEPS += \
./Core/Src/DrvSPI/DrvSPI.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/DrvSPI/DrvSPI.o: ../Core/Src/DrvSPI/DrvSPI.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L031xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/DrvSPI/DrvSPI.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

