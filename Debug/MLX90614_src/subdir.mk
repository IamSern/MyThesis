################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MLX90614_src/MLX90614.c 

OBJS += \
./MLX90614_src/MLX90614.o 

C_DEPS += \
./MLX90614_src/MLX90614.d 


# Each subdirectory must supply rules for building sources it contributes
MLX90614_src/MLX90614.o: ../MLX90614_src/MLX90614.c MLX90614_src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"D:/_Project/LVTN/RTOS_thesis/Source_gLCD" -I"D:/_Project/LVTN/RTOS_thesis/UI_gLCD" -I"D:/_Project/LVTN/RTOS_thesis/MLX90614_src" -I"D:/_Project/LVTN/RTOS_thesis/RC522_src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MLX90614_src/MLX90614.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

