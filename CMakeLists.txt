CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET (CMAKE_VERBOSE_MAKEFILE OFF)

PROJECT (free)
ENABLE_LANGUAGE (ASM-ATT)

INCLUDE_DIRECTORIES("src/")

SET (FREE_RTOS_ROOT "deps/FreeRTOS-Kernel")
INCLUDE_DIRECTORIES("${FREE_RTOS_ROOT}/include")
INCLUDE_DIRECTORIES("${FREE_RTOS_ROOT}/portable/GCC/ARM_CM4F")

INCLUDE_DIRECTORIES("deps/etl/include/")
include_directories("deps/GSL/include")
include_directories("deps/fmt/include")

add_definitions("-DUSE_FULL_LL_DRIVER=1")

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf
        "src/app_main.cc"
        "src/main.cc"
        # "src/tickless.c"
        "src/lptimTick.c"
        "src/stm32l4xx_it.c"
        "src/system_stm32l4xx.c"
        "src/libc-compat.c"
        "src/hal-compat.c"
        "src/freertos-hooks.c"
        "src/itoa.c"

        "src/logging.cc"
        "src/uart.cc"        

        "${FREE_RTOS_ROOT}/list.c"
        "${FREE_RTOS_ROOT}/queue.c"
        "${FREE_RTOS_ROOT}/tasks.c"
        "${FREE_RTOS_ROOT}/timers.c"
        "${FREE_RTOS_ROOT}/portable/GCC/ARM_CM4F/port.c"
        "${FREE_RTOS_ROOT}/portable/GCC/ARM_CM4F/portmacro.h"
        "${FREE_RTOS_ROOT}/portable/MemMang/heap_4.c"
        "${STARTUP_CODE}"

        # "deps/fmt/src/format.cc"

        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_comp.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cryp.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cryp_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dac.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dac_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dcmi.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma2d.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dsi.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_firewall.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gfxmmu.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_hash.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_hash_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_hcd.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_irda.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_iwdg.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lcd.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lptim.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_ltdc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_ltdc_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_nand.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_nor.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_ospi.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rng.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_smartcard.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_smartcard_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_smbus.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sram.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_swpmi.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tsc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart_ex.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_wwdg.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_adc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_comp.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_crc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_crs.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dac.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma2d.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_fmc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_i2c.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_lptim.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_lpuart.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_opamp.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_pwr.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rcc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rng.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rtc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_spi.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_swpmi.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usart.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c"
        "deps/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c"
)

FIND_PROGRAM (OPENOCD openocd)
ADD_CUSTOM_TARGET("upload" DEPENDS ${CMAKE_PROJECT_NAME}.elf COMMAND ${OPENOCD} -f /home/iwasz/local/share/openocd/scripts/interface/stlink.cfg -f /home/iwasz/local/share/openocd/scripts/target/stm32l4x.cfg -c 'program ${CMAKE_PROJECT_NAME}.elf verify reset exit')
ADD_CUSTOM_TARGET("debug" DEPENDS ${CMAKE_PROJECT_NAME}.elf COMMAND ${OPENOCD} -f /home/iwasz/local/share/openocd/scripts/interface/stlink.cfg -f /home/iwasz/local/share/openocd/scripts/target/stm32l4x.cfg)

ADD_CUSTOM_TARGET(${CMAKE_PROJECT_NAME}.bin ALL DEPENDS ${CMAKE_PROJECT_NAME}.elf COMMAND ${CMAKE_OBJCOPY} -Obinary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin)
