SET (DEVICE "STM32L476xx")
SET (CUBE_ROOT "${CMAKE_CURRENT_LIST_DIR}/deps")
SET (STARTUP_CODE "${CMAKE_CURRENT_LIST_DIR}/startup_stm32l476xx.s")
#SET (LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/STM32L475VGTX_FLASH.ld")
SET (LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/STM32L476RG_FLASH_SRAM2.ld")

SET (CMAKE_SYSTEM_NAME Generic)
SET (CMAKE_SYSTEM_PROCESSOR arm)

SET(CMAKE_C_FLAGS "-std=c11 -fdata-sections -ffunction-sections -Wall -Werror=return-type" CACHE INTERNAL "c compiler flags")
SET(CMAKE_CXX_FLAGS "-std=c++17 -fno-rtti -fno-exceptions -Wall -fdata-sections -ffunction-sections -Wall -Werror=return-type" CACHE INTERNAL "cxx compiler flags")
#SET(CMAKE_CXX_FLAGS "-mcpu=cortex-m4 -std=c++17 -fno-exceptions -Wall -fdata-sections -ffunction-sections -Wall -Werror=return-type" CACHE INTERNAL "cxx compiler flags")
SET (CMAKE_EXE_LINKER_FLAGS "-T ${LINKER_SCRIPT} -specs=nosys.specs -Wl,--gc-sections" CACHE INTERNAL "exe link flags")
#SET (CMAKE_EXE_LINKER_FLAGS "-T ${LINKER_SCRIPT} -specs=rdimon.specs -Wl,--gc-sections" CACHE INTERNAL "exe link flags")

INCLUDE_DIRECTORIES(${SUPPORT_FILES})
LINK_DIRECTORIES(${SUPPORT_FILES})
ADD_DEFINITIONS(-D${DEVICE})

INCLUDE_DIRECTORIES("${CUBE_ROOT}/STM32L4xx_HAL_Driver/Inc/")
INCLUDE_DIRECTORIES("${CUBE_ROOT}/CMSIS/Device/ST/STM32L4xx/Include/")
INCLUDE_DIRECTORIES("${CUBE_ROOT}/CMSIS/Include/")
