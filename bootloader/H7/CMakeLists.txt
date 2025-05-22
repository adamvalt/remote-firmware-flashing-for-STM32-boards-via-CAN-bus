# CMakeLists.txt for dual-core STM32H7xx MCUs and CLion IDE
#
# DISCLAIMER: Experimental version, based on undocumented assumptions how STM32CubeMX works
# DISCLAIMER: THIS FILE IS PROVIDED UNDER "The Unlicense" LICENSE ON AN "AS IS" BASIS, WITHOUT WARRANTY OF ANY KIND
#
# Requirements:
#     Toolchain binaries have to be in system path
#     STM32CubeMX field "Project Manager | Code Generator | Target IDE" must be set to "STM32CubeIDE"
#
# Tested under environment:
#     Windows 10
#     GCC toolchain for ARM Cortex-M V2019q4
#     STM32CubeMX V6.1.0
#     STM32CubeH7 Firmware Package V1.8.0
#     CLion 2020.3
#     NUCLEO-H745ZI-Q MCU board
#
# How To Use:
#     1. Set up CLion according to https://www.jetbrains.com/help/clion/2020.3/embedded-overview.html#build-system
#     2. Create a project using STM32CubeMX for dual-core STM32H7xx MCU, set "Target IDE" as STM32CubeIDE, and click GENERATE CODE
#     3. Place this file into the project root folder
#     4. Set relevant project name as 'project' clause 1st argument
#     5. Open the project root folder as CLion project "File | Open..."
#
# Written by ilia.motornyi[%]jetbrains.com
#
cmake_minimum_required(VERSION 3.16)

# Cross compilers and tools
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Project settings.
project(bootloader_h7 C CXX ASM)
# Common compile settings
#    Language standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#    Generic compiler settings
add_compile_options(-mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_link_options(-mthumb -mthumb-interwork -specs=nosys.specs)
add_link_options(-Wl,-gc-sections,--print-memory-usage)

#    Build types
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_definitions(DEBUG)
    add_compile_options(-Og -g3)
endif ()

#    Enable hardware FPU
add_definitions("-DCO_CONFIG_SLAVE")
add_definitions("-DSTM32H745xx")

add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#    Uncomment to mitigate c++17 register variable warnings
#add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-register>)

# End of common part

# Detect linker scripts. Priorities: *_USER.ld, if not found  *_FLASH.ld, if not found first *.ld
foreach (KRNL CM4 CM7)
    file(GLOB LINKER_SCRIPTS "${KRNL}/*_USER.ld")
    if (NOT LINKER_SCRIPTS)
        file(GLOB LINKER_SCRIPTS "${KRNL}/*_FLASH.ld")
        if (NOT LINKER_SCRIPTS)
            file(GLOB LINKER_SCRIPTS "${KRNL}/*.ld")
        endif ()
    endif ()
    #file(GLOB LINKER_SCRIPTS "${KRNL}/*_RAM.ld")
    list(GET LINKER_SCRIPTS 0 LINKER_SCRIPT_${KRNL})
    message(STATUS "Detected Linker Script for ${KRNL}: ${LINKER_SCRIPT_${KRNL}}")
endforeach ()

# Kernel-specific sources
#    Cortex-M4
file(GLOB_RECURSE CM4_SRC "CM4/*.*" "Common/*.*")
add_executable(${PROJECT_NAME}_CM4.elf ${LINKER_SCRIPT_CM4} ${CM4_SRC} )

#    Cortex-M7
file(GLOB_RECURSE CM7_SRC "CM7/*.*" "Common/*.*" "Drivers/*.*" "CM7/Core/*.*")
include_directories(Drivers/canopenlibrary Drivers/canopenlibrary/CANopenNode CM7/Core)
add_executable(${PROJECT_NAME}_CM7.elf ${LINKER_SCRIPT_CM7} ${CM7_SRC})

# Kernel-specific build settings
target_compile_options(${PROJECT_NAME}_CM4.elf PRIVATE -mcpu=cortex-m4)
target_compile_options(${PROJECT_NAME}_CM7.elf PRIVATE -mcpu=cortex-m7)

target_link_options(${PROJECT_NAME}_CM4.elf PRIVATE -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}_CM4.map -mcpu=cortex-m4 -T ${LINKER_SCRIPT_CM4})
target_link_options(${PROJECT_NAME}_CM7.elf PRIVATE -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}_CM7.map -mcpu=cortex-m7 -T ${LINKER_SCRIPT_CM7})

# Custom commands to build .hex files
foreach (KRNL CM4 CM7)
    set(HEX_NAME ${PROJECT_NAME}_${KRNL}.hex)
    set(HEX_FILE ${PROJECT_BINARY_DIR}/${HEX_NAME})
    add_custom_command(TARGET ${PROJECT_NAME}_${KRNL}.elf POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}_${KRNL}.elf> ${HEX_FILE}
            BYPRODUCTS ${HEX_FILE}
            COMMENT "Building ${HEX_NAME}")
endforeach ()

# Read kernel-specific header paths, defines, and sources from ".mxproject"
file(STRINGS .mxproject LINES)
foreach (LINE ${LINES})
    if (LINE MATCHES "\\[Cortex(M4|M7):(PreviousUsedCubeIDEFiles|PreviousLibFiles)\\]") #Detect relevant group
        set(CUBE_PRJ_GROUP "C${CMAKE_MATCH_1}")
    elseif (LINE MATCHES "^\\[.*\\]$") #Detect non-relevant groups
        unset(CUBE_PRJ_GROUP)
    elseif (CUBE_PRJ_GROUP)
        if (LINE MATCHES "^\\s*CDefines=\\s*(.*)") #Detect defines
            target_compile_definitions(${PROJECT_NAME}_${CUBE_PRJ_GROUP}.elf PRIVATE ${CMAKE_MATCH_1})
        elseif (LINE MATCHES "^\\s*HeaderPath=\\s*(.*)\\s*") #Detect header paths
            string(REGEX MATCHALL "[^;]+" INCL_LIST "${CMAKE_MATCH_1}")
            target_include_directories(${PROJECT_NAME}_${CUBE_PRJ_GROUP}.elf PRIVATE ${INCL_LIST})
        elseif (LINE MATCHES "^\\s*LibFiles=\\s*(.*)\\s*") #Detect library sources
            string(REGEX MATCHALL "[^;]+" SRC_LIST "${CMAKE_MATCH_1}")
            foreach (SRC_FILE ${SRC_LIST})
                if (EXISTS "${CMAKE_SOURCE_DIR}/${SRC_FILE}")
                    target_sources(${PROJECT_NAME}_${CUBE_PRJ_GROUP}.elf PRIVATE ${SRC_FILE})
                endif ()
            endforeach ()
        endif ()
    endif ()
endforeach ()
