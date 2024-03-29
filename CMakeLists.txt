cmake_minimum_required(VERSION 3.17)

# Configure the toolchain file. Set before the project() line.
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/.cmake/arm-cortex-gcc-toolchain.cmake) 

# Configure C and C++ standards.
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Specify overall project name.
project(apps LANGUAGES C CXX ASM)

# Option to disable the generation of the MISRA Cppcheck targets. Override on the command
# line when CMake is called using the following parameter: -DMISRA_ENABLED=ON
option(MISRA_ENABLED "Configurable to enable/disable the MISRA Cppcheck target" OFF)

# Include the MicroTBX sources.
add_subdirectory(third_party/microtbx)

# Include the MicroTBX-Modbus sources.
add_subdirectory(third_party/microtbx-modbus)

# Include other third party components.
include("${CMAKE_SOURCE_DIR}/.cmake/FreeRTOS.cmake")
include("${CMAKE_SOURCE_DIR}/.cmake/CubeF0.cmake")
include("${CMAKE_SOURCE_DIR}/.cmake/CubeF4.cmake")

# Include the demos sources.
add_subdirectory(demos)


