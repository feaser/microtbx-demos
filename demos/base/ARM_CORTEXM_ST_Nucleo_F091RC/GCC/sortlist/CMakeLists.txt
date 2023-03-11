# Configure the project name.
project(NucleoF091RC_Base_SortList)

# Configure project libraries.
set(PROJECT_LIBRARIES
    sort_list_demo
    nucleo_f091rc_gcc_base
)

# Configure project specific CPU/compiler flags and linker script.
set(PROJECT_CPU_FLAGS -mcpu=cortex-m0 -mthumb -mfloat-abi=soft)
set(PROJECT_COMPILER_FLAGS -Wall -Wextra -Wno-unused-parameter)
set(PROJECT_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../STM32F091RC_FLASH.ld")

# Set target executable.
add_executable(${PROJECT_NAME})

# Add additional compiler options.
target_compile_options(${PROJECT_NAME} PRIVATE
    ${PROJECT_CPU_FLAGS}
    ${PROJECT_COMPILER_FLAGS}
)

# Add libraries to include.
target_link_libraries(${PROJECT_NAME}
    ${PROJECT_LIBRARIES}
)

# Add additional link options.
target_link_options(${PROJECT_NAME} PRIVATE
    ${PROJECT_CPU_FLAGS}
    -T "${PROJECT_LINKER_SCRIPT}"
    --specs=nano.specs
)

# Generate a map file (optional).
arm_cortex_gcc_generate_map(${PROJECT_NAME})

# Generate an additional firmware file in the S-record format (optional).
arm_cortex_gcc_generate_extra_output(${PROJECT_NAME} .srec srec)

# Show ROM/RAM usage information at the end of the build (optional).
arm_cortex_gcc_display_size(${PROJECT_NAME})

