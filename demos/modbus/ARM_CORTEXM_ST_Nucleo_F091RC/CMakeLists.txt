# Create interface library for the Nucleo-F091RC board specific sources.
add_library(nucleo_f091rc_modbus INTERFACE)

target_sources(nucleo_f091rc_modbus INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/bsp.c"
    "${CMAKE_CURRENT_LIST_DIR}/tbxmb_port.c"
    "${CMAKE_CURRENT_LIST_DIR}/main.c"
    "${CMAKE_CURRENT_LIST_DIR}/system_stm32f0xx.c"
)

target_include_directories(nucleo_f091rc_modbus INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_compile_definitions(nucleo_f091rc_modbus INTERFACE
    STM32F091xC
)

target_link_libraries(nucleo_f091rc_modbus INTERFACE
  microtbx-cortexm
  cube_f0_ll
)

# Include the compiler specifics which also contains the actual demo projects.
add_subdirectory(GCC)
