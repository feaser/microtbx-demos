# Create interface library for the Nucleo-F429ZI board specific sources.
add_library(nucleo_f429zi_modbus INTERFACE)

target_sources(nucleo_f429zi_modbus INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/bsp.c"
    "${CMAKE_CURRENT_LIST_DIR}/tbxmb_port.c"
    "${CMAKE_CURRENT_LIST_DIR}/main.c"
    "${CMAKE_CURRENT_LIST_DIR}/system_stm32f4xx.c"
)

target_include_directories(nucleo_f429zi_modbus INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_compile_definitions(nucleo_f429zi_modbus INTERFACE
    STM32F429xx
)

target_link_libraries(nucleo_f429zi_modbus INTERFACE
  microtbx-cortexm
  cube_f4_hal
)

# Include the compiler specifics which also contains the actual demo projects.
add_subdirectory(GCC)
