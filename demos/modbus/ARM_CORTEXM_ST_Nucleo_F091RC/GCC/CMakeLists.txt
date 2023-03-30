# Create interface library for the Nucleo-F091RC GCC compiler specific sources.
add_library(nucleo_f091rc_gcc_modbus INTERFACE)

target_sources(nucleo_f091rc_gcc_modbus INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/startup_stm32f091xc.s"
    "${CMAKE_CURRENT_LIST_DIR}/syscalls.c"
)

target_include_directories(nucleo_f091rc_gcc_modbus INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(nucleo_f091rc_gcc_modbus INTERFACE
  nucleo_f091rc_modbus
)

# Include the actual demo projects.
add_subdirectory(serverrtu)
add_subdirectory(serverrtu_cplusplus)
add_subdirectory(serverrtu_freertos)
add_subdirectory(clientrtu)
add_subdirectory(clientrtu_cplusplus)
add_subdirectory(clientrtu_freertos)

