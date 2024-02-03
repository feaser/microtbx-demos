# Create interface library for the Nucleo-F429ZI GCC compiler specific sources.
add_library(nucleo_f429zi_gcc_modbus INTERFACE)

target_sources(nucleo_f429zi_gcc_modbus INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/startup_stm32f429zitx.s"
    "${CMAKE_CURRENT_LIST_DIR}/syscalls.c"
    "${CMAKE_CURRENT_LIST_DIR}/sysmem.c"
)

target_include_directories(nucleo_f429zi_gcc_modbus INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(nucleo_f429zi_gcc_modbus INTERFACE
  nucleo_f429zi_modbus
)

# Include the actual demo projects.
add_subdirectory(serverrtu)
add_subdirectory(serverrtu_cplusplus)
add_subdirectory(serverrtu_freertos)
add_subdirectory(clientrtu)
add_subdirectory(clientrtu_cplusplus)
#add_subdirectory(clientrtu_freertos)

