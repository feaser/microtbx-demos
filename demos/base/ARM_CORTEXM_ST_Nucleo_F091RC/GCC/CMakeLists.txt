# Create interface library for the Nucleo-F091RC GCC compiler specific sources.
add_library(nucleo_f091rc_gcc INTERFACE)

target_sources(nucleo_f091rc_gcc INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/startup_stm32f091xc.s"
    "${CMAKE_CURRENT_LIST_DIR}/syscalls.c"
)

target_include_directories(nucleo_f091rc_gcc INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(nucleo_f091rc_gcc INTERFACE
  nucleo_f091rc
)

# Include the actual demo projects.
add_subdirectory(dynamicarray)
add_subdirectory(fifobuffer)
add_subdirectory(randomdata)
add_subdirectory(securedata)
add_subdirectory(sortlist)

