# Import library dependencies.
include("${CMAKE_SOURCE_DIR}/.cmake/CubeF0.cmake")

# Create interface library for the Nucleo-F091RC board specific sources.
add_library(nucleo_f091rc INTERFACE)

target_sources(nucleo_f091rc INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/bsp.c"
    "${CMAKE_CURRENT_LIST_DIR}/main.c"
    "${CMAKE_CURRENT_LIST_DIR}/system_stm32f0xx.c"
)

target_include_directories(nucleo_f091rc INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_compile_definitions(nucleo_f091rc INTERFACE
    STM32F091xC
)

target_link_libraries(nucleo_f091rc INTERFACE
  microtbx-cortexm
  cube_f0
)

# Include the compiler specifics which also contains the actual demo projects.
add_subdirectory(GCC)
