# Create interface library for the dynamic array demo application sources.
add_library(dynamic_array_demo INTERFACE)

target_sources(dynamic_array_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/dynamicarray.c"
)

target_include_directories(dynamic_array_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(dynamic_array_demo INTERFACE
    microtbx
)

# Create interface library for the fifo buffer demo application sources.
add_library(fifo_buffer_demo INTERFACE)

target_sources(fifo_buffer_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/fifobuffer.c"
)

target_include_directories(fifo_buffer_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(fifo_buffer_demo INTERFACE
    microtbx
)

# Create interface library for the random data demo application sources.
add_library(random_data_demo INTERFACE)

target_sources(random_data_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/randomdata.c"
)

target_include_directories(random_data_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(random_data_demo INTERFACE
    microtbx
)

# Create interface library for the secure data demo application sources.
add_library(secure_data_demo INTERFACE)

target_sources(secure_data_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/securedata.c"
)

target_include_directories(secure_data_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(secure_data_demo INTERFACE
    microtbx
)

# Create interface library for the sort list demo application sources.
add_library(sort_list_demo INTERFACE)

target_sources(sort_list_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/sortlist.c"
)

target_include_directories(sort_list_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(sort_list_demo INTERFACE
    microtbx
)

# Create microtbx_MISRA for MISRA checking with cppcheck.
if(MISRA_ENABLED)
include("${CMAKE_SOURCE_DIR}/third_party/microtbx/source/misra.cmake")
endif()

# Include all supported boards.
add_subdirectory(ARM_CORTEXM_ST_Nucleo_F091RC)

