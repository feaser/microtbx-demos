# Create interface library for the server RTU demo application sources.
add_library(server_rtu_demo INTERFACE)

target_sources(server_rtu_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/serverrtu.c"
)

target_include_directories(server_rtu_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(server_rtu_demo INTERFACE
    common_demo
    microtbx
    microtbx-modbus
    microtbx-modbus-osal-superloop
)

# Create interface library for the server RTU C++ demo application sources.
add_library(server_rtu_cplusplus_demo INTERFACE)

target_sources(server_rtu_cplusplus_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/serverrtu_cplusplus.cpp"
)

target_include_directories(server_rtu_cplusplus_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(server_rtu_cplusplus_demo INTERFACE
    common_demo
    microtbx
    microtbx-extra-cpp
    microtbx-modbus
    microtbx-modbus-osal-superloop
    microtbx-modbus-extra-cpp
)

# Create interface library for the server RTU FreeRTOS demo application sources.
add_library(server_rtu_freertos_demo INTERFACE)

target_sources(server_rtu_freertos_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/serverrtu_freertos.c"
)

target_include_directories(server_rtu_freertos_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(server_rtu_freertos_demo INTERFACE
    common_demo
    microtbx
    microtbx-extra-freertos
    microtbx-modbus
    microtbx-modbus-osal-freertos
    freertos
)

# Create interface library for the client RTU demo application sources.
add_library(client_rtu_demo INTERFACE)

target_sources(client_rtu_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/clientrtu.c"
)

target_include_directories(client_rtu_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(client_rtu_demo INTERFACE
    common_demo
    microtbx
    microtbx-modbus
    microtbx-modbus-osal-superloop
)

# Create interface library for the client RTU C++ demo application sources.
add_library(client_rtu_cplusplus_demo INTERFACE)

target_sources(client_rtu_cplusplus_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/clientrtu_cplusplus.cpp"
)

target_include_directories(client_rtu_cplusplus_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(client_rtu_cplusplus_demo INTERFACE
    common_demo
    microtbx
    microtbx-extra-cpp
    microtbx-modbus
    microtbx-modbus-osal-superloop
    microtbx-modbus-extra-cpp
)

# Create interface library for the client RTU FreeRTOS demo application sources.
add_library(client_rtu_freertos_demo INTERFACE)

target_sources(client_rtu_freertos_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}/clientrtu_freertos.c"
)

target_include_directories(client_rtu_freertos_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)

target_link_libraries(client_rtu_freertos_demo INTERFACE
    common_demo
    microtbx
    microtbx-extra-freertos
    microtbx-modbus
    microtbx-modbus-osal-freertos
    freertos
)

# Create microtbx_modbus_MISRA for MISRA checking with cppcheck.
include("${CMAKE_SOURCE_DIR}/third_party/microtbx-modbus/source/misra.cmake")

# Include all supported boards and the common sources
add_subdirectory(common)
add_subdirectory(ARM_CORTEXM_ST_Nucleo_F091RC)

