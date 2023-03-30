# Create interface library for the common demo sources.
add_library(common_demo INTERFACE)

target_include_directories(common_demo INTERFACE
  "${CMAKE_CURRENT_LIST_DIR}"
)


