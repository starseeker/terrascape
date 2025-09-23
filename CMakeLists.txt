cmake_minimum_required(VERSION 3.12)
project(TerraScape VERSION 0.9)

# Set C standard to C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set C++ standard to C++17 (for the old demo if needed)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable CTest framework
enable_testing()

# Since TerraScape is now header-only C99, we only need to build test programs
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Create the basic C interface test
add_executable(test_terrascape test_terrascape.c)
target_link_libraries(test_terrascape m) # Link math library for sqrt, sin, cos
set_target_properties(test_terrascape PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Create the BRL-CAD integration demo
add_executable(brlcad_integration_demo brlcad_integration_demo.c)
target_link_libraries(brlcad_integration_demo m) # Link math library
set_target_properties(brlcad_integration_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Old demo (keep for comparison, but requires C++ now)
# This would need to be updated to use the C API
# add_executable(terrascape_demo main.cpp)
# target_include_directories(terrascape_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
