cmake_minimum_required(VERSION 3.12)
project(TerraScape VERSION 0.8)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable CTest framework
enable_testing()

# Find GDAL as optional dependency
find_package(GDAL CONFIG)
if(GDAL_FOUND)
	add_definitions(-DHAVE_GDAL)
endif()

# Create the terrascape_demo executable (now header-only)
add_executable(terrascape_demo main.cpp)
target_include_directories(terrascape_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(terrascape_demo PRIVATE ${GDAL_INCLUDE_DIRS})
target_link_libraries(terrascape_demo ${GDAL_LIBRARIES})
target_compile_options(terrascape_demo PRIVATE ${GDAL_CFLAGS_OTHER})
set_target_properties(terrascape_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Create the C interface library for BRL-CAD integration
add_library(terrascape_c STATIC terrascape_c_interface.cpp)
target_include_directories(terrascape_c PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(terrascape_c PRIVATE ${GDAL_INCLUDE_DIRS})
target_link_libraries(terrascape_c ${GDAL_LIBRARIES})
target_compile_options(terrascape_c PRIVATE ${GDAL_CFLAGS_OTHER})

# Create the C interface test program
add_executable(test_c_interface test_c_interface.c)
target_link_libraries(test_c_interface terrascape_c)
set_target_properties(test_c_interface PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Create the BRL-CAD integration example
add_executable(brlcad_integration_example brlcad_integration_example.c)
target_link_libraries(brlcad_integration_example terrascape_c)
set_target_properties(brlcad_integration_example PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

