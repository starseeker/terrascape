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

