cmake_minimum_required(VERSION 3.12)
project(terrascape VERSION 0.7)

# Set C++ standard to C++17 (required for bg_detria.hpp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compile definitions for legacy code compatibility
add_definitions(-DSAFETY)

# Source files for the Terra library
set(TERRA_SOURCES
    GreedyInsert.cc
    Map.cc
    Mask.cc
    Quadedge.cc
    Subdivision.cc
    Heap.cc
    greedy.cc
    output.cc
)

# Header files
set(TERRA_HEADERS
    terra.h
    GreedyInsert.h
    Map.h
    Mask.h
    Quadedge.h
    Subdivision.h
    Heap.h
    Geom.h
    version.h
    vmath.h
)

# Create the terrascape_demo executable (using bg_grid_mesh.h with detria)
add_executable(terrascape_demo main.cpp bg_detria_mesh.cpp)

# Create the new grid mesh test executable (using bg_grid_mesh.h with detria)
add_executable(test_gridmesh test_gridmesh.cpp bg_detria_mesh.cpp)

# Create the legacy terrascape executable (using the old Terra system)
add_executable(terrascape_legacy legacy_main.cpp ${TERRA_SOURCES})

# Include current directory for headers
target_include_directories(terrascape_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(test_gridmesh PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(terrascape_legacy PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Set output directory
set_target_properties(terrascape_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(test_gridmesh PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(terrascape_legacy PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Install targets
install(TARGETS terrascape_demo test_gridmesh terrascape_legacy DESTINATION bin)