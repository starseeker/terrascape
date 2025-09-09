cmake_minimum_required(VERSION 3.12)
project(TerraScape VERSION 0.7)

# Set C++ standard to C++17 (required for bg_detria.hpp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable CTest framework
enable_testing()

# Add compile definitions
add_definitions(-DSAFETY)

# Create the terrascape_demo executable (using TerraScape.hpp with detria)
add_executable(terrascape_demo main.cpp TerraScapeImpl.cpp)

# Create the new grid mesh test executable (using TerraScape.hpp with detria)
add_executable(test_gridmesh test_gridmesh.cpp TerraScapeImpl.cpp)

# Create the new detria integration test
add_executable(test_detria_integration test_detria_integration.cpp TerraScapeImpl.cpp)

# Create the incremental insertion demonstration
add_executable(test_incremental_demo test_incremental_demo.cpp TerraScapeImpl.cpp)

# Create focused correctness tests  
add_executable(test_correctness_fixes test_correctness_fixes.cpp TerraScapeImpl.cpp)

# Create batch insertion performance test
add_executable(test_batch_performance test_batch_performance.cpp TerraScapeImpl.cpp)

# Create triangle winding consistency test
add_executable(test_triangle_winding test_triangle_winding.cpp TerraScapeImpl.cpp)

# Create comprehensive SoS test
add_executable(test_sos_comprehensive test_sos_comprehensive.cpp)

# Include current directory for headers
target_include_directories(terrascape_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(test_gridmesh PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(test_detria_integration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(test_incremental_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(test_correctness_fixes PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(test_batch_performance PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(test_triangle_winding PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(test_sos_comprehensive PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Set output directory
set_target_properties(terrascape_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(test_gridmesh PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(test_detria_integration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(test_incremental_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(test_correctness_fixes PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(test_batch_performance PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(test_triangle_winding PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(test_sos_comprehensive PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Install targets
install(TARGETS terrascape_demo test_gridmesh test_detria_integration test_incremental_demo test_correctness_fixes test_batch_performance test_triangle_winding test_sos_comprehensive DESTINATION bin)

# Add CTest tests
add_test(NAME CorrectnessFixesTest COMMAND test_correctness_fixes)
add_test(NAME DetriaIntegrationTest COMMAND test_detria_integration)