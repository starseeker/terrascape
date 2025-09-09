cmake_minimum_required(VERSION 3.12)
project(TerraScape VERSION 0.8)

# Set C++ standard to C++17 (required for bg_detria.hpp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable CTest framework
enable_testing()

# Add compile definitions
add_definitions(-DSAFETY)

# Create the terrascape_demo executable (using TerraScape.hpp with detria)
add_executable(terrascape_demo main.cpp TerraScapeImpl.cpp)

# Create consolidated comprehensive test suite
add_executable(terrascape_tests terrascape_tests.cpp TerraScapeImpl.cpp)

# Include current directory for headers
target_include_directories(terrascape_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(terrascape_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Set output directory
set_target_properties(terrascape_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(terrascape_tests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Install targets
install(TARGETS terrascape_demo terrascape_tests DESTINATION bin)

# Add CTest tests
add_test(NAME ComprehensiveTestSuite COMMAND terrascape_tests)

# Add SoS performance tests
add_executable(sos_performance_tests 
    /tmp/sos_performance_tests.cpp 
    TerraScapeImpl.cpp
)
target_include_directories(sos_performance_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(sos_performance_tests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
add_test(NAME SoSPerformanceTests COMMAND sos_performance_tests)

# Add SoS analysis tests
add_executable(sos_analysis 
    /tmp/sos_analysis.cpp 
    TerraScapeImpl.cpp
)
target_include_directories(sos_analysis PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(sos_analysis PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
add_test(NAME SoSAnalysis COMMAND sos_analysis)