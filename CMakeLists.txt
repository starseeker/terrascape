cmake_minimum_required(VERSION 3.12)
project(TerraScape VERSION 0.8)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable CTest framework
enable_testing()

# Find GDAL as optional dependency
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(GDAL gdal)
    if(GDAL_FOUND)
        message(STATUS "GDAL found: ${GDAL_VERSION}")
        add_definitions(-DHAVE_GDAL)
    else()
        message(STATUS "GDAL not found - terrain data download and validation features disabled")
    endif()
else()
    message(STATUS "PkgConfig not found - GDAL detection disabled")
endif()

# Add compile definitions
add_definitions(-DSAFETY)

# Create the terrascape_demo executable (now header-only)
add_executable(terrascape_demo main.cpp terrain_data_utils.cpp)

# Create unified test suite with cxxopts
add_executable(unified_tests unified_tests.cpp terrain_data_utils.cpp dsp_reader.cpp)

# Include current directory for headers
target_include_directories(terrascape_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(unified_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Link with GDAL if available
if(GDAL_FOUND)
    target_include_directories(terrascape_demo PRIVATE ${GDAL_INCLUDE_DIRS})
    target_link_libraries(terrascape_demo ${GDAL_LIBRARIES})
    target_compile_options(terrascape_demo PRIVATE ${GDAL_CFLAGS_OTHER})
    
    target_include_directories(unified_tests PRIVATE ${GDAL_INCLUDE_DIRS})
    target_link_libraries(unified_tests ${GDAL_LIBRARIES})
    target_compile_options(unified_tests PRIVATE ${GDAL_CFLAGS_OTHER})
    
    # Also need curl for downloading
    find_package(CURL REQUIRED)
    target_link_libraries(terrascape_demo ${CURL_LIBRARIES})
    target_link_libraries(unified_tests ${CURL_LIBRARIES})
endif()

# Set output directory
set_target_properties(terrascape_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(unified_tests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Install targets
install(TARGETS terrascape_demo unified_tests DESTINATION bin)

# Add CTest tests using the unified test executable
add_test(NAME BasicFunctionalityTests COMMAND unified_tests --basic --verbose)
add_test(NAME VolumetricMeshTests COMMAND unified_tests --volumetric --verbose)
add_test(NAME PerformanceTests COMMAND unified_tests --performance --verbose)
add_test(NAME DSPHawaiiTests COMMAND unified_tests --dsp --verbose)
add_test(NAME EdgeCaseTests COMMAND unified_tests --edge --verbose)
add_test(NAME AllTests COMMAND unified_tests --all --verbose)

# Add GDAL-enabled terrain data tests if GDAL is available
if(GDAL_FOUND)
    # Create terrain data directory
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/terrain_data)
    
    # Add terrain data executable for downloading and processing real terrain data
    add_executable(terrain_data_processor 
        terrain_data_processor.cpp 
        terrain_data_utils.cpp
    )
    target_include_directories(terrain_data_processor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_include_directories(terrain_data_processor PRIVATE ${GDAL_INCLUDE_DIRS})
    target_link_libraries(terrain_data_processor ${GDAL_LIBRARIES} ${CURL_LIBRARIES})
    target_compile_options(terrain_data_processor PRIVATE ${GDAL_CFLAGS_OTHER})
    set_target_properties(terrain_data_processor PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
    
    # Add test for real terrain data processing
    add_test(NAME RealTerrainDataTest COMMAND terrain_data_processor)
    
    # Custom target to download and extract Hawaii BigIsland terrain data
    add_custom_target(download_hawaii_bigisland
        COMMAND ${CMAKE_COMMAND} -E echo "Downloading Hawaii BigIsland terrain data..."
        COMMAND ${CMAKE_BINARY_DIR}/bin/terrain_data_processor --download-only
        COMMAND ${CMAKE_COMMAND} -E echo "Extracting bigisland.zip..."
        COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_BINARY_DIR}/terrain_data tar xf bigisland.zip
        DEPENDS terrain_data_processor
        COMMENT "Downloading and extracting Hawaii BigIsland terrain data for real terrain testing"
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    
    # Custom target to download all terrain data
    add_custom_target(download_terrain_data
        COMMAND ${CMAKE_BINARY_DIR}/bin/terrain_data_processor --download-only
        DEPENDS terrain_data_processor
        COMMENT "Downloading Hawaii terrain data for testing"
    )
    
    message(STATUS "GDAL terrain processing features enabled")
    message(STATUS "  - Use 'make download_hawaii_bigisland' to download and extract BigIsland terrain data")
    message(STATUS "  - Use 'make download_terrain_data' to download all Hawaii terrain files")
    message(STATUS "  - Run 'ctest -R RealTerrainDataTest' to test with real terrain data")
else()
    message(STATUS "Header-only TerraScape library ready for use")
    message(STATUS "  - All functionality consolidated into TerraScape.hpp")
    message(STATUS "  - Use unified_tests executable for all testing")
endif()

# Information about running tests
message(STATUS "Test execution:")
message(STATUS "  - Run 'ctest' to run all tests")
message(STATUS "  - Run 'ctest -R BasicFunctionalityTests' for core functionality")
message(STATUS "  - Run 'ctest -R PerformanceTests' for performance validation")
message(STATUS "  - Run 'bin/unified_tests --help' for detailed test options")