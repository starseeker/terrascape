cmake_minimum_required(VERSION 3.12)
project(TerraScape VERSION 0.8)

# Set C++ standard to C++17 (required for bg_detria.hpp)
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

# Create the terrascape_demo executable (using TerraScape.hpp with detria)
add_executable(terrascape_demo main.cpp TerraScapeImpl.cpp terrain_data_utils.cpp)

# Create consolidated comprehensive test suite
add_executable(terrascape_tests terrascape_tests.cpp TerraScapeImpl.cpp terrain_data_utils.cpp)

# Create Hawaii BigIsland performance test
add_executable(hawaii_bigisland_test hawaii_bigisland_test.cpp TerraScapeImpl.cpp terrain_data_utils.cpp)

# Create volumetric mesh test
add_executable(volumetric_test volumetric_test.cpp TerraScapeImpl.cpp terrain_data_utils.cpp)

# Create separated volumetric mesh test
add_executable(volumetric_separated_test volumetric_separated_test.cpp TerraScapeImpl.cpp terrain_data_utils.cpp)

# Create separated volumetric mesh example
add_executable(volumetric_separated_example volumetric_separated_example.cpp TerraScapeImpl.cpp terrain_data_utils.cpp)

# Create preprocessing robustness test
add_executable(preprocessing_tests preprocessing_tests.cpp TerraScapeImpl.cpp terrain_data_utils.cpp)

# Create assertion failure prevention test  
add_executable(assertion_failure_tests assertion_failure_tests.cpp TerraScapeImpl.cpp terrain_data_utils.cpp)

# Include current directory for headers
target_include_directories(terrascape_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(terrascape_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(hawaii_bigisland_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(volumetric_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(volumetric_separated_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(volumetric_separated_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(preprocessing_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(assertion_failure_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Link with GDAL if available
if(GDAL_FOUND)
    target_include_directories(terrascape_demo PRIVATE ${GDAL_INCLUDE_DIRS})
    target_link_libraries(terrascape_demo ${GDAL_LIBRARIES})
    target_compile_options(terrascape_demo PRIVATE ${GDAL_CFLAGS_OTHER})
    
    target_include_directories(terrascape_tests PRIVATE ${GDAL_INCLUDE_DIRS})
    target_link_libraries(terrascape_tests ${GDAL_LIBRARIES})
    target_compile_options(terrascape_tests PRIVATE ${GDAL_CFLAGS_OTHER})
    
    target_include_directories(hawaii_bigisland_test PRIVATE ${GDAL_INCLUDE_DIRS})
    target_link_libraries(hawaii_bigisland_test ${GDAL_LIBRARIES})
    target_compile_options(hawaii_bigisland_test PRIVATE ${GDAL_CFLAGS_OTHER})
    
    # Also need curl for downloading
    find_package(CURL REQUIRED)
    target_link_libraries(terrascape_demo ${CURL_LIBRARIES})
    target_link_libraries(terrascape_tests ${CURL_LIBRARIES})
    target_link_libraries(hawaii_bigisland_test ${CURL_LIBRARIES})
endif()

# Set output directory
set_target_properties(terrascape_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(terrascape_tests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(hawaii_bigisland_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(volumetric_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(volumetric_separated_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(volumetric_separated_example PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(preprocessing_tests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(assertion_failure_tests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Install targets
install(TARGETS terrascape_demo terrascape_tests hawaii_bigisland_test volumetric_test volumetric_separated_test preprocessing_tests assertion_failure_tests DESTINATION bin)

# Add CTest tests
add_test(NAME ComprehensiveTestSuite COMMAND terrascape_tests)
add_test(NAME HawaiiBigIslandPerformanceTest COMMAND hawaii_bigisland_test)
add_test(NAME VolumetricMeshTest COMMAND volumetric_test)
add_test(NAME VolumetricSeparatedMeshTest COMMAND volumetric_separated_test)
add_test(NAME PreprocessingRobustnessTest COMMAND preprocessing_tests)
add_test(NAME AssertionFailurePreventionTest COMMAND assertion_failure_tests)

# Add GDAL-enabled terrain data tests if GDAL is available
if(GDAL_FOUND)
    # Create terrain data directory
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/terrain_data)
    
    # Add terrain data executable for downloading and processing real terrain data
    add_executable(terrain_data_processor 
        terrain_data_processor.cpp 
        TerraScapeImpl.cpp 
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
    
    # Make hawaii_bigisland_test depend on terrain data when GDAL is available
    add_dependencies(hawaii_bigisland_test terrain_data_processor)
    
    message(STATUS "GDAL terrain processing features enabled")
    message(STATUS "  - Use 'make download_hawaii_bigisland' to download and extract BigIsland terrain data")
    message(STATUS "  - Use 'make download_terrain_data' to download all Hawaii terrain files")
    message(STATUS "  - Run 'ctest -R RealTerrainDataTest' to test with real terrain data")
    message(STATUS "  - Run 'ctest -R HawaiiBigIslandPerformanceTest' to test BigIsland performance with real data")
endif()

# Additional tests can be added here if needed