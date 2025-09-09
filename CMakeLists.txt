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

# Include current directory for headers
target_include_directories(terrascape_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(terrascape_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Link with GDAL if available
if(GDAL_FOUND)
    target_include_directories(terrascape_demo PRIVATE ${GDAL_INCLUDE_DIRS})
    target_link_libraries(terrascape_demo ${GDAL_LIBRARIES})
    target_compile_options(terrascape_demo PRIVATE ${GDAL_CFLAGS_OTHER})
    
    target_include_directories(terrascape_tests PRIVATE ${GDAL_INCLUDE_DIRS})
    target_link_libraries(terrascape_tests ${GDAL_LIBRARIES})
    target_compile_options(terrascape_tests PRIVATE ${GDAL_CFLAGS_OTHER})
    
    # Also need curl for downloading
    find_package(CURL REQUIRED)
    target_link_libraries(terrascape_demo ${CURL_LIBRARIES})
    target_link_libraries(terrascape_tests ${CURL_LIBRARIES})
endif()

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
    
    # Custom target to download terrain data
    add_custom_target(download_terrain_data
        COMMAND ${CMAKE_BINARY_DIR}/bin/terrain_data_processor --download-only
        DEPENDS terrain_data_processor
        COMMENT "Downloading Hawaii terrain data for testing"
    )
    
    message(STATUS "GDAL terrain processing features enabled")
    message(STATUS "  - Use 'make download_terrain_data' to download Hawaii terrain files")
    message(STATUS "  - Run 'ctest -R RealTerrainDataTest' to test with real terrain data")
endif()

# Additional tests can be added here if needed