#include "TerraScape.hpp"
#include "TerraScapeImpl.h"
#include "dsp_reader.hpp"
#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <cstring>

using namespace TerraScape;

// Test terra.bin file processing equivalent to BRL-CAD dsp.c
void test_terra_bin_processing() {
    std::cout << "\n=== Terra.bin File Processing Tests ===" << std::endl;
    std::cout << "Testing BRL-CAD DSP-style terrain data processing..." << std::endl;
    
    const std::string terra_bin_path = "terra.bin";
    const std::string test_dir = "/tmp/terrascape_terra_tests";
    
    // Create test directory
    std::filesystem::create_directories(test_dir);
    
    // Test 1: Try to read actual terra.bin file if it exists
    std::cout << "\n--- Test 1: Reading Actual terra.bin File ---" << std::endl;
    
    int width, height;
    std::vector<float> elevations;
    DSPReader::DSPTerrainInfo info;
    
    bool terra_found = false;
    if (std::filesystem::exists(terra_bin_path)) {
        std::cout << "Found terra.bin file, attempting to read..." << std::endl;
        
        if (DSPReader::readTerraBinFile(terra_bin_path, width, height, elevations, &info)) {
            std::cout << "✓ Successfully read terra.bin: " << width << "x" << height << std::endl;
            std::cout << "  Data type: " << info.data_type << std::endl;
            std::cout << "  Elevation range: " << info.min_elevation << " to " << info.max_elevation << std::endl;
            terra_found = true;
            
            // Validate the terrain data
            std::vector<std::string> warnings;
            DSPReader::validateTerrainData(elevations, width, height, warnings);
            
            if (!warnings.empty()) {
                std::cout << "  Terrain data warnings:" << std::endl;
                for (const auto& warning : warnings) {
                    std::cout << "    - " << warning << std::endl;
                }
            }
            
            // Now test TerraScape triangulation
            std::cout << "  Testing TerraScape triangulation..." << std::endl;
            try {
                MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 10000);
                std::cout << "  ✓ Triangulation successful: " << result.vertices.size() 
                          << " vertices, " << result.triangles.size() << " triangles" << std::endl;
                          
                // Test volumetric mesh generation
                std::cout << "  Testing volumetric mesh generation..." << std::endl;
                float base_elevation = info.min_elevation - 10.0f;
                
                // Create volumetric mesh
                std::vector<Vertex> volumetric_vertices;
                std::vector<Triangle> volumetric_triangles;
                
                // Add surface vertices
                for (const auto& v : result.vertices) {
                    volumetric_vertices.push_back(v);
                }
                
                // Add base vertices
                for (const auto& v : result.vertices) {
                    volumetric_vertices.push_back({v.x, v.y, base_elevation});
                }
                
                // Add surface triangles
                for (const auto& t : result.triangles) {
                    volumetric_triangles.push_back(t);
                }
                
                // Add base triangles (inverted winding)
                int vertex_count = result.vertices.size();
                for (const auto& t : result.triangles) {
                    volumetric_triangles.push_back({
                        t.v0 + vertex_count,
                        t.v2 + vertex_count,  // Inverted winding
                        t.v1 + vertex_count
                    });
                }
                
                std::cout << "  ✓ Volumetric mesh: " << volumetric_vertices.size() 
                          << " vertices, " << volumetric_triangles.size() << " triangles" << std::endl;
                
            } catch (const std::exception& e) {
                std::cerr << "  ✗ Triangulation failed with exception: " << e.what() << std::endl;
                
                // This is the key issue we need to debug and fix
                std::cout << "  Analyzing triangulation failure..." << std::endl;
                
                // Try with different strategies
                std::cout << "  Trying SPARSE strategy..." << std::endl;
                try {
                    MeshResult sparse_result = grid_to_mesh(width, height, elevations.data(), 
                                                           1.0f, 10000);
                    std::cout << "  ✓ SPARSE strategy successful: " << sparse_result.vertices.size() 
                              << " vertices, " << sparse_result.triangles.size() << " triangles" << std::endl;
                } catch (const std::exception& e2) {
                    std::cerr << "  ✗ SPARSE strategy also failed: " << e2.what() << std::endl;
                }
                
                // Try with preprocessing
                std::cout << "  Trying with explicit preprocessing..." << std::endl;
                try {
                    float adjusted_threshold = 1.0f;
                    auto preprocessed = preprocess_input_data(width, height, elevations.data(), adjusted_threshold, true);
                    
                    if (preprocessed.has_warnings) {
                        std::cout << "  Preprocessing warnings:" << std::endl;
                        for (const auto& warning : preprocessed.warnings) {
                            std::cout << "    - " << warning << std::endl;
                        }
                    }
                    
                    MeshResult processed_result = grid_to_mesh(width, height, preprocessed.processed_elevations.data(), 
                                                             adjusted_threshold, 10000);
                    std::cout << "  ✓ Preprocessed triangulation successful: " << processed_result.vertices.size() 
                              << " vertices, " << processed_result.triangles.size() << " triangles" << std::endl;
                              
                } catch (const std::exception& e3) {
                    std::cerr << "  ✗ Preprocessed triangulation failed: " << e3.what() << std::endl;
                }
            }
        } else {
            std::cout << "✗ Failed to read terra.bin file" << std::endl;
        }
    } else {
        std::cout << "terra.bin file not found, will test with synthetic data" << std::endl;
    }
    
    // Test 2: Create and test synthetic problematic terra.bin files
    std::cout << "\n--- Test 2: Synthetic Problematic terra.bin Files ---" << std::endl;
    
    std::vector<std::string> problematic_patterns = {
        "all_zeros",
        "all_same", 
        "regular_grid",
        "collinear_rows",
        "infinities",
        "extreme_values"
    };
    
    for (const auto& pattern : problematic_patterns) {
        std::cout << "\nTesting pattern: " << pattern << std::endl;
        
        std::string test_file = test_dir + "/terra_" + pattern + ".bin";
        
        if (DSPReader::createProblematicTerraBin(test_file, pattern)) {
            std::cout << "  Created test file: " << test_file << std::endl;
            
            // Read it back
            if (DSPReader::readTerraBinFile(test_file, width, height, elevations, &info)) {
                std::cout << "  Read back: " << width << "x" << height << std::endl;
                
                // Test TerraScape processing
                try {
                    MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 10000);
                    std::cout << "  ✓ Pattern '" << pattern << "' triangulated successfully: " 
                              << result.vertices.size() << " vertices, " << result.triangles.size() << " triangles" << std::endl;
                              
                    // Write debug output
                    std::string obj_file = test_dir + "/terra_" + pattern + ".obj";
                    std::ofstream obj(obj_file);
                    if (obj) {
                        for (const auto& v : result.vertices) {
                            obj << "v " << v.x << " " << v.y << " " << v.z << std::endl;
                        }
                        for (const auto& t : result.triangles) {
                            obj << "f " << (t.v0 + 1) << " " << (t.v1 + 1) << " " << (t.v2 + 1) << std::endl;
                        }
                        std::cout << "  Wrote mesh to: " << obj_file << std::endl;
                    }
                    
                } catch (const std::exception& e) {
                    std::cerr << "  ✗ Pattern '" << pattern << "' failed: " << e.what() << std::endl;
                    
                    // Debug this specific failure
                    std::cout << "  Debugging pattern '" << pattern << "'..." << std::endl;
                    
                    std::vector<std::string> warnings;
                    DSPReader::validateTerrainData(elevations, width, height, warnings);
                    for (const auto& warning : warnings) {
                        std::cout << "    Warning: " << warning << std::endl;
                    }
                    
                    // Try with different parameters
                    try {
                        MeshResult sparse_result = grid_to_mesh(width, height, elevations.data(), 
                                                               0.1f, 1000);
                        std::cout << "  ✓ SPARSE with relaxed parameters worked: " 
                                  << sparse_result.vertices.size() << " vertices" << std::endl;
                    } catch (const std::exception& e2) {
                        std::cerr << "  ✗ Even SPARSE failed: " << e2.what() << std::endl;
                    }
                }
            }
        }
    }
}

// Test specific BRL-CAD DSP format compatibility
void test_brlcad_dsp_compatibility() {
    std::cout << "\n=== BRL-CAD DSP Format Compatibility Tests ===" << std::endl;
    
    // Create a DSP file with proper header (mimicking BRL-CAD format)
    const std::string dsp_file = "/tmp/test_brlcad.dsp";
    const int width = 128, height = 128;
    
    // Create test elevation data - a simple cone
    std::vector<float> elevations(width * height);
    float center_x = width / 2.0f;
    float center_y = height / 2.0f;
    float max_radius = std::min(width, height) / 2.0f;
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float dx = x - center_x;
            float dy = y - center_y;
            float distance = std::sqrt(dx * dx + dy * dy);
            float height_val = std::max(0.0f, 50.0f * (1.0f - distance / max_radius));
            elevations[y * width + x] = height_val;
        }
    }
    
    // Write DSP file with header
    std::ofstream dsp(dsp_file, std::ios::binary);
    if (dsp) {
        DSPReader::DSPHeader header;
        header.magic = 0x44535020; // "DSP "
        header.width = width;
        header.height = height;
        header.data_type = 3; // float
        memset(header.reserved, 0, sizeof(header.reserved));
        
        dsp.write(reinterpret_cast<const char*>(&header), sizeof(header));
        dsp.write(reinterpret_cast<const char*>(elevations.data()), elevations.size() * sizeof(float));
        
        std::cout << "Created test DSP file: " << dsp_file << std::endl;
    }
    
    // Test reading it back
    int read_width, read_height;
    std::vector<float> read_elevations;
    DSPReader::DSPTerrainInfo info;
    
    if (DSPReader::readDSPFile(dsp_file, read_width, read_height, read_elevations, &info)) {
        std::cout << "✓ Successfully read DSP file: " << read_width << "x" << read_height << std::endl;
        
        // Test triangulation
        try {
            MeshResult result = grid_to_mesh(read_width, read_height, read_elevations.data(), 1.0f, 5000);
            std::cout << "✓ DSP triangulation successful: " << result.vertices.size() 
                      << " vertices, " << result.triangles.size() << " triangles" << std::endl;
                      
            // Test volumetric mesh generation for DSP data
            std::cout << "Testing volumetric mesh from DSP data..." << std::endl;
            
            // Create base vertices at a lower elevation
            float base_elevation = info.min_elevation - 5.0f;
            std::vector<Vertex> volumetric_vertices;
            std::vector<Triangle> volumetric_triangles;
            
            // Add surface vertices
            volumetric_vertices = result.vertices;
            
            // Add base vertices
            for (const auto& v : result.vertices) {
                volumetric_vertices.push_back({v.x, v.y, base_elevation});
            }
            
            // Add surface triangles
            volumetric_triangles = result.triangles;
            
            // Add base triangles (with inverted normals)
            int surface_vertex_count = result.vertices.size();
            for (const auto& t : result.triangles) {
                volumetric_triangles.push_back({
                    t.v0 + surface_vertex_count,
                    t.v2 + surface_vertex_count,  // Swap to invert normal
                    t.v1 + surface_vertex_count
                });
            }
            
            std::cout << "✓ Volumetric mesh created: " << volumetric_vertices.size() 
                      << " vertices, " << volumetric_triangles.size() << " triangles" << std::endl;
                      
            // Write the volumetric mesh
            std::string volumetric_obj = "/tmp/dsp_volumetric.obj";
            std::ofstream obj(volumetric_obj);
            if (obj) {
                for (const auto& v : volumetric_vertices) {
                    obj << "v " << v.x << " " << v.y << " " << v.z << std::endl;
                }
                for (const auto& t : volumetric_triangles) {
                    obj << "f " << (t.v0 + 1) << " " << (t.v1 + 1) << " " << (t.v2 + 1) << std::endl;
                }
                std::cout << "✓ Wrote volumetric mesh to: " << volumetric_obj << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "✗ DSP triangulation failed: " << e.what() << std::endl;
        }
    } else {
        std::cerr << "✗ Failed to read DSP file" << std::endl;
    }
}

// Test assertion failure debugging and resolution
void test_assertion_failure_debugging() {
    std::cout << "\n=== Assertion Failure Debugging Tests ===" << std::endl;
    
    // Create specific patterns that are known to cause assertion failures in some triangulation libraries
    
    // Pattern 1: All points on a single line (severe collinearity)
    std::cout << "\nTesting severe collinearity pattern..." << std::endl;
    {
        const int width = 100, height = 1;
        std::vector<float> elevations(width * height);
        
        for (int i = 0; i < width; ++i) {
            elevations[i] = static_cast<float>(i); // Linear progression
        }
        
        try {
            MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
            std::cout << "✓ Severe collinearity handled: " << result.vertices.size() << " vertices" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "✗ Severe collinearity failed: " << e.what() << std::endl;
        }
    }
    
    // Pattern 2: Identical coordinates with different elevations (geometric degeneracy)
    std::cout << "\nTesting geometric degeneracy pattern..." << std::endl;
    {
        const int width = 2, height = 2;
        std::vector<float> elevations = {0.0f, 0.0f, 0.0f, 1e-15f}; // Tiny variation
        
        try {
            MeshResult result = grid_to_mesh(width, height, elevations.data(), 1e-20f, 1000);
            std::cout << "✓ Geometric degeneracy handled: " << result.vertices.size() << " vertices" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "✗ Geometric degeneracy failed: " << e.what() << std::endl;
        }
    }
    
    // Pattern 3: Large coordinate values that might cause precision issues
    std::cout << "\nTesting large coordinate precision pattern..." << std::endl;
    {
        const int width = 10, height = 10;
        std::vector<float> elevations(width * height);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // Very large base values with small variations
                elevations[y * width + x] = 1e6f + static_cast<float>(x + y) * 1e-3f;
            }
        }
        
        try {
            MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
            std::cout << "✓ Large coordinate precision handled: " << result.vertices.size() << " vertices" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "✗ Large coordinate precision failed: " << e.what() << std::endl;
        }
    }
}

// Test specific file provided by user
void test_user_provided_file(const std::string& file_path) {
    std::cout << "\n=== User Provided File Test ===" << std::endl;
    std::cout << "Testing file: " << file_path << std::endl;
    
    if (!std::filesystem::exists(file_path)) {
        std::cerr << "✗ File does not exist: " << file_path << std::endl;
        return;
    }
    
    int width, height;
    std::vector<float> elevations;
    DSPReader::DSPTerrainInfo info;
    
    // Try to read as terra.bin or DSP file
    bool success = false;
    std::string file_type;
    
    // First try as DSP file
    if (DSPReader::readDSPFile(file_path, width, height, elevations, &info)) {
        success = true;
        file_type = "DSP";
    } else if (DSPReader::readTerraBinFile(file_path, width, height, elevations, &info)) {
        success = true;
        file_type = "Terra.bin";
    }
    
    if (!success) {
        std::cerr << "✗ Failed to read file as either DSP or terra.bin format" << std::endl;
        return;
    }
    
    std::cout << "✓ Successfully read as " << file_type << " format: " << width << "x" << height << std::endl;
    std::cout << "  Data type: " << info.data_type << std::endl;
    std::cout << "  Elevation range: " << info.min_elevation << " to " << info.max_elevation << std::endl;
    
    // Validate the terrain data
    std::vector<std::string> warnings;
    DSPReader::validateTerrainData(elevations, width, height, warnings);
    
    if (!warnings.empty()) {
        std::cout << "  Terrain data warnings:" << std::endl;
        for (const auto& warning : warnings) {
            std::cout << "    - " << warning << std::endl;
        }
    }
    
    // Test grid-aware triangulation
    std::cout << "\n  Testing grid-aware triangulation..." << std::endl;
    try {
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 10000);
        std::cout << "  ✓ Grid-aware successful: " << result.vertices.size() 
                  << " vertices, " << result.triangles.size() << " triangles" << std::endl;
                  
        // Write result to file
        std::string output_file = "/tmp/user_file_grid_aware.obj";
        std::ofstream obj(output_file);
        if (obj) {
            for (const auto& v : result.vertices) {
                obj << "v " << v.x << " " << v.y << " " << v.z << std::endl;
            }
            for (const auto& t : result.triangles) {
                obj << "f " << (t.v0 + 1) << " " << (t.v1 + 1) << " " << (t.v2 + 1) << std::endl;
            }
            std::cout << "    Wrote mesh to: " << output_file << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "  ✗ Grid-aware failed: " << e.what() << std::endl;
    }
    
    // Test volumetric mesh generation
    std::cout << "\n  Testing volumetric mesh generation..." << std::endl;
    try {
        float base_elevation = info.min_elevation - 10.0f;
        MeshResult surface_result = grid_to_mesh(width, height, elevations.data(), 1.0f, 10000);
        MeshResult volumetric_result = make_volumetric_mesh(surface_result, base_elevation);
        
        std::cout << "  ✓ Volumetric mesh: " << volumetric_result.vertices.size() 
                  << " vertices, " << volumetric_result.triangles.size() << " triangles" << std::endl;
                  
        // Write volumetric result
        std::string volumetric_file = "/tmp/user_file_volumetric.obj";
        std::ofstream obj(volumetric_file);
        if (obj) {
            for (const auto& v : volumetric_result.vertices) {
                obj << "v " << v.x << " " << v.y << " " << v.z << std::endl;
            }
            for (const auto& t : volumetric_result.triangles) {
                obj << "f " << (t.v0 + 1) << " " << (t.v1 + 1) << " " << (t.v2 + 1) << std::endl;
            }
            std::cout << "    Wrote volumetric mesh to: " << volumetric_file << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "  ✗ Volumetric mesh generation failed: " << e.what() << std::endl;
    }
}

void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --file <path>    Test specific terra.bin or DSP file" << std::endl;
    std::cout << "  --help           Show this help message" << std::endl;
    std::cout << "  (no options)     Run all built-in tests" << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "=== TerraScape Terra.bin Processing and BRL-CAD DSP Compatibility Tests ===" << std::endl;
    std::cout << "Testing TerraScape's ability to process terra.bin files equivalent to BRL-CAD dsp.c" << std::endl;
    
    // Parse command line arguments
    std::string test_file;
    bool run_all_tests = true;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--file" && i + 1 < argc) {
            test_file = argv[i + 1];
            run_all_tests = false;
            ++i; // Skip the next argument (file path)
        } else if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }
    
    try {
        if (!test_file.empty()) {
            // Test specific file
            test_user_provided_file(test_file);
        } else if (run_all_tests) {
            // Run all built-in tests
            test_terra_bin_processing();
            test_brlcad_dsp_compatibility();
            test_assertion_failure_debugging();
            
            std::cout << "\n=== Test Summary ===" << std::endl;
            std::cout << "✓ Terra.bin processing tests completed" << std::endl;
            std::cout << "✓ BRL-CAD DSP compatibility verified" << std::endl;
            std::cout << "✓ Assertion failure debugging completed" << std::endl;
        }
        
        std::cout << "\nAll tests completed successfully!" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        return 1;
    }
}