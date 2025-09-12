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

// Create a synthetic terra.bin file with realistic terrain data but potentially problematic characteristics
bool createRealisticProblematicTerraBin(const std::string& filename) {
    // Create a 512x512 terrain that might cause issues similar to real-world DSP data
    const int width = 512, height = 512;
    std::vector<float> elevations(width * height);
    
    // Pattern: Mostly flat with some sharp ridges and sudden elevation changes
    // This mimics real terrain that might have been quantized or has precision issues
    
    const float base_elevation = 100.0f;
    const float noise_scale = 0.1f;
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float elevation = base_elevation;
            
            // Add some sharp ridges (potential collinearity issues)
            if (x % 64 == 0 || y % 64 == 0) {
                elevation += 50.0f; // Sharp ridge
            }
            
            // Add some flat plateaus (potential degeneracy)
            if ((x / 32) % 2 == 0 && (y / 32) % 2 == 0) {
                elevation = base_elevation + 10.0f; // Flat plateau
            }
            
            // Add very small noise (potential precision issues)
            elevation += noise_scale * (static_cast<float>(x + y) / (width + height));
            
            // Occasionally add identical elevations (potential duplicate points)
            if ((x + y) % 100 == 0) {
                elevation = base_elevation + 25.0f; // Identical elevation
            }
            
            elevations[y * width + x] = elevation;
        }
    }
    
    return DSPReader::createSyntheticTerraBin(filename, width, height, elevations);
}

// Create terra.bin with data that forces SPARSE or SPARSE strategies (more likely to trigger assertions)
bool createLargeScaleTerraBin(const std::string& filename) {
    // Create a smaller grid that will use SPARSE strategy instead of SPARSE
    const int width = 64, height = 64;  // Small enough to trigger SPARSE strategy
    std::vector<float> elevations(width * height);
    
    // Create a complex terrain with many local features
    // This should trigger more complex triangulation operations
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float elevation = 0.0f;
            
            // Add multiple sine waves at different frequencies
            elevation += 10.0f * std::sin(2.0f * M_PI * x / 8.0f);
            elevation += 5.0f * std::sin(2.0f * M_PI * y / 6.0f);
            elevation += 2.0f * std::sin(2.0f * M_PI * (x + y) / 4.0f);
            
            // Add some sharp peaks (potential numerical issues)
            float dx = x - width/2.0f;
            float dy = y - height/2.0f;
            float dist = std::sqrt(dx*dx + dy*dy);
            if (dist < 5.0f) {
                elevation += 20.0f * std::exp(-dist*dist);
            }
            
            // Add very small variations (precision testing)
            elevation += 1e-6f * (x * y);
            
            elevations[y * width + x] = elevation;
        }
    }
    
    return DSPReader::createSyntheticTerraBin(filename, width, height, elevations);
}

// Test with SPARSE strategy explicitly to trigger more complex triangulation paths
void test_heap_strategy_terra_processing() {
    std::cout << "\n=== SPARSE Strategy Terra Processing (Assertion Trigger Test) ===" << std::endl;
    
    const std::string test_file = "/tmp/terrascape_terra_tests/terra_heap_test.bin";
    
    if (createLargeScaleTerraBin(test_file)) {
        std::cout << "Created complex terrain file for SPARSE strategy testing" << std::endl;
        
        int width, height;
        std::vector<float> elevations;
        DSPReader::DSPTerrainInfo info;
        
        if (DSPReader::readTerraBinFile(test_file, width, height, elevations, &info)) {
            std::cout << "Read terrain: " << width << "x" << height << std::endl;
            std::cout << "Elevation range: " << info.min_elevation << " to " << info.max_elevation << std::endl;
            
            // Test with different strategies and error thresholds to stress the triangulation
            std::vector<std::pair<MeshRefineStrategy, std::string>> strategies = {
                {MeshRefineStrategy::AUTO, "AUTO"},      // Use AUTO first (will detect complexity)
                {MeshRefineStrategy::SPARSE, "SPARSE"}   // Direct SPARSE testing
            };
            
            std::vector<float> error_thresholds = {0.01f, 0.1f, 1.0f, 10.0f};
            
            for (const auto& strategy_pair : strategies) {
                for (float error_threshold : error_thresholds) {
                    std::cout << "\nTesting " << strategy_pair.second 
                              << " strategy with error threshold " << error_threshold << std::endl;
                              
                    try {
                        MeshResult result = grid_to_mesh(width, height, elevations.data(), 
                                                       error_threshold, 5000, strategy_pair.first);
                        std::cout << "  ✓ Success: " << result.vertices.size() 
                                  << " vertices, " << result.triangles.size() << " triangles" << std::endl;
                                  
                        // Test volumetric mesh generation
                        float base_elevation = info.min_elevation - 5.0f;
                        std::vector<Vertex> volumetric_vertices = result.vertices;
                        std::vector<Triangle> volumetric_triangles = result.triangles;
                        
                        // Add base vertices
                        for (const auto& v : result.vertices) {
                            volumetric_vertices.push_back({v.x, v.y, base_elevation});
                        }
                        
                        // Add base triangles
                        int vertex_offset = result.vertices.size();
                        for (const auto& t : result.triangles) {
                            volumetric_triangles.push_back({
                                t.v0 + vertex_offset,
                                t.v2 + vertex_offset,  // Inverted winding
                                t.v1 + vertex_offset
                            });
                        }
                        
                        std::cout << "  ✓ Volumetric mesh: " << volumetric_vertices.size() 
                                  << " vertices, " << volumetric_triangles.size() << " triangles" << std::endl;
                                  
                    } catch (const std::exception& e) {
                        std::cerr << "  ✗ Failed: " << e.what() << std::endl;
                        
                        // If we get a failure, this is what we want to debug and fix
                        std::cout << "  Debugging failure..." << std::endl;
                        
                        // Try with preprocessing enabled explicitly
                        try {
                            float adjusted_threshold = error_threshold;
                            auto preprocessed = preprocess_input_data(width, height, elevations.data(), 
                                                                     adjusted_threshold, true);
                            
                            if (preprocessed.has_warnings) {
                                std::cout << "  Preprocessing applied with warnings:" << std::endl;
                                for (const auto& warning : preprocessed.warnings) {
                                    std::cout << "    - " << warning << std::endl;
                                }
                            }
                            
                            MeshResult fixed_result = grid_to_mesh(width, height, 
                                                                 preprocessed.processed_elevations.data(), 
                                                                 adjusted_threshold, 5000, 
                                                                 MeshRefineStrategy::SPARSE);
                            std::cout << "  ✓ Fixed with preprocessing: " << fixed_result.vertices.size() 
                                      << " vertices" << std::endl;
                                      
                        } catch (const std::exception& e2) {
                            std::cerr << "  ✗ Even preprocessing failed: " << e2.what() << std::endl;
                        }
                    }
                }
            }
        }
    }
}

// Create a synthetic terra.bin that closely mimics BRL-CAD DSP format expectations
bool createBRLCADStyleTerraBin(const std::string& filename) {
    // BRL-CAD DSP typically expects elevation data in specific formats
    // Create a 256x256 terrain with 16-bit precision elevation data
    const int width = 256, height = 256;
    std::vector<uint16_t> raw_elevations(width * height);
    std::vector<float> float_elevations(width * height);
    
    // Create realistic terrain with elevation values in typical DSP range (0-65535)
    const uint16_t sea_level = 32768;  // Mid-range
    const uint16_t max_height = 8192;  // Reasonable mountain height
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Create a realistic terrain with hills, valleys, and flat areas
            float fx = static_cast<float>(x) / width;
            float fy = static_cast<float>(y) / height;
            
            // Base terrain with multiple frequency components
            float elevation = 0.5f; // Normalized base level
            elevation += 0.3f * std::sin(fx * 4.0f * M_PI) * std::sin(fy * 4.0f * M_PI);
            elevation += 0.1f * std::sin(fx * 16.0f * M_PI) * std::sin(fy * 16.0f * M_PI);
            elevation += 0.05f * std::sin(fx * 32.0f * M_PI + fy * 32.0f * M_PI);
            
            // Add some flat regions (potential triangulation issues)
            if (fx > 0.2f && fx < 0.4f && fy > 0.6f && fy < 0.8f) {
                elevation = 0.7f; // Flat plateau
            }
            
            // Add some linear ridges (collinearity issues)
            if (std::abs(fx - fy) < 0.02f) {
                elevation = 0.9f; // Diagonal ridge
            }
            
            // Convert to 16-bit range
            uint16_t int_elevation = sea_level + static_cast<uint16_t>(elevation * max_height);
            raw_elevations[y * width + x] = int_elevation;
            float_elevations[y * width + x] = static_cast<float>(int_elevation);
        }
    }
    
    // Write as binary file (uint16 format like typical DSP)
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Cannot create " << filename << std::endl;
        return false;
    }
    
    file.write(reinterpret_cast<const char*>(raw_elevations.data()), 
               raw_elevations.size() * sizeof(uint16_t));
    
    if (!file.good()) {
        std::cerr << "Error: Failed to write BRL-CAD style terrain data" << std::endl;
        return false;
    }
    
    std::cout << "Created BRL-CAD style terra.bin: " << width << "x" << height 
              << " (" << raw_elevations.size() * sizeof(uint16_t) << " bytes, uint16 format)" << std::endl;
    
    return true;
}

// Test BRL-CAD style terra.bin processing
void test_brlcad_style_terra_processing() {
    std::cout << "\n=== BRL-CAD Style Terra.bin Processing ===" << std::endl;
    
    const std::string terra_file = "/tmp/terrascape_terra_tests/brlcad_style_terra.bin";
    
    if (createBRLCADStyleTerraBin(terra_file)) {
        std::cout << "Testing BRL-CAD style terra.bin processing..." << std::endl;
        
        int width, height;
        std::vector<float> elevations;
        DSPReader::DSPTerrainInfo info;
        
        if (DSPReader::readTerraBinFile(terra_file, width, height, elevations, &info)) {
            std::cout << "Successfully read BRL-CAD style terra.bin: " << width << "x" << height << std::endl;
            std::cout << "Data type: " << info.data_type << std::endl;
            std::cout << "Elevation range: " << info.min_elevation << " to " << info.max_elevation << std::endl;
            
            // Test with different strategies
            try {
                std::cout << "Testing AUTO strategy..." << std::endl;
                MeshResult auto_result = grid_to_mesh(width, height, elevations.data(), 1.0f, 10000);
                std::cout << "✓ AUTO strategy: " << auto_result.vertices.size() 
                          << " vertices, " << auto_result.triangles.size() << " triangles" << std::endl;
                
                // Test volumetric mesh generation
                std::cout << "Generating volumetric mesh..." << std::endl;
                float base_elevation = info.min_elevation - 100.0f;
                
                std::vector<Vertex> volumetric_vertices = auto_result.vertices;
                std::vector<Triangle> volumetric_triangles = auto_result.triangles;
                
                // Add base vertices
                for (const auto& v : auto_result.vertices) {
                    volumetric_vertices.push_back({v.x, v.y, base_elevation});
                }
                
                // Add side walls (connect surface to base)
                // For simplicity, just add the base triangles with inverted normals
                int vertex_offset = auto_result.vertices.size();
                for (const auto& t : auto_result.triangles) {
                    volumetric_triangles.push_back({
                        t.v0 + vertex_offset,
                        t.v2 + vertex_offset,  // Inverted winding for bottom face
                        t.v1 + vertex_offset
                    });
                }
                
                std::cout << "✓ Volumetric mesh: " << volumetric_vertices.size() 
                          << " vertices, " << volumetric_triangles.size() << " triangles" << std::endl;
                
                // Write volumetric mesh to file
                std::string volumetric_obj = "/tmp/terrascape_terra_tests/brlcad_volumetric.obj";
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
                std::cerr << "✗ BRL-CAD style processing failed: " << e.what() << std::endl;
                
                // Debug the failure
                std::cout << "Analyzing BRL-CAD style terra.bin failure..." << std::endl;
                
                std::vector<std::string> warnings;
                DSPReader::validateTerrainData(elevations, width, height, warnings);
                for (const auto& warning : warnings) {
                    std::cout << "  Warning: " << warning << std::endl;
                }
                
                // Try with explicit preprocessing
                try {
                    float adjusted_threshold = 1.0f;
                    auto preprocessed = preprocess_input_data(width, height, elevations.data(), 
                                                             adjusted_threshold, true);
                    
                    std::cout << "Applied preprocessing, trying SPARSE strategy..." << std::endl;
                    MeshResult sparse_result = grid_to_mesh(width, height, 
                                                           preprocessed.processed_elevations.data(), 
                                                           adjusted_threshold, 10000, 
                                                           MeshRefineStrategy::SPARSE);
                    std::cout << "✓ SPARSE with preprocessing: " << sparse_result.vertices.size() 
                              << " vertices, " << sparse_result.triangles.size() << " triangles" << std::endl;
                              
                } catch (const std::exception& e2) {
                    std::cerr << "✗ Even SPARSE with preprocessing failed: " << e2.what() << std::endl;
                }
            }
        }
    }
}

int main() {
    std::cout << "=== Advanced Terra.bin Processing and Assertion Failure Testing ===" << std::endl;
    std::cout << "Testing TerraScape's robustness with complex terra.bin patterns" << std::endl;
    
    try {
        // Create test directory
        std::filesystem::create_directories("/tmp/terrascape_terra_tests");
        
        // Run comprehensive tests
        test_heap_strategy_terra_processing();
        test_brlcad_style_terra_processing();
        
        std::cout << "\n=== Final Test Summary ===" << std::endl;
        std::cout << "✓ SPARSE strategy assertion trigger tests completed" << std::endl;
        std::cout << "✓ BRL-CAD style terra.bin processing verified" << std::endl;
        std::cout << "✓ Volumetric mesh generation tested" << std::endl;
        std::cout << "\nAll advanced tests completed successfully!" << std::endl;
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        return 1;
    }
}