#include <iostream>
#include <vector>
#include <chrono>
#include <cstring>
#include "greedy_cuts.hpp"

// Test edge cases for robustness
void testEdgeCase(const std::string& test_name, std::vector<float> elevations, int width, int height, const uint8_t* mask = nullptr) {
    std::cout << "\n=== " << test_name << " ===" << std::endl;
    std::cout << "Grid size: " << width << "x" << height << std::endl;
    
    terrascape::Mesh mesh;
    auto start = std::chrono::high_resolution_clock::now();
    
    try {
        terrascape::triangulateRegionGrowing(elevations.data(), width, height, mesh, 0.5, mask);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        
        std::cout << "✓ Success: " << duration << "ms" << std::endl;
        std::cout << "  Vertices: " << mesh.vertices.size() << std::endl;
        std::cout << "  Triangles: " << mesh.triangles.size() << std::endl;
        
        // Basic sanity checks
        bool valid = true;
        if (mesh.vertices.empty()) {
            std::cout << "  ⚠ Warning: No vertices generated" << std::endl;
            valid = false;
        }
        if (mesh.triangles.empty() && mesh.vertices.size() >= 3) {
            std::cout << "  ⚠ Warning: No triangles generated with " << mesh.vertices.size() << " vertices" << std::endl;
            valid = false;
        }
        
        // Check triangle indices are valid
        for (const auto& tri : mesh.triangles) {
            for (int i = 0; i < 3; ++i) {
                if (tri[i] < 0 || tri[i] >= static_cast<int>(mesh.vertices.size())) {
                    std::cout << "  ✗ Error: Invalid triangle index " << tri[i] << std::endl;
                    valid = false;
                    break;
                }
            }
            if (!valid) break;
        }
        
        if (valid) {
            std::cout << "  ✓ Mesh validation passed" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "✗ Exception: " << e.what() << std::endl;
    } catch (...) {
        std::cout << "✗ Unknown exception" << std::endl;
    }
}

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "         TERRASCAPE EDGE CASE ROBUSTNESS TEST" << std::endl;
    std::cout << "==================================================" << std::endl;
    
    // Test 1: Minimal grid
    {
        std::vector<float> elevations = {1.0f, 2.0f, 3.0f, 4.0f};
        testEdgeCase("Minimal 2x2 Grid", elevations, 2, 2);
    }
    
    // Test 2: Flat terrain
    {
        std::vector<float> elevations(100, 5.0f);  // 10x10 grid, all same height
        testEdgeCase("Flat Terrain (10x10)", elevations, 10, 10);
    }
    
    // Test 3: Single spike
    {
        std::vector<float> elevations(100, 1.0f);
        elevations[55] = 100.0f;  // Single spike in the middle
        testEdgeCase("Single Spike (10x10)", elevations, 10, 10);
    }
    
    // Test 4: Checkerboard pattern
    {
        std::vector<float> elevations(100);
        for (int y = 0; y < 10; ++y) {
            for (int x = 0; x < 10; ++x) {
                elevations[y * 10 + x] = ((x + y) % 2) ? 1.0f : 10.0f;
            }
        }
        testEdgeCase("Checkerboard Pattern (10x10)", elevations, 10, 10);
    }
    
    // Test 5: Gradient terrain
    {
        std::vector<float> elevations(200);  // 20x10
        for (int y = 0; y < 10; ++y) {
            for (int x = 0; x < 20; ++x) {
                elevations[y * 20 + x] = static_cast<float>(x + y);
            }
        }
        testEdgeCase("Gradient Terrain (20x10)", elevations, 20, 10);
    }
    
    // Test 6: Masked terrain (half masked out)
    {
        std::vector<float> elevations(100);
        std::vector<uint8_t> mask(100);
        for (int i = 0; i < 100; ++i) {
            elevations[i] = static_cast<float>(i % 20);
            mask[i] = (i % 2) ? 1 : 0;  // Checkerboard mask
        }
        testEdgeCase("Masked Terrain (10x10, 50% masked)", elevations, 10, 10, mask.data());
    }
    
    // Test 7: Large sparse grid
    {
        std::vector<float> elevations(10000, 1.0f);  // 100x100, mostly flat
        // Add a few random spikes
        elevations[1234] = 50.0f;
        elevations[5678] = 75.0f;
        elevations[9012] = 25.0f;
        testEdgeCase("Large Sparse Grid (100x100)", elevations, 100, 100);
    }
    
    // Test 8: Extreme values
    {
        std::vector<float> elevations = {
            -1000.0f, 0.0f, 1000.0f, 
            std::numeric_limits<float>::lowest(), 
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::epsilon()
        };
        elevations.resize(9, 0.0f);  // Pad to 3x3
        testEdgeCase("Extreme Values (3x3)", elevations, 3, 3);
    }
    
    std::cout << "\n==================================================" << std::endl;
    std::cout << "Edge case testing completed." << std::endl;
    std::cout << "==================================================" << std::endl;
    
    return 0;
}