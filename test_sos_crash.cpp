#include "TerraScape.hpp" 
#include "TerraScapeImpl.h"
#include <iostream>

// Test that reproduces the specific failure pattern seen in the debug output
int main() {
    std::cout << "=== Focused SoS Crash Test ===" << std::endl;
    
    // Try to create a scenario that mimics the failing test
    // Create an 8x8 grid with some specific values that trigger the issue
    int width = 8, height = 8;
    std::vector<float> grid(width * height);
    
    // Initialize with a simple gradient that might cause edge collinearity
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            grid[y * width + x] = float(x + y) * 0.1f;
        }
    }
    
    try {
        std::cout << "Testing 8x8 grid with gradient values..." << std::endl;
        auto result = TerraScape::grid_to_mesh(width, height, grid.data(), 0.05f, 30, 
                                               TerraScape::MeshRefineStrategy::HEAP);
        
        std::cout << "SUCCESS: Generated " << result.vertices.size() << " vertices, " 
                  << result.triangles.size() << " triangles" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cout << "EXCEPTION: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cout << "SEGFAULT or other crash occurred" << std::endl;
        return 1;
    }
}