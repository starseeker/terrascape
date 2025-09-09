#include "TerraScape.hpp"
#include "TerraScapeImpl.h"
#include <iostream>

// Test specifically designed to trigger the "point on edge" issue
// that SoS should resolve
int main() {
    std::cout << "=== SoS Degeneracy Test ===" << std::endl;
    
    // Create a 3x3 grid that should trigger exact collinearity
    int width = 3, height = 3;
    float grid[9] = {
        0.0f, 0.0f, 0.0f,  // Points will be at (0,0), (1,0), (2,0) - collinear!
        0.0f, 1.0f, 0.0f,  
        0.0f, 0.0f, 0.0f   
    };
    
    try {
        std::cout << "Testing 3x3 grid with potential collinear points..." << std::endl;
        auto result = TerraScape::grid_to_mesh(width, height, grid, 0.01f, 50, 
                                               TerraScape::MeshRefineStrategy::HEAP);
        
        std::cout << "SUCCESS: Generated " << result.vertices.size() << " vertices, " 
                  << result.triangles.size() << " triangles" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cout << "EXCEPTION: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cout << "UNKNOWN EXCEPTION occurred" << std::endl;
        return 1;
    }
}