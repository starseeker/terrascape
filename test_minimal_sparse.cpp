#include <iostream>
#include <vector>
#include <cassert>
#include "TerraScape.hpp"

// Add debugging to understand where the assertion is failing
int main() {
    std::cout << "=== Debugging SPARSE Strategy Assertion Failure ===" << std::endl;
    
    // Create a very minimal test case to isolate the issue
    std::vector<float> data = {
        1.0f, 1.0f,
        1.0f, 2.0f
    };
    int width = 2, height = 2;
    
    std::cout << "Testing with minimal 2x2 grid:" << std::endl;
    
    try {
        // Test SPARSE strategy with minimal points
        auto result = TerraScape::grid_to_mesh(
            width, height, data.data(), 
            0.1f,  // error_threshold
            20,    // point_limit
            TerraScape::MeshRefineStrategy::SPARSE
        );
        
        std::cout << "Minimal SPARSE test succeeded" << std::endl;
        
    } catch (...) {
        std::cout << "Even minimal SPARSE test failed" << std::endl;
    }
    
    // Try with a slightly larger grid
    std::vector<float> data3x3 = {
        1.0f, 1.0f, 1.0f,
        1.0f, 1.1f, 1.0f,  // Changed from 2.0f to 1.1f to avoid extreme elevation difference
        1.0f, 1.0f, 1.0f
    };
    width = 3; height = 3;
    
    std::cout << "\nTesting with 3x3 grid:" << std::endl;
    
    try {
        auto result = TerraScape::grid_to_mesh(
            width, height, data3x3.data(), 
            0.1f,  // error_threshold
            30,    // point_limit
            TerraScape::MeshRefineStrategy::SPARSE
        );
        
        std::cout << "3x3 SPARSE test succeeded" << std::endl;
        
    } catch (...) {
        std::cout << "3x3 SPARSE test failed" << std::endl;
    }
    
    return 0;
}