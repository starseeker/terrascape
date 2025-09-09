#include <iostream>
#include <vector>
#include <cassert>
#include "TerraScape.hpp"

// Create a test case with 12+ points that triggers the SPARSE strategy segfault
std::vector<float> create_complex_test_data() {
    // Create a 4x4 grid with complex elevation pattern
    std::vector<float> data = {
        1.0f, 1.1f, 1.2f, 1.0f,
        1.1f, 3.0f, 2.5f, 1.2f,
        1.2f, 2.5f, 3.2f, 1.1f,
        1.0f, 1.2f, 1.1f, 1.0f
    };
    return data;
}

int main() {
    std::cout << "=== Testing SPARSE Strategy Segfault Issue ===" << std::endl;
    
    auto data = create_complex_test_data();
    int width = 4, height = 4;
    
    std::cout << "Testing SPARSE strategy with " << (width * height) << " grid points:" << std::endl;
    
    try {
        // Force SPARSE strategy specifically
        auto result = TerraScape::grid_to_mesh(
            width, height, data.data(), 
            0.1f,  // error_threshold
            50,    // point_limit (high to allow many points)
            TerraScape::MeshRefineStrategy::SPARSE
        );
        
        std::cout << "SPARSE strategy succeeded:" << std::endl;
        std::cout << "  Vertices: " << result.vertices.size() << std::endl;
        std::cout << "  Triangles: " << result.triangles.size() << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "SPARSE strategy failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cout << "SPARSE strategy failed with unknown exception/segfault" << std::endl;
        return 1;
    }
    
    std::cout << "\n=== Comparing with AUTO strategy ===" << std::endl;
    
    try {
        // Compare with AUTO strategy (which should work)
        auto result_auto = TerraScape::grid_to_mesh(
            width, height, data.data(), 
            0.1f,  // error_threshold
            50,    // point_limit
            TerraScape::MeshRefineStrategy::AUTO
        );
        
        std::cout << "AUTO strategy succeeded:" << std::endl;
        std::cout << "  Vertices: " << result_auto.vertices.size() << std::endl;
        std::cout << "  Triangles: " << result_auto.triangles.size() << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "AUTO strategy failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cout << "AUTO strategy failed with unknown exception" << std::endl;
        return 1;
    }
    
    return 0;
}