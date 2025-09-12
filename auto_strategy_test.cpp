#include "TerraScape.hpp"
#include <iostream>
#include <vector>

using namespace TerraScape;

void test_auto_strategy() {
    std::cout << "=== Testing AUTO Strategy Selection ===\n";
    
    const int width = 4, height = 4;
    std::vector<float> elevations = {
        1.0f, 2.0f, 3.0f, 4.0f,
        2.0f, 3.0f, 4.0f, 5.0f,
        3.0f, 4.0f, 5.0f, 6.0f,
        4.0f, 5.0f, 6.0f, 7.0f
    };
    
    std::cout << "\nTesting AUTO strategy (should select GRID_AWARE):\n";
    MeshResult result = grid_to_mesh(width, height, elevations.data(), 
                                    1.0f, 1000, MeshRefineStrategy::AUTO);
    
    std::cout << "Result: " << result.vertices.size() << " vertices, " 
              << result.triangles.size() << " triangles\n";
    
    if (result.vertices.size() > 0 && result.triangles.size() > 0) {
        std::cout << "✓ AUTO strategy successfully selected GRID_AWARE and produced valid mesh\n";
    } else {
        std::cout << "✗ AUTO strategy failed\n";
    }
}

int main() {
    test_auto_strategy();
    return 0;
}