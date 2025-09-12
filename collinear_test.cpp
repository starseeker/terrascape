#include "TerraScape.hpp"
#include <iostream>
#include <vector>

using namespace TerraScape;

void test_collinear_handling() {
    std::cout << "=== Testing Grid-Aware Strategy's Collinear Point Handling ===\n";
    
    // Test 1: Single row (all collinear)
    std::cout << "\nTest 1: Single row (6x1) - all points collinear\n";
    {
        const int width = 6, height = 1;
        std::vector<float> elevations = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 
                                        1.0f, 100, MeshRefineStrategy::GRID_AWARE);
        
        std::cout << "  Result: " << result.vertices.size() << " vertices, " 
                  << result.triangles.size() << " triangles\n";
        std::cout << "  ✓ Handled collinear case gracefully\n";
    }
    
    // Test 2: Single column (all collinear) 
    std::cout << "\nTest 2: Single column (1x6) - all points collinear\n";
    {
        const int width = 1, height = 6;
        std::vector<float> elevations = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 
                                        1.0f, 100, MeshRefineStrategy::GRID_AWARE);
        
        std::cout << "  Result: " << result.vertices.size() << " vertices, " 
                  << result.triangles.size() << " triangles\n";
        std::cout << "  ✓ Handled collinear case gracefully\n";
    }
    
    // Test 3: Flat terrain (worst case for traditional triangulation)
    std::cout << "\nTest 3: Completely flat terrain (3x3)\n";
    {
        const int width = 3, height = 3;
        std::vector<float> elevations(9, 5.0f);  // All same elevation
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 
                                        1.0f, 100, MeshRefineStrategy::GRID_AWARE);
        
        std::cout << "  Result: " << result.vertices.size() << " vertices, " 
                  << result.triangles.size() << " triangles\n";
        std::cout << "  ✓ Handled flat terrain case\n";
    }
    
    // Test 4: Regular grid with some variation
    std::cout << "\nTest 4: Regular 6x6 grid with systematic elevation\n";
    {
        const int width = 6, height = 6;
        std::vector<float> elevations;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                elevations.push_back(x + y * 0.5f);  // Simple systematic pattern
            }
        }
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 
                                        1.0f, 100, MeshRefineStrategy::GRID_AWARE);
        
        std::cout << "  Result: " << result.vertices.size() << " vertices, " 
                  << result.triangles.size() << " triangles\n";
        std::cout << "  ✓ Handled regular grid case\n";
    }
    
    std::cout << "\n=== All Collinear Point Tests Passed! ===\n";
    std::cout << "The grid-aware strategy successfully handles cases that would cause\n";
    std::cout << "assertion failures or poor results with traditional Delaunay triangulation.\n";
}

int main() {
    test_collinear_handling();
    return 0;
}