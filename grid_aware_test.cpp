#include "TerraScape.hpp"
#include <iostream>
#include <vector>

using namespace TerraScape;

void test_grid_aware_strategy() {
    std::cout << "=== Testing Grid-Aware Triangulation Strategy ===\n";
    
    // Test 1: Simple 4x4 grid
    std::cout << "\nTest 1: 4x4 grid with varied elevations\n";
    {
        const int width = 4, height = 4;
        std::vector<float> elevations = {
            1.0f, 2.0f, 3.0f, 4.0f,
            2.0f, 3.0f, 4.0f, 5.0f,
            3.0f, 4.0f, 5.0f, 6.0f,
            4.0f, 5.0f, 6.0f, 7.0f
        };
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 
                                        1.0f, 1000, MeshRefineStrategy::GRID_AWARE);
        
        std::cout << "  Grid-aware result: " << result.vertices.size() << " vertices, " 
                  << result.triangles.size() << " triangles\n";
        
        if (result.vertices.size() > 0 && result.triangles.size() > 0) {
            std::cout << "  ✓ Grid-aware triangulation successful\n";
        } else {
            std::cout << "  ✗ Grid-aware triangulation failed\n";
        }
    }
    
    // Test 2: Compare with SPARSE strategy
    std::cout << "\nTest 2: Comparison with SPARSE strategy\n";
    {
        const int width = 5, height = 5;
        std::vector<float> elevations;
        
        // Create some interesting terrain
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float elevation = sin(x * 0.5f) + cos(y * 0.5f) + x * 0.1f + y * 0.1f;
                elevations.push_back(elevation);
            }
        }
        
        MeshResult grid_aware_result = grid_to_mesh(width, height, elevations.data(), 
                                                   1.0f, 100, MeshRefineStrategy::GRID_AWARE);
        
        MeshResult sparse_result = grid_to_mesh(width, height, elevations.data(), 
                                               1.0f, 100, MeshRefineStrategy::SPARSE);
        
        std::cout << "  Grid-aware: " << grid_aware_result.vertices.size() << " vertices, " 
                  << grid_aware_result.triangles.size() << " triangles\n";
        std::cout << "  SPARSE:     " << sparse_result.vertices.size() << " vertices, " 
                  << sparse_result.triangles.size() << " triangles\n";
        
        if (grid_aware_result.vertices.size() > 0 && sparse_result.vertices.size() > 0) {
            std::cout << "  ✓ Both strategies produced valid meshes\n";
        }
    }
    
    // Test 3: Degenerate case (collinear points)
    std::cout << "\nTest 3: Collinear points (grid-aware should handle better)\n";
    {
        const int width = 6, height = 1;  // Single row - all collinear
        std::vector<float> elevations = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
        
        MeshResult grid_aware_result = grid_to_mesh(width, height, elevations.data(), 
                                                   1.0f, 100, MeshRefineStrategy::GRID_AWARE);
        
        std::cout << "  Grid-aware (collinear): " << grid_aware_result.vertices.size() << " vertices, " 
                  << grid_aware_result.triangles.size() << " triangles\n";
        
        if (grid_aware_result.vertices.size() > 0) {
            std::cout << "  ✓ Grid-aware handled collinear case\n";
        }
    }
    
    std::cout << "\n=== Grid-Aware Strategy Tests Complete ===\n";
}

int main() {
    test_grid_aware_strategy();
    return 0;
}