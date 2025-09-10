#include <iostream>
#include <vector>
#include "TerraScape.hpp"

int main() {
    std::cout << "=== TerraScape Separated Volumetric Mesh Example ===\n\n";
    
    // Create a simple 3x3 terrain with mixed heights
    const int width = 3, height = 3;
    std::vector<float> elevations = {
        0.5f, 2.0f, 0.8f,  // Row 0: mix of high and low
        1.8f, 2.5f, 1.2f,  // Row 1: mix of high and low  
        0.6f, 2.2f, 0.9f   // Row 2: mix of high and low
    };
    
    float z_base = 1.5f;  // Base level
    
    std::cout << "Terrain elevations (3x3 grid):\n";
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float elev = elevations[y * width + x];
            char marker = (elev > z_base) ? '+' : (elev < z_base) ? '-' : '=';
            std::cout << elev << marker << "  ";
        }
        std::cout << "\n";
    }
    std::cout << "\nBase level: " << z_base << "\n";
    std::cout << "(+ = above base, - = below base, = = at base)\n\n";
    
    // Generate separated volumetric meshes
    auto result = TerraScape::grid_to_mesh_volumetric_separated(width, height, elevations.data(), z_base);
    
    std::cout << "Results:\n";
    std::cout << "  Has positive volume: " << (result.has_positive_volume ? "YES" : "NO") << "\n";
    std::cout << "  Has negative volume: " << (result.has_negative_volume ? "YES" : "NO") << "\n";
    
    if (result.has_positive_volume) {
        std::cout << "  Positive volume mesh: " << result.positive_volume.vertices.size() 
                  << " vertices, " << result.positive_volume.triangles.size() << " triangles\n";
        std::cout << "    - Represents terrain areas above base level\n";
        std::cout << "    - Normal outward-facing normals\n";
    }
    
    if (result.has_negative_volume) {
        std::cout << "  Negative volume mesh: " << result.negative_volume.vertices.size() 
                  << " vertices, " << result.negative_volume.triangles.size() << " triangles\n";
        std::cout << "    - Represents terrain areas below base level\n"; 
        std::cout << "    - Reversed normals for inverted volume\n";
    }
    
    std::cout << "\nKey features:\n";
    std::cout << "- Avoids degenerate meshes by separating positive/negative volumes\n";
    std::cout << "- Skips triangles at exactly the base level to prevent zero-height volumes\n";
    std::cout << "- Provides separate meshes for different processing (e.g., cut/fill operations)\n";
    
    return 0;
}