#include <iostream>
#include <vector>
#include "TerraScape.hpp"

int main() {
    std::cout << "=== Volume Calculation Analysis ===" << std::endl;
    
    // Create a simple test case: 3x3 grid with known elevation
    int width = 3, height = 3;
    std::vector<float> elevations = {
        1.0f, 2.0f, 1.0f,  // row 0
        2.0f, 3.0f, 2.0f,  // row 1  
        1.0f, 2.0f, 1.0f   // row 2
    };
    
    std::cout << "Test case: 3x3 grid with elevations:" << std::endl;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::cout << elevations[y * width + x] << " ";
        }
        std::cout << std::endl;
    }
    
    // Calculate expected volume manually
    // Each cell has area 1x1, heights above min elevation (1.0)
    double expected_volume = 0.0 + 1.0 + 0.0 +  // row 0: heights above 1.0
                            1.0 + 2.0 + 1.0 +  // row 1
                            0.0 + 1.0 + 0.0;   // row 2
    std::cout << "\nExpected volume (manual calculation): " << expected_volume << std::endl;
    
    // Generate surface mesh
    TerraScape::MeshResult surface_mesh = TerraScape::grid_to_mesh(width, height, elevations.data(), 0.1f);
    std::cout << "Surface mesh: " << surface_mesh.vertices.size() << " vertices, " 
              << surface_mesh.triangles.size() << " triangles" << std::endl;
    
    // Generate volumetric mesh
    TerraScape::MeshResult volumetric_mesh = TerraScape::grid_to_mesh_volumetric(width, height, elevations.data(), 1.0f, 0.1f);
    std::cout << "Volumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " 
              << volumetric_mesh.triangles.size() << " triangles" << std::endl;
    
    // Test volume calculations
    double surface_volume = TerraScape::calculate_mesh_volume(surface_mesh, 1.0f);
    double volumetric_volume = TerraScape::calculate_mesh_volume(volumetric_mesh, 1.0f);
    double heightfield_volume = TerraScape::calculate_heightfield_volume(width, height, elevations.data(), 1.0f);
    
    std::cout << "\nVolume calculations:" << std::endl;
    std::cout << "Heightfield volume: " << heightfield_volume << std::endl;
    std::cout << "Surface mesh volume: " << surface_volume << std::endl;
    std::cout << "Volumetric mesh volume: " << volumetric_volume << std::endl;
    std::cout << "Expected volume: " << expected_volume << std::endl;
    
    std::cout << "\nVolume ratios:" << std::endl;
    std::cout << "Surface/Expected: " << (surface_volume / expected_volume) << std::endl;
    std::cout << "Volumetric/Expected: " << (volumetric_volume / expected_volume) << std::endl;
    std::cout << "Heightfield/Expected: " << (heightfield_volume / expected_volume) << std::endl;
    
    return 0;
}