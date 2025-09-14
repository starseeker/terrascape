#include <iostream>
#include <vector>
#include <cmath>
#include "TerraScape.hpp"

bool test_mixed_elevation_holes() {
    std::cout << "=== Mixed Elevation Holes Test ===" << std::endl;
    
    // Create terrain with both zero and non-zero heights in the surface mesh
    int width = 4, height = 4;
    std::vector<float> elevations = {
        1.0f, 1.0f, 1.0f, 1.0f,  // row 0: elevated
        1.0f, 0.0f, 0.0f, 1.0f,  // row 1: hole in center
        1.0f, 0.0f, 0.0f, 1.0f,  // row 2: hole in center
        1.0f, 1.0f, 1.0f, 1.0f   // row 3: elevated
    };
    
    float z_base = 0.0f; // Base at zero level
    
    std::cout << "Test terrain (4x4 grid with mixed elevations):" << std::endl;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::cout << elevations[y * width + x] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    
    // Generate surface mesh with low error threshold to preserve details
    TerraScape::MeshResult surface_mesh = TerraScape::grid_to_mesh(width, height, elevations.data(), 0.01f);
    std::cout << "Surface mesh: " << surface_mesh.vertices.size() << " vertices, " 
              << surface_mesh.triangles.size() << " triangles" << std::endl;
    
    // Show all surface vertices
    std::cout << "Surface mesh vertices:" << std::endl;
    for (size_t i = 0; i < surface_mesh.vertices.size(); ++i) {
        const auto& v = surface_mesh.vertices[i];
        std::cout << "  Vertex " << i << ": (" << v.x << ", " << v.y << ", " << v.z << ")";
        if (std::abs(v.z) < 0.1f) {
            std::cout << " [ZERO-HEIGHT]";
        } else {
            std::cout << " [ELEVATED]";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    
    // Generate volumetric mesh
    TerraScape::MeshResult volumetric_mesh = TerraScape::grid_to_mesh_volumetric(width, height, elevations.data(), z_base, 0.01f);
    std::cout << "Volumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " 
              << volumetric_mesh.triangles.size() << " triangles" << std::endl;
    
    // Count interior walls
    int interior_wall_count = 0;
    for (size_t i = 0; i < volumetric_mesh.triangles.size(); ++i) {
        const auto& tri = volumetric_mesh.triangles[i];
        const auto& v0 = volumetric_mesh.vertices[tri.v0];
        const auto& v1 = volumetric_mesh.vertices[tri.v1];
        const auto& v2 = volumetric_mesh.vertices[tri.v2];
        
        // Check if triangle connects zero-height surface to base
        bool has_zero_surface = false, has_base = false;
        
        if (std::abs(v0.z) < 0.1f && std::abs(v0.z - z_base) > 0.1f) has_zero_surface = true;
        if (std::abs(v1.z) < 0.1f && std::abs(v1.z - z_base) > 0.1f) has_zero_surface = true;
        if (std::abs(v2.z) < 0.1f && std::abs(v2.z - z_base) > 0.1f) has_zero_surface = true;
        
        if (std::abs(v0.z - z_base) < 0.1f) has_base = true;
        if (std::abs(v1.z - z_base) < 0.1f) has_base = true;
        if (std::abs(v2.z - z_base) < 0.1f) has_base = true;
        
        if (has_zero_surface && has_base) {
            interior_wall_count++;
            std::cout << "Interior wall triangle " << i << ": connects zero-height to base" << std::endl;
        }
    }
    
    std::cout << "Interior wall triangles found: " << interior_wall_count << std::endl;
    return interior_wall_count > 0;
}

int main() {
    bool result = test_mixed_elevation_holes();
    if (result) {
        std::cout << "\n🎉 Interior wall generation is working!" << std::endl;
    } else {
        std::cout << "\n❌ Interior wall generation still needs work!" << std::endl;
    }
    return result ? 0 : 1;
}