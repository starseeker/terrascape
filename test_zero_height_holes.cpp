#include <iostream>
#include <vector>
#include <cmath>
#include "TerraScape.hpp"

bool test_zero_height_holes() {
    std::cout << "=== Zero Height Holes Test ===" << std::endl;
    
    // Create terrain with a hole (zero region surrounded by elevated terrain)
    int width = 5, height = 5;
    std::vector<float> elevations = {
        1.0f, 1.0f, 1.0f, 1.0f, 1.0f,  // row 0: elevated boundary
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f,  // row 1: hole with elevated sides
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f,  // row 2: hole with elevated sides
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f,  // row 3: hole with elevated sides
        1.0f, 1.0f, 1.0f, 1.0f, 1.0f   // row 4: elevated boundary
    };
    
    float z_base = -0.5f; // Base below terrain
    
    std::cout << "Test terrain (5x5 grid with zero-height hole):" << std::endl;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::cout << elevations[y * width + x] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    
    // Generate surface mesh
    TerraScape::MeshResult surface_mesh = TerraScape::grid_to_mesh(width, height, elevations.data(), 0.1f);
    std::cout << "Surface mesh: " << surface_mesh.vertices.size() << " vertices, " 
              << surface_mesh.triangles.size() << " triangles" << std::endl;
    
    // Count vertices at different heights
    int zero_height_vertices = 0, elevated_vertices = 0;
    for (const auto& v : surface_mesh.vertices) {
        if (std::abs(v.z) < 0.1f) {
            zero_height_vertices++;
        } else {
            elevated_vertices++;
        }
    }
    
    std::cout << "Surface mesh analysis:" << std::endl;
    std::cout << "  Zero-height vertices: " << zero_height_vertices << std::endl;
    std::cout << "  Elevated vertices: " << elevated_vertices << std::endl;
    
    // Generate volumetric mesh
    TerraScape::MeshResult volumetric_mesh = TerraScape::grid_to_mesh_volumetric(width, height, elevations.data(), z_base, 0.1f);
    std::cout << "Volumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " 
              << volumetric_mesh.triangles.size() << " triangles" << std::endl;
    
    // Analyze volumetric mesh structure
    int surface_verts = 0, base_verts = 0;
    for (const auto& v : volumetric_mesh.vertices) {
        if (std::abs(v.z - z_base) < 0.01f) {
            base_verts++;
        } else {
            surface_verts++;
        }
    }
    
    std::cout << "Volumetric mesh analysis:" << std::endl;
    std::cout << "  Surface vertices: " << surface_verts << std::endl;
    std::cout << "  Base vertices: " << base_verts << std::endl;
    
    // Check if interior walls are needed for the zero-height hole
    // The problem: zero-height regions should have "floor" at the base level
    // and walls connecting the zero-height boundary to the base
    
    bool has_interior_walls = false;
    
    // Look for walls that might connect zero-height regions to the base
    for (size_t i = 0; i < volumetric_mesh.triangles.size(); ++i) {
        const auto& tri = volumetric_mesh.triangles[i];
        const auto& v0 = volumetric_mesh.vertices[tri.v0];
        const auto& v1 = volumetric_mesh.vertices[tri.v1];
        const auto& v2 = volumetric_mesh.vertices[tri.v2];
        
        // Check if this triangle connects a zero-height surface vertex to a base vertex
        bool has_zero_surface = false, has_base = false;
        
        if (std::abs(v0.z) < 0.1f && std::abs(v0.z - z_base) > 0.1f) has_zero_surface = true;
        if (std::abs(v1.z) < 0.1f && std::abs(v1.z - z_base) > 0.1f) has_zero_surface = true;
        if (std::abs(v2.z) < 0.1f && std::abs(v2.z - z_base) > 0.1f) has_zero_surface = true;
        
        if (std::abs(v0.z - z_base) < 0.1f) has_base = true;
        if (std::abs(v1.z - z_base) < 0.1f) has_base = true;
        if (std::abs(v2.z - z_base) < 0.1f) has_base = true;
        
        if (has_zero_surface && has_base) {
            has_interior_walls = true;
            std::cout << "Found interior wall triangle " << i << " connecting zero-height to base" << std::endl;
        }
    }
    
    std::cout << "Interior walls for zero-height regions: " << (has_interior_walls ? "✓ Present" : "❌ Missing") << std::endl;
    
    if (!has_interior_walls) {
        std::cout << "⚠  WARNING: Zero-height regions may create open holes in the volumetric mesh!" << std::endl;
        std::cout << "   This violates the closed manifold requirement." << std::endl;
    }
    
    return has_interior_walls;
}

int main() {
    bool result = test_zero_height_holes();
    if (result) {
        std::cout << "\n🎉 Zero-height hole handling is working correctly!" << std::endl;
    } else {
        std::cout << "\n❌ Zero-height hole handling needs improvement!" << std::endl;
    }
    return result ? 0 : 1;
}