#include <iostream>
#include <vector>
#include <set>
#include "TerraScape.hpp"

// Test to demonstrate the wall mesh issue and validate the fix
int main() {
    std::cout << "=== Volume Mesh Wall Issue Test ===" << std::endl;
    
    // Create a simple 4x4 grid where the surface mesh doesn't reach all edges
    // This simulates real terrain where the mesh is simplified at boundaries
    int width = 4, height = 4;
    std::vector<float> elevations = {
        0.0f, 0.0f, 0.0f, 0.0f,  // row 0: zero elevation at edges
        0.0f, 1.0f, 1.0f, 0.0f,  // row 1: elevated interior
        0.0f, 1.0f, 1.0f, 0.0f,  // row 2: elevated interior
        0.0f, 0.0f, 0.0f, 0.0f   // row 3: zero elevation at edges
    };
    
    std::cout << "Test case: 4x4 grid with zero elevation at edges:" << std::endl;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::cout << elevations[y * width + x] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    
    // Generate surface mesh with a lower error threshold to ensure we get interior vertices
    float z_base = 0.0f;
    TerraScape::MeshResult surface_mesh = TerraScape::grid_to_mesh(width, height, elevations.data(), 0.1f);
    std::cout << "Surface mesh: " << surface_mesh.vertices.size() << " vertices, " 
              << surface_mesh.triangles.size() << " triangles" << std::endl;
    
    std::cout << "Surface mesh vertices:" << std::endl;
    for (size_t i = 0; i < surface_mesh.vertices.size(); ++i) {
        const auto& v = surface_mesh.vertices[i];
        std::cout << "  Vertex " << i << ": (" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
    }
    std::cout << std::endl;
    
    // Generate volumetric mesh with current implementation
    TerraScape::MeshResult volumetric_mesh = TerraScape::grid_to_mesh_volumetric(width, height, elevations.data(), z_base, 0.1f);
    std::cout << "Current volumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " 
              << volumetric_mesh.triangles.size() << " triangles" << std::endl;
    
    // Analyze which vertices are at z_base (base vertices)
    int base_vertices = 0;
    int surface_vertices = 0;
    std::cout << "Volumetric mesh vertices:" << std::endl;
    for (size_t i = 0; i < volumetric_mesh.vertices.size(); ++i) {
        const auto& v = volumetric_mesh.vertices[i];
        std::cout << "  Vertex " << i << ": (" << v.x << ", " << v.y << ", " << v.z << ")";
        if (std::abs(v.z - z_base) < 0.001f) {
            base_vertices++;
            std::cout << " [BASE]";
        } else {
            surface_vertices++;
            std::cout << " [SURFACE]";
        }
        std::cout << std::endl;
    }
    
    std::cout << std::endl;
    
    std::cout << "Analysis:" << std::endl;
    std::cout << "  Surface vertices: " << surface_vertices << std::endl;
    std::cout << "  Base vertices: " << base_vertices << std::endl;
    std::cout << "  Expected base vertices for proper walls: " << 
        (4 + 4 + 2 + 2) << " (perimeter)" << std::endl; // 4 corners + edges
    
    // The issue: if boundary detection uses triangle edges, it may create walls
    // from interior vertices that happen to be on the triangulation boundary,
    // rather than from the actual data boundary
    
    return 0;
}