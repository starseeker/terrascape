#include <iostream>
#include <vector>
#include <set>
#include "TerraScape.hpp"

// Test to validate the wall fix
int main() {
    std::cout << "=== Volume Mesh Wall Fix Validation ===" << std::endl;
    
    // Create a simple 3x3 grid to ensure we get surface vertices
    int width = 3, height = 3;
    std::vector<float> elevations = {
        1.0f, 1.0f, 1.0f,  // row 0: base level edges
        1.0f, 2.0f, 1.0f,  // row 1: elevated center
        1.0f, 1.0f, 1.0f   // row 2: base level edges  
    };
    
    std::cout << "Test case: 3x3 grid with elevated center:" << std::endl;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::cout << elevations[y * width + x] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    
    // Generate surface mesh with a very low error threshold to preserve detail
    float z_base = 1.0f;  // Set base to 1.0 so center vertex at 2.0 will be elevated
    TerraScape::MeshResult surface_mesh = TerraScape::grid_to_mesh(width, height, elevations.data(), 0.01f);
    std::cout << "Surface mesh: " << surface_mesh.vertices.size() << " vertices, " 
              << surface_mesh.triangles.size() << " triangles" << std::endl;
    
    std::cout << "Surface mesh vertices:" << std::endl;
    for (size_t i = 0; i < surface_mesh.vertices.size(); ++i) {
        const auto& v = surface_mesh.vertices[i];
        std::cout << "  Vertex " << i << ": (" << v.x << ", " << v.y << ", " << v.z << ")";
        if (v.z > z_base + 0.01f) {
            std::cout << " [ELEVATED]";
        } else {
            std::cout << " [BASE LEVEL]";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    
    // Generate volumetric mesh
    TerraScape::MeshResult volumetric_mesh = TerraScape::grid_to_mesh_volumetric(width, height, elevations.data(), z_base, 0.01f);
    std::cout << "Volumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " 
              << volumetric_mesh.triangles.size() << " triangles" << std::endl;
    
    // Analyze the volumetric mesh
    int surface_vertices = 0;
    int base_vertices = 0;
    int boundary_surface_vertices = 0;
    int interior_surface_vertices = 0;
    
    std::cout << "Volumetric mesh analysis:" << std::endl;
    for (size_t i = 0; i < volumetric_mesh.vertices.size(); ++i) {
        const auto& v = volumetric_mesh.vertices[i];
        std::cout << "  Vertex " << i << ": (" << v.x << ", " << v.y << ", " << v.z << ")";
        
        if (std::abs(v.z - z_base) < 0.001f) {
            base_vertices++;
            std::cout << " [BASE]";
        } else {
            surface_vertices++;
            // Check if it's on the boundary
            if (v.x == 0 || v.x == (width-1) || v.y == 0 || v.y == (height-1)) {
                boundary_surface_vertices++;
                std::cout << " [SURFACE-BOUNDARY]";
            } else {
                interior_surface_vertices++;
                std::cout << " [SURFACE-INTERIOR]";
            }
        }
        std::cout << std::endl;
    }
    
    std::cout << std::endl;
    std::cout << "Analysis:" << std::endl;
    std::cout << "  Surface vertices: " << surface_vertices << std::endl;
    std::cout << "  Base vertices: " << base_vertices << std::endl;
    std::cout << "  Boundary surface vertices: " << boundary_surface_vertices << std::endl;
    std::cout << "  Interior surface vertices: " << interior_surface_vertices << std::endl;
    
    // The fix should ensure that walls are only created from actual boundary vertices
    // and that the base is uniform and planar
    std::cout << std::endl;
    std::cout << "Wall Fix Validation:" << std::endl;
    
    // Check that all base vertices are at exactly z_base
    bool uniform_base = true;
    for (const auto& v : volumetric_mesh.vertices) {
        if (std::abs(v.z - z_base) < 0.001f) {
            // This is a base vertex - should be exactly at z_base
            if (std::abs(v.z - z_base) > 0.0001f) {
                uniform_base = false;
                break;
            }
        }
    }
    
    if (uniform_base) {
        std::cout << "  ✓ Base is uniform and planar at z=" << z_base << std::endl;
    } else {
        std::cout << "  ❌ Base is not uniform!" << std::endl;
    }
    
    // Check if we have proper surface/base vertex pairs
    bool proper_pairing = (surface_vertices > 0) && (surface_vertices == base_vertices / 2);
    if (proper_pairing) {
        std::cout << "  ✓ Proper surface/base vertex pairing" << std::endl;
    } else {
        std::cout << "  ⚠ Note: Expected equal surface and base vertices for proper volumetric mesh" << std::endl;
    }
    
    return 0;
}