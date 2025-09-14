#include <iostream>
#include <vector>
#include <cmath>
#include "TerraScape.hpp"

bool test_detailed_interior_walls() {
    std::cout << "=== Detailed Interior Wall Test ===" << std::endl;
    
    // Create terrain with both zero and non-zero heights
    int width = 4, height = 4;
    std::vector<float> elevations = {
        1.0f, 1.0f, 1.0f, 1.0f,  // row 0: elevated
        1.0f, 0.0f, 0.0f, 1.0f,  // row 1: hole in center
        1.0f, 0.0f, 0.0f, 1.0f,  // row 2: hole in center
        1.0f, 1.0f, 1.0f, 1.0f   // row 3: elevated
    };
    
    float z_base = 0.0f; // Base at zero level
    
    // Generate volumetric mesh
    TerraScape::MeshResult volumetric_mesh = TerraScape::grid_to_mesh_volumetric(width, height, elevations.data(), z_base, 0.01f);
    
    std::cout << "Volumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " 
              << volumetric_mesh.triangles.size() << " triangles" << std::endl;
    
    // Show all vertices to understand the structure
    std::cout << "\nAll vertices:" << std::endl;
    for (size_t i = 0; i < volumetric_mesh.vertices.size(); ++i) {
        const auto& v = volumetric_mesh.vertices[i];
        std::cout << "  Vertex " << i << ": (" << v.x << ", " << v.y << ", " << v.z << ")";
        
        if (std::abs(v.z - z_base) < 0.01f) {
            std::cout << " [BASE/ZERO]";
        } else {
            std::cout << " [ELEVATED]";
        }
        
        // Check if this could be an interior zero-height vertex
        int grid_x = static_cast<int>(std::round(v.x));
        int grid_y = static_cast<int>(std::round(v.y));
        if (grid_x >= 1 && grid_x <= 2 && grid_y >= 1 && grid_y <= 2) {
            std::cout << " [INTERIOR]";
        }
        
        std::cout << std::endl;
    }
    
    // Count different types of triangles
    int surface_triangles = 0, base_triangles = 0, wall_triangles = 0, interior_wall_triangles = 0;
    
    for (size_t i = 0; i < volumetric_mesh.triangles.size(); ++i) {
        const auto& tri = volumetric_mesh.triangles[i];
        const auto& v0 = volumetric_mesh.vertices[tri.v0];
        const auto& v1 = volumetric_mesh.vertices[tri.v1];
        const auto& v2 = volumetric_mesh.vertices[tri.v2];
        
        // Check triangle type
        bool all_base = (std::abs(v0.z - z_base) < 0.01f) && 
                       (std::abs(v1.z - z_base) < 0.01f) && 
                       (std::abs(v2.z - z_base) < 0.01f);
        
        bool all_elevated = (v0.z > z_base + 0.01f) && 
                           (v1.z > z_base + 0.01f) && 
                           (v2.z > z_base + 0.01f);
        
        bool mixed = !all_base && !all_elevated;
        
        if (all_elevated) {
            surface_triangles++;
        } else if (all_base) {
            base_triangles++;
        } else if (mixed) {
            wall_triangles++;
            
            // Check if this is an interior wall triangle
            bool has_interior_vertex = false;
            for (int v_idx : {tri.v0, tri.v1, tri.v2}) {
                const auto& v = volumetric_mesh.vertices[v_idx];
                int grid_x = static_cast<int>(std::round(v.x));
                int grid_y = static_cast<int>(std::round(v.y));
                if (grid_x >= 1 && grid_x <= 2 && grid_y >= 1 && grid_y <= 2) {
                    has_interior_vertex = true;
                    break;
                }
            }
            
            if (has_interior_vertex) {
                interior_wall_triangles++;
                std::cout << "Interior wall triangle " << i << ": ";
                std::cout << "v" << tri.v0 << "(" << v0.x << "," << v0.y << "," << v0.z << ") ";
                std::cout << "v" << tri.v1 << "(" << v1.x << "," << v1.y << "," << v1.z << ") ";
                std::cout << "v" << tri.v2 << "(" << v2.x << "," << v2.y << "," << v2.z << ")" << std::endl;
            }
        }
    }
    
    std::cout << "\nTriangle summary:" << std::endl;
    std::cout << "  Surface triangles: " << surface_triangles << std::endl;
    std::cout << "  Base triangles: " << base_triangles << std::endl;
    std::cout << "  Wall triangles: " << wall_triangles << std::endl;
    std::cout << "  Interior wall triangles: " << interior_wall_triangles << std::endl;
    
    return interior_wall_triangles > 0;
}

int main() {
    bool result = test_detailed_interior_walls();
    if (result) {
        std::cout << "\n🎉 Interior wall generation is working!" << std::endl;
    } else {
        std::cout << "\n❌ Interior wall generation still needs work!" << std::endl;
    }
    return result ? 0 : 1;
}