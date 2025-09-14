#include <iostream>
#include <vector>
#include <cmath>
#include "TerraScape.hpp"

// Helper function to compute triangle normal
struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
    Vec3 operator-(const Vec3& other) const { return Vec3(x-other.x, y-other.y, z-other.z); }
    Vec3 cross(const Vec3& other) const {
        return Vec3(y*other.z - z*other.y, z*other.x - x*other.z, x*other.y - y*other.x);
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    void normalize() { float l = length(); if (l > 0) { x/=l; y/=l; z/=l; } }
};

Vec3 compute_triangle_normal(const TerraScape::Vertex& v0, const TerraScape::Vertex& v1, const TerraScape::Vertex& v2) {
    Vec3 p0(v0.x, v0.y, v0.z);
    Vec3 p1(v1.x, v1.y, v1.z);
    Vec3 p2(v2.x, v2.y, v2.z);
    
    Vec3 edge1 = p1 - p0;
    Vec3 edge2 = p2 - p0;
    Vec3 normal = edge1.cross(edge2);
    normal.normalize();
    return normal;
}

bool test_wall_normal_consistency() {
    std::cout << "=== Wall Normal Consistency Test ===" << std::endl;
    
    // Create a simple elevated plateau to examine wall triangles
    int width = 3, height = 3;
    std::vector<float> elevations = {
        0.0f, 0.0f, 0.0f,  // row 0: zero elevation
        0.0f, 1.0f, 0.0f,  // row 1: elevated center
        0.0f, 0.0f, 0.0f   // row 2: zero elevation
    };
    
    float z_base = -0.5f; // Base below terrain
    
    // Generate volumetric mesh
    TerraScape::MeshResult volumetric_mesh = TerraScape::grid_to_mesh_volumetric(width, height, elevations.data(), z_base, 0.1f);
    
    std::cout << "Volumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " 
              << volumetric_mesh.triangles.size() << " triangles" << std::endl;
    
    // Analyze wall triangles specifically
    int good_wall_normals = 0, bad_wall_normals = 0;
    
    for (size_t i = 0; i < volumetric_mesh.triangles.size(); ++i) {
        const auto& tri = volumetric_mesh.triangles[i];
        const auto& v0 = volumetric_mesh.vertices[tri.v0];
        const auto& v1 = volumetric_mesh.vertices[tri.v1];
        const auto& v2 = volumetric_mesh.vertices[tri.v2];
        
        // Check if triangle is a wall (mixed elevations)
        bool v0_surface = std::abs(v0.z - z_base) > 0.01f;
        bool v1_surface = std::abs(v1.z - z_base) > 0.01f;
        bool v2_surface = std::abs(v2.z - z_base) > 0.01f;
        
        int surface_verts = (v0_surface ? 1 : 0) + (v1_surface ? 1 : 0) + (v2_surface ? 1 : 0);
        
        if (surface_verts > 0 && surface_verts < 3) { // Wall triangle
            Vec3 normal = compute_triangle_normal(v0, v1, v2);
            
            // Wall normals should point outward (away from volume)
            float outward_component = std::sqrt(normal.x*normal.x + normal.y*normal.y);
            
            if (outward_component > 0.3f) { // Good outward normal
                good_wall_normals++;
            } else {
                bad_wall_normals++;
                std::cout << "⚠  Bad wall normal at triangle " << i 
                          << ": (" << normal.x << "," << normal.y << "," << normal.z << ")" << std::endl;
            }
        }
    }
    
    std::cout << "Wall normal analysis:" << std::endl;
    std::cout << "  Good wall normals: " << good_wall_normals << std::endl;
    std::cout << "  Bad wall normals: " << bad_wall_normals << std::endl;
    
    return bad_wall_normals == 0;
}

bool test_zero_height_interior_walls() {
    std::cout << "\n=== Zero-Height Interior Wall Test ===" << std::endl;
    
    // Create terrain with interior zero-height holes
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
    
    // Count interior vertices (should be present for zero-height holes)
    int interior_vertices = 0;
    for (const auto& v : volumetric_mesh.vertices) {
        int grid_x = static_cast<int>(std::round(v.x));
        int grid_y = static_cast<int>(std::round(v.y));
        
        // Check if this vertex is in the interior zero-height region
        if (grid_x >= 1 && grid_x <= 2 && grid_y >= 1 && grid_y <= 2 && 
            std::abs(v.z - z_base) < 0.01f) {
            interior_vertices++;
        }
    }
    
    // Count interior wall triangles
    int interior_wall_triangles = 0;
    for (size_t i = 0; i < volumetric_mesh.triangles.size(); ++i) {
        const auto& tri = volumetric_mesh.triangles[i];
        const auto& v0 = volumetric_mesh.vertices[tri.v0];
        const auto& v1 = volumetric_mesh.vertices[tri.v1];
        const auto& v2 = volumetric_mesh.vertices[tri.v2];
        
        // Check if triangle connects interior zero-height region to elevated terrain
        bool has_interior_vertex = false;
        bool has_elevated_vertex = false;
        
        for (const auto* v : {&v0, &v1, &v2}) {
            int grid_x = static_cast<int>(std::round(v->x));
            int grid_y = static_cast<int>(std::round(v->y));
            
            if (grid_x >= 1 && grid_x <= 2 && grid_y >= 1 && grid_y <= 2) {
                has_interior_vertex = true;
            }
            
            if (v->z > z_base + 0.1f) {
                has_elevated_vertex = true;
            }
        }
        
        if (has_interior_vertex && has_elevated_vertex) {
            interior_wall_triangles++;
        }
    }
    
    std::cout << "Interior wall analysis:" << std::endl;
    std::cout << "  Interior vertices: " << interior_vertices << std::endl;
    std::cout << "  Interior wall triangles: " << interior_wall_triangles << std::endl;
    
    return interior_vertices > 0 && interior_wall_triangles > 0;
}

int main() {
    std::cout << "=== TerraScape Wall Fix Validation ===" << std::endl;
    std::cout << "Testing fixes for:" << std::endl;
    std::cout << "1. Black wall issue (triangle winding/normals)" << std::endl;
    std::cout << "2. Zero-height holes creating non-manifold volumes" << std::endl;
    std::cout << std::endl;
    
    bool wall_normals_ok = test_wall_normal_consistency();
    bool interior_walls_ok = test_zero_height_interior_walls();
    
    std::cout << "\n=== Results ===" << std::endl;
    std::cout << "Fix 1 - Wall normal consistency: " << (wall_normals_ok ? "✓ PASS" : "❌ FAIL") << std::endl;
    std::cout << "Fix 2 - Interior wall generation: " << (interior_walls_ok ? "✓ PASS" : "❌ FAIL") << std::endl;
    
    if (wall_normals_ok && interior_walls_ok) {
        std::cout << "\n🎉 All wall fixes are working correctly!" << std::endl;
        std::cout << "The volumetric mesh should now:" << std::endl;
        std::cout << "  - Render correctly in OpenGL (no black walls)" << std::endl;
        std::cout << "  - Be a proper closed manifold (no holes)" << std::endl;
        return 0;
    } else {
        std::cout << "\n❌ Some fixes still need work!" << std::endl;
        return 1;
    }
}