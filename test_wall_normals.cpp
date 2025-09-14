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

bool test_wall_normals() {
    std::cout << "=== Wall Normal Analysis Test ===" << std::endl;
    
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
    
    // Analyze triangles by type (surface, base, wall)
    int surface_triangles = 0, base_triangles = 0, wall_triangles = 0;
    
    for (size_t i = 0; i < volumetric_mesh.triangles.size(); ++i) {
        const auto& tri = volumetric_mesh.triangles[i];
        const auto& v0 = volumetric_mesh.vertices[tri.v0];
        const auto& v1 = volumetric_mesh.vertices[tri.v1];
        const auto& v2 = volumetric_mesh.vertices[tri.v2];
        
        // Check if triangle is on surface, base, or wall
        bool v0_surface = std::abs(v0.z - z_base) > 0.01f;
        bool v1_surface = std::abs(v1.z - z_base) > 0.01f;
        bool v2_surface = std::abs(v2.z - z_base) > 0.01f;
        
        int surface_verts = (v0_surface ? 1 : 0) + (v1_surface ? 1 : 0) + (v2_surface ? 1 : 0);
        
        Vec3 normal = compute_triangle_normal(v0, v1, v2);
        
        std::cout << "Triangle " << i << ": ";
        std::cout << "v" << tri.v0 << "(" << v0.x << "," << v0.y << "," << v0.z << ") ";
        std::cout << "v" << tri.v1 << "(" << v1.x << "," << v1.y << "," << v1.z << ") ";
        std::cout << "v" << tri.v2 << "(" << v2.x << "," << v2.y << "," << v2.z << ")";
        std::cout << " Normal: (" << normal.x << "," << normal.y << "," << normal.z << ")";
        
        if (surface_verts == 3) {
            surface_triangles++;
            std::cout << " [SURFACE]";
            if (normal.z < 0) std::cout << " ⚠ DOWNWARD NORMAL";
        } else if (surface_verts == 0) {
            base_triangles++;
            std::cout << " [BASE]";
            if (normal.z > 0) std::cout << " ⚠ UPWARD NORMAL";
        } else {
            wall_triangles++;
            std::cout << " [WALL]";
            // Wall normals should point outward (away from volume)
            float outward_component = std::sqrt(normal.x*normal.x + normal.y*normal.y);
            if (outward_component < 0.5f) std::cout << " ⚠ WEAK OUTWARD NORMAL";
            
            // Check if this is a potential "black wall" - inward facing normal
            if (normal.x*normal.x + normal.y*normal.y > 0.1f) {
                float wall_direction = std::atan2(normal.y, normal.x) * 180.0f / M_PI;
                std::cout << " (direction: " << wall_direction << "°)";
            }
        }
        std::cout << std::endl;
    }
    
    std::cout << std::endl;
    std::cout << "Triangle summary:" << std::endl;
    std::cout << "  Surface triangles: " << surface_triangles << std::endl;
    std::cout << "  Base triangles: " << base_triangles << std::endl;
    std::cout << "  Wall triangles: " << wall_triangles << std::endl;
    
    return true;
}

int main() {
    test_wall_normals();
    return 0;
}