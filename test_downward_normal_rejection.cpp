#include "TerraScape.hpp"
#include <iostream>
#include <vector>
#include <cmath>

using namespace TerraScape;

// Helper function to compute triangle normal and check winding
struct Vec3 {
    float x, y, z;
    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }
    
    Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }
    
    float length() const {
        return std::sqrt(x*x + y*y + z*z);
    }
    
    Vec3 normalize() const {
        float len = length();
        return len > 1e-6 ? Vec3(x/len, y/len, z/len) : Vec3(0, 0, 1);
    }
};

bool test_downward_normal_rejection() {
    std::cout << "Testing downward normal rejection for surface triangles..." << std::endl;
    
    // Create a heightfield with significant variation that would previously create downward normals
    std::vector<float> heights = {
        0, 1, 0,
        1, 2, 1,  
        0, 1, 0
    };
    
    // Generate surface mesh
    auto surface_mesh = grid_to_mesh(3, 3, heights.data(), 0.1f);
    
    std::cout << "Surface mesh: " << surface_mesh.vertices.size() << " vertices, " 
              << surface_mesh.triangles.size() << " triangles" << std::endl;
    
    // Analyze triangle normals
    int upward = 0, downward = 0, horizontal = 0, degenerate = 0;
    
    for (const auto& tri : surface_mesh.triangles) {
        if (tri.v0 >= surface_mesh.vertices.size() || tri.v1 >= surface_mesh.vertices.size() || tri.v2 >= surface_mesh.vertices.size()) {
            degenerate++;
            continue;
        }
        
        const auto& v0 = surface_mesh.vertices[tri.v0];
        const auto& v1 = surface_mesh.vertices[tri.v1];
        const auto& v2 = surface_mesh.vertices[tri.v2];
        
        Vec3 edge1(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        Vec3 edge2(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        Vec3 normal = edge1.cross(edge2);
        
        if (normal.length() < 1e-6) {
            degenerate++;
            continue;
        }
        
        normal = normal.normalize();
        
        if (normal.z > 0.3) {
            upward++;
        } else if (normal.z < -0.3) {
            downward++;
            std::cout << "  Found downward normal: (" << normal.x << "," << normal.y << "," << normal.z << ")" << std::endl;
        } else {
            horizontal++;
        }
    }
    
    std::cout << "  Surface triangles: Upward: " << upward << ", Downward: " << downward 
              << ", Horizontal: " << horizontal << ", Degenerate: " << degenerate << std::endl;
    
    // Test should pass if there are no downward normals
    bool test_passed = (downward == 0);
    
    if (test_passed) {
        std::cout << "  ✅ PASS: No surface triangles have downward normals" << std::endl;
    } else {
        std::cout << "  ❌ FAIL: Found " << downward << " surface triangles with downward normals" << std::endl;
    }
    
    return test_passed;
}

bool test_volumetric_mesh_still_works() {
    std::cout << "\nTesting that volumetric meshes still work correctly..." << std::endl;
    
    // Simple heightfield
    std::vector<float> heights = {
        1, 2, 1,
        2, 3, 2,  
        1, 2, 1
    };
    
    // Generate volumetric mesh  
    auto volumetric_mesh = grid_to_mesh_volumetric(3, 3, heights.data(), 0.0f, 0.1f);
    
    std::cout << "Volumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " 
              << volumetric_mesh.triangles.size() << " triangles" << std::endl;
    
    // Count surface vs base triangles
    int surface_triangles = 0, base_triangles = 0, wall_triangles = 0;
    int upward_surface = 0, downward_surface = 0;
    int upward_base = 0, downward_base = 0;
    
    float z_base = 0.0f;
    
    for (const auto& tri : volumetric_mesh.triangles) {
        const auto& v0 = volumetric_mesh.vertices[tri.v0];
        const auto& v1 = volumetric_mesh.vertices[tri.v1];
        const auto& v2 = volumetric_mesh.vertices[tri.v2];
        
        // Check if triangle is on surface, base, or wall
        bool v0_surface = std::abs(v0.z - z_base) > 0.01f;
        bool v1_surface = std::abs(v1.z - z_base) > 0.01f;
        bool v2_surface = std::abs(v2.z - z_base) > 0.01f;
        
        int surface_verts = (v0_surface ? 1 : 0) + (v1_surface ? 1 : 0) + (v2_surface ? 1 : 0);
        
        Vec3 edge1(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        Vec3 edge2(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        Vec3 normal = edge1.cross(edge2);
        
        if (normal.length() > 1e-6) {
            normal = normal.normalize();
            
            if (surface_verts == 3) {
                surface_triangles++;
                if (normal.z > 0) upward_surface++;
                else if (normal.z < 0) downward_surface++;
            } else if (surface_verts == 0) {
                base_triangles++;
                if (normal.z > 0) upward_base++;
                else if (normal.z < 0) downward_base++;
            } else {
                wall_triangles++;
            }
        }
    }
    
    std::cout << "  Surface triangles: " << surface_triangles << " (upward: " << upward_surface 
              << ", downward: " << downward_surface << ")" << std::endl;
    std::cout << "  Base triangles: " << base_triangles << " (upward: " << upward_base 
              << ", downward: " << downward_base << ")" << std::endl;
    std::cout << "  Wall triangles: " << wall_triangles << std::endl;
    
    // Test passes if:
    // 1. Surface triangles have no downward normals
    // 2. Base triangles have downward normals (as they should)
    // 3. We have the expected triangle types
    bool test_passed = (downward_surface == 0) && (downward_base > 0) && 
                       (surface_triangles > 0) && (base_triangles > 0);
    
    if (test_passed) {
        std::cout << "  ✅ PASS: Volumetric mesh has correct triangle orientations" << std::endl;
    } else {
        std::cout << "  ❌ FAIL: Volumetric mesh orientation is incorrect" << std::endl;
    }
    
    return test_passed;
}

int main() {
    std::cout << "=== Testing Downward Normal Rejection ===" << std::endl;
    
    bool surface_test = test_downward_normal_rejection();
    bool volumetric_test = test_volumetric_mesh_still_works();
    
    if (surface_test && volumetric_test) {
        std::cout << "\n🎉 All downward normal rejection tests PASSED!" << std::endl;
        std::cout << "✓ Surface triangles properly reject downward normals" << std::endl;
        std::cout << "✓ Volumetric meshes still work correctly" << std::endl;
        return 0;
    } else {
        std::cout << "\n💥 Some downward normal rejection tests FAILED!" << std::endl;
        return 1;
    }
}