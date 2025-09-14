#include "TerraScape.hpp"
#include <iostream>
#include <vector>
#include <cmath>

using namespace TerraScape;

// Function to calculate triangle normal and check winding
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

bool test_triangle_winding_consistency() {
    std::cout << "Testing triangle winding consistency..." << std::endl;
    
    // Create a simple mountain-like heightfield
    std::vector<float> heights = {
        10, 20, 30, 20, 10,
        20, 40, 60, 40, 20,
        30, 60, 100, 60, 30,
        20, 40, 60, 40, 20,
        10, 20, 30, 20, 10
    };
    
    // Test with different mesh types
    std::vector<std::pair<std::string, MeshResult>> test_cases = {
        {"Surface mesh (small error)", grid_to_mesh(5, 5, heights.data(), 1.0f)},
        {"Surface mesh (large error)", grid_to_mesh(5, 5, heights.data(), 50.0f)},
        {"Volumetric mesh", grid_to_mesh_volumetric(5, 5, heights.data(), 0.0f, 1.0f)}
    };
    
    bool all_passed = true;
    
    for (const auto& [name, mesh] : test_cases) {
        int upward = 0, downward = 0, horizontal = 0, degenerate = 0;
        
        for (const auto& tri : mesh.triangles) {
            if (tri.v0 >= mesh.vertices.size() || tri.v1 >= mesh.vertices.size() || tri.v2 >= mesh.vertices.size()) {
                degenerate++;
                continue;
            }
            
            const auto& v0 = mesh.vertices[tri.v0];
            const auto& v1 = mesh.vertices[tri.v1];
            const auto& v2 = mesh.vertices[tri.v2];
            
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
            } else {
                horizontal++;
            }
        }
        
        std::cout << name << ": " << mesh.vertices.size() << " vertices, " << mesh.triangles.size() << " triangles" << std::endl;
        std::cout << "  Upward: " << upward << ", Downward: " << downward << ", Horizontal: " << horizontal << ", Degenerate: " << degenerate << std::endl;
        
        // For terrain meshes, we should have mostly upward or horizontal normals
        // Downward normals should be minimal (only for base of volumetric meshes)
        bool test_passed = true;
        
        if (name.find("Surface") != std::string::npos) {
            // Surface meshes should have mostly upward normals
            if (downward > upward) {
                std::cout << "  ❌ FAIL: Surface mesh has more downward than upward normals" << std::endl;
                test_passed = false;
            } else {
                std::cout << "  ✅ PASS: Surface mesh normals are correctly oriented" << std::endl;
            }
        } else if (name.find("Volumetric") != std::string::npos) {
            // Volumetric meshes can have both upward and downward normals, but should be balanced
            // or have slightly more upward (due to more surface area on top)
            if (downward > upward * 2) {
                std::cout << "  ❌ FAIL: Volumetric mesh has excessive downward normals" << std::endl;
                test_passed = false;
            } else {
                std::cout << "  ✅ PASS: Volumetric mesh normals are reasonably balanced" << std::endl;
            }
        }
        
        all_passed = all_passed && test_passed;
        std::cout << std::endl;
    }
    
    return all_passed;
}

int main() {
    bool passed = test_triangle_winding_consistency();
    
    if (passed) {
        std::cout << "🎉 All triangle winding tests PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "💥 Some triangle winding tests FAILED!" << std::endl;
        return 1;
    }
}