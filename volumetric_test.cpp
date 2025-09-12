#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <set>
#include <fstream>
#include <unordered_set>
#include <map>
#include "TerraScape.hpp"

namespace {

// Helper function to write OBJ file
void writeOBJ(const std::string& filename, const TerraScape::MeshResult& mesh) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open " << filename << " for writing\n";
        return;
    }
    
    // Write vertices
    for (const auto& v : mesh.vertices) {
        file << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    
    // Write faces (OBJ uses 1-based indexing)
    for (const auto& t : mesh.triangles) {
        file << "f " << (t.v0 + 1) << " " << (t.v1 + 1) << " " << (t.v2 + 1) << "\n";
    }
    
    file.close();
    std::cout << "Wrote mesh to " << filename << std::endl;
}

// Test if mesh is manifold (each edge is shared by exactly 2 triangles)
bool isManifold(const TerraScape::MeshResult& mesh) {
    std::map<TerraScape::Edge, int> edge_count;
    
    // Count edge occurrences
    for (const auto& tri : mesh.triangles) {
        edge_count[TerraScape::Edge(tri.v0, tri.v1)]++;
        edge_count[TerraScape::Edge(tri.v1, tri.v2)]++;
        edge_count[TerraScape::Edge(tri.v2, tri.v0)]++;
    }
    
    // Check that each edge appears exactly twice (manifold condition)
    for (const auto& [edge, count] : edge_count) {
        if (count != 2) {
            std::cout << "Non-manifold edge: " << edge.v0 << "-" << edge.v1 
                      << " appears " << count << " times\n";
            return false;
        }
    }
    
    return true;
}

// Test if the mesh is closed (watertight)
bool isWatertight(const TerraScape::MeshResult& mesh) {
    return isManifold(mesh); // For this test, manifold implies watertight
}

// Test if vertices are correctly placed
bool testVolumetricStructure(const TerraScape::MeshResult& surface_mesh, 
                           const TerraScape::MeshResult& volumetric_mesh, 
                           float z_base) {
    // Check that we have exactly double the vertices
    if (volumetric_mesh.vertices.size() != 2 * surface_mesh.vertices.size()) {
        std::cout << "Error: Expected " << (2 * surface_mesh.vertices.size()) 
                  << " vertices, got " << volumetric_mesh.vertices.size() << std::endl;
        return false;
    }
    
    // Check that surface vertices are preserved
    for (size_t i = 0; i < surface_mesh.vertices.size(); ++i) {
        const auto& surface_v = surface_mesh.vertices[i];
        const auto& volumetric_v = volumetric_mesh.vertices[i];
        
        if (std::abs(surface_v.x - volumetric_v.x) > 1e-6f ||
            std::abs(surface_v.y - volumetric_v.y) > 1e-6f ||
            std::abs(surface_v.z - volumetric_v.z) > 1e-6f) {
            std::cout << "Error: Surface vertex " << i << " not preserved\n";
            return false;
        }
    }
    
    // Check that base vertices have correct z-coordinate
    for (size_t i = surface_mesh.vertices.size(); i < volumetric_mesh.vertices.size(); ++i) {
        const auto& base_v = volumetric_mesh.vertices[i];
        if (std::abs(base_v.z - z_base) > 1e-6f) {
            std::cout << "Error: Base vertex " << i << " has wrong z-coordinate: " 
                      << base_v.z << " (expected " << z_base << ")\n";
            return false;
        }
    }
    
    return true;
}

// Calculate mesh volume using divergence theorem
float calculateVolume(const TerraScape::MeshResult& mesh) {
    float volume = 0.0f;
    
    for (const auto& tri : mesh.triangles) {
        const auto& v0 = mesh.vertices[tri.v0];
        const auto& v1 = mesh.vertices[tri.v1];
        const auto& v2 = mesh.vertices[tri.v2];
        
        // Compute triangle contribution to volume
        volume += (v0.x * (v1.y * v2.z - v2.y * v1.z) +
                   v1.x * (v2.y * v0.z - v0.y * v2.z) +
                   v2.x * (v0.y * v1.z - v1.y * v0.z)) / 6.0f;
    }
    
    return std::abs(volume);
}

} // anonymous namespace

int main() {
    std::cout << "=== TerraScape Volumetric Mesh Test Suite ===\n\n";
    
    bool all_tests_passed = true;
    
    // Test 1: Simple hill terrain
    std::cout << "Test 1: Simple Hill Terrain\n";
    {
        // Create a simple 5x5 hill
        const int width = 5, height = 5;
        std::vector<float> elevations(width * height);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float dx = x - 2.0f;
                float dy = y - 2.0f;
                elevations[y * width + x] = 5.0f - std::sqrt(dx*dx + dy*dy);
            }
        }
        
        // Generate surface mesh
        auto surface_mesh = TerraScape::grid_to_mesh(width, height, elevations.data(), 
                                                    0.1f, 1000);
        
        // Generate volumetric mesh
        float z_base = 0.0f;
        auto volumetric_mesh = TerraScape::grid_to_mesh_volumetric(width, height, elevations.data(),
                                                                  z_base, 0.1f, 1000);
        
        std::cout << "  Surface mesh: " << surface_mesh.vertices.size() << " vertices, " 
                  << surface_mesh.triangles.size() << " triangles\n";
        std::cout << "  Volumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " 
                  << volumetric_mesh.triangles.size() << " triangles\n";
        
        // Test structure
        if (!testVolumetricStructure(surface_mesh, volumetric_mesh, z_base)) {
            std::cout << "  ❌ Structure test FAILED\n";
            all_tests_passed = false;
        } else {
            std::cout << "  ✓ Structure test PASSED\n";
        }
        
        // Test manifold property
        if (!isManifold(volumetric_mesh)) {
            std::cout << "  ❌ Manifold test FAILED\n";
            all_tests_passed = false;
        } else {
            std::cout << "  ✓ Manifold test PASSED\n";
        }
        
        // Test watertight property
        if (!isWatertight(volumetric_mesh)) {
            std::cout << "  ❌ Watertight test FAILED\n";
            all_tests_passed = false;
        } else {
            std::cout << "  ✓ Watertight test PASSED\n";
        }
        
        // Test volumetric flag
        if (!volumetric_mesh.is_volumetric) {
            std::cout << "  ❌ Volumetric flag test FAILED\n";
            all_tests_passed = false;
        } else {
            std::cout << "  ✓ Volumetric flag test PASSED\n";
        }
        
        // Calculate and report volume
        float volume = calculateVolume(volumetric_mesh);
        std::cout << "  Volume: " << volume << " cubic units\n";
        
        // Write meshes for inspection
        writeOBJ("/tmp/simple_hill_surface.obj", surface_mesh);
        writeOBJ("/tmp/simple_hill_volumetric.obj", volumetric_mesh);
    }
    
    std::cout << "\n";
    
    // Test 2: Flat plane (minimal case)
    std::cout << "Test 2: Flat Plane\n";
    {
        const int width = 3, height = 3;
        std::vector<float> elevations(width * height, 1.0f); // All same height
        
        auto surface_mesh = TerraScape::grid_to_mesh(width, height, elevations.data());
        auto volumetric_mesh = TerraScape::grid_to_mesh_volumetric(width, height, elevations.data(), 0.0f);
        
        std::cout << "  Surface mesh: " << surface_mesh.vertices.size() << " vertices, " 
                  << surface_mesh.triangles.size() << " triangles\n";
        std::cout << "  Volumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " 
                  << volumetric_mesh.triangles.size() << " triangles\n";
        
        bool flat_test_passed = true;
        
        // Test structure
        if (!testVolumetricStructure(surface_mesh, volumetric_mesh, 0.0f)) {
            std::cout << "  ❌ Structure test FAILED\n";
            flat_test_passed = false;
        } else {
            std::cout << "  ✓ Structure test PASSED\n";
        }
        
        // Test manifold property
        if (!isManifold(volumetric_mesh)) {
            std::cout << "  ❌ Manifold test FAILED\n";
            flat_test_passed = false;
        } else {
            std::cout << "  ✓ Manifold test PASSED\n";
        }
        
        // Volume should be approximately 1.0 * 2.0 * 2.0 = 4.0 for a 2x2 grid with height 1
        float volume = calculateVolume(volumetric_mesh);
        std::cout << "  Volume: " << volume << " cubic units\n";
        
        writeOBJ("/tmp/flat_plane_surface.obj", surface_mesh);
        writeOBJ("/tmp/flat_plane_volumetric.obj", volumetric_mesh);
        
        if (flat_test_passed) {
            std::cout << "  ✓ All flat plane tests PASSED\n";
        } else {
            all_tests_passed = false;
        }
    }
    
    std::cout << "\n";
    
    // Test 3: Different base levels
    std::cout << "Test 3: Different Base Levels\n";
    {
        const int width = 4, height = 4;
        std::vector<float> elevations(width * height);
        
        // Create a simple pyramid
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                elevations[y * width + x] = 10.0f - std::max(std::abs(x - 1.5f), std::abs(y - 1.5f));
            }
        }
        
        float z_base = -5.0f; // Base below zero
        auto volumetric_mesh = TerraScape::grid_to_mesh_volumetric(width, height, elevations.data(), z_base);
        
        std::cout << "  Volumetric mesh with z_base=" << z_base << ": " 
                  << volumetric_mesh.vertices.size() << " vertices, " 
                  << volumetric_mesh.triangles.size() << " triangles\n";
        
        // Check that base vertices have correct z-coordinate
        bool base_test_passed = true;
        size_t surface_vertex_count = volumetric_mesh.vertices.size() / 2;
        for (size_t i = surface_vertex_count; i < volumetric_mesh.vertices.size(); ++i) {
            if (std::abs(volumetric_mesh.vertices[i].z - z_base) > 1e-6f) {
                std::cout << "  ❌ Base vertex " << i << " has wrong z-coordinate\n";
                base_test_passed = false;
                break;
            }
        }
        
        if (base_test_passed && isManifold(volumetric_mesh)) {
            std::cout << "  ✓ Base level test PASSED\n";
        } else {
            std::cout << "  ❌ Base level test FAILED\n";
            all_tests_passed = false;
        }
        
        writeOBJ("/tmp/pyramid_volumetric.obj", volumetric_mesh);
    }
    
    std::cout << "\n=== Test Summary ===\n";
    if (all_tests_passed) {
        std::cout << "🎉 All volumetric mesh tests PASSED!\n";
        std::cout << "The volumetric mesh generation produces watertight, manifold meshes.\n";
        return 0;
    } else {
        std::cout << "❌ Some tests FAILED!\n";
        return 1;
    }
}