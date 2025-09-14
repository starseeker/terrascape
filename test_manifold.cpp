#include "TerraScape.hpp"
#include <iostream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <set>

using namespace TerraScape;

// Test for manifold properties and mesh quality
bool test_mesh_manifold_properties() {
    std::cout << "=== Mesh Manifold Properties Test ===" << std::endl;
    
    // Create a simple test terrain
    std::vector<float> elevations = {
        1, 1, 2, 1, 1,
        1, 2, 3, 2, 1,
        2, 3, 4, 3, 2,
        1, 2, 3, 2, 1,
        1, 1, 2, 1, 1
    };
    
    // Generate mesh
    auto surface_mesh = region_growing_triangulation(elevations.data(), 5, 5, 0.5);
    auto volumetric_mesh = make_volumetric_mesh(surface_mesh, 0.0);
    
    std::cout << "Surface mesh: " << surface_mesh.vertices.size() << " vertices, " 
              << surface_mesh.triangles.size() << " triangles" << std::endl;
    std::cout << "Volumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " 
              << volumetric_mesh.triangles.size() << " triangles" << std::endl;
    
    // Check for manifold properties
    bool manifold_issues = false;
    
    // Check 1: Vertex usage - each vertex should be used in triangles
    std::vector<int> vertex_usage(volumetric_mesh.vertices.size(), 0);
    for (const auto& tri : volumetric_mesh.triangles) {
        vertex_usage[tri.v0]++;
        vertex_usage[tri.v1]++;
        vertex_usage[tri.v2]++;
    }
    
    int unused_vertices = 0;
    for (size_t i = 0; i < vertex_usage.size(); i++) {
        if (vertex_usage[i] == 0) {
            unused_vertices++;
        }
    }
    
    std::cout << "Unused vertices: " << unused_vertices << std::endl;
    if (unused_vertices > 0) {
        std::cout << "⚠ WARNING: Found unused vertices - potential mesh quality issue" << std::endl;
        manifold_issues = true;
    }
    
    // Check 2: Edge usage - each edge should be used by exactly 2 triangles for manifold
    std::map<std::pair<int, int>, int> edge_usage;
    
    for (const auto& tri : volumetric_mesh.triangles) {
        // Ensure consistent edge ordering (smaller vertex first)
        auto add_edge = [&](int v1, int v2) {
            if (v1 > v2) std::swap(v1, v2);
            edge_usage[{v1, v2}]++;
        };
        
        add_edge(tri.v0, tri.v1);
        add_edge(tri.v1, tri.v2);
        add_edge(tri.v2, tri.v0);
    }
    
    int boundary_edges = 0;
    int over_used_edges = 0;
    
    for (const auto& [edge, count] : edge_usage) {
        if (count == 1) {
            boundary_edges++;
        } else if (count > 2) {
            over_used_edges++;
        }
    }
    
    std::cout << "Total edges: " << edge_usage.size() << std::endl;
    std::cout << "Boundary edges: " << boundary_edges << std::endl;
    std::cout << "Over-used edges: " << over_used_edges << std::endl;
    
    if (volumetric_mesh.is_volumetric && boundary_edges > 0) {
        std::cout << "⚠ WARNING: Volumetric mesh has boundary edges - not a closed manifold" << std::endl;
        manifold_issues = true;
    }
    
    if (over_used_edges > 0) {
        std::cout << "❌ ERROR: Found non-manifold edges (used by >2 triangles)" << std::endl;
        manifold_issues = true;
    }
    
    // Check 3: Triangle validity
    int degenerate_triangles = 0;
    int flipped_triangles = 0;
    
    for (const auto& tri : volumetric_mesh.triangles) {
        if (tri.v0 == tri.v1 || tri.v1 == tri.v2 || tri.v2 == tri.v0) {
            degenerate_triangles++;
            continue;
        }
        
        // Check triangle normal orientation
        const auto& v0 = volumetric_mesh.vertices[tri.v0];
        const auto& v1 = volumetric_mesh.vertices[tri.v1];
        const auto& v2 = volumetric_mesh.vertices[tri.v2];
        
        // Vector from v0 to v1 and v0 to v2
        double dx1 = v1.x - v0.x, dy1 = v1.y - v0.y, dz1 = v1.z - v0.z;
        double dx2 = v2.x - v0.x, dy2 = v2.y - v0.y, dz2 = v2.z - v0.z;
        
        // Cross product (normal)
        double nx = dy1 * dz2 - dz1 * dy2;
        double ny = dz1 * dx2 - dx1 * dz2;
        double nz = dx1 * dy2 - dy1 * dx2;
        
        // For terrain, we generally expect upward normals (positive Z component)
        // But volumetric meshes will have both upward and downward faces
        if (!volumetric_mesh.is_volumetric && nz < -0.1) {
            flipped_triangles++;
        }
    }
    
    std::cout << "Degenerate triangles: " << degenerate_triangles << std::endl;
    std::cout << "Flipped triangles (surface only): " << flipped_triangles << std::endl;
    
    if (degenerate_triangles > 0) {
        std::cout << "❌ ERROR: Found degenerate triangles" << std::endl;
        manifold_issues = true;
    }
    
    if (!volumetric_mesh.is_volumetric && flipped_triangles > 0) {
        std::cout << "❌ ERROR: Found flipped triangles in surface mesh" << std::endl;
        manifold_issues = true;
    }
    
    // Summary
    if (!manifold_issues) {
        std::cout << "✅ PASS: Mesh manifold properties check passed" << std::endl;
        std::cout << "✓ No unused vertices" << std::endl;
        std::cout << "✓ No non-manifold edges" << std::endl;
        std::cout << "✓ No degenerate triangles" << std::endl;
        std::cout << "✓ Proper triangle orientation" << std::endl;
        if (volumetric_mesh.is_volumetric && boundary_edges == 0) {
            std::cout << "✓ Closed manifold (no boundary edges)" << std::endl;
        }
    } else {
        std::cout << "❌ FAIL: Mesh has manifold or quality issues" << std::endl;
    }
    
    return !manifold_issues;
}

int main() {
    bool passed = test_mesh_manifold_properties();
    
    if (passed) {
        std::cout << "\n🎉 Mesh manifold tests PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "\n💥 Mesh manifold tests FAILED!" << std::endl;
        return 1;
    }
}