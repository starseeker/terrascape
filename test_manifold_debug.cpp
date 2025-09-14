#include "TerraScape.hpp"
#include <iostream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <map>

using namespace TerraScape;

// Detailed analysis of manifold issues
bool debug_manifold_issues() {
    std::cout << "=== Detailed Manifold Analysis ===" << std::endl;
    
    // Create a simple test terrain
    std::vector<float> elevations = {
        1, 1, 1,
        1, 2, 1,
        1, 1, 1
    };
    
    // Generate mesh
    auto surface_mesh = region_growing_triangulation(elevations.data(), 3, 3, 0.5);
    auto volumetric_mesh = make_volumetric_mesh(surface_mesh, 0.0);
    
    std::cout << "Surface mesh: " << surface_mesh.vertices.size() << " vertices, " 
              << surface_mesh.triangles.size() << " triangles" << std::endl;
    std::cout << "Volumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " 
              << volumetric_mesh.triangles.size() << " triangles" << std::endl;
    
    // Analyze edge usage in detail
    std::map<std::pair<int, int>, std::vector<int>> edge_triangles; // edge -> list of triangle indices
    
    for (size_t tri_idx = 0; tri_idx < volumetric_mesh.triangles.size(); tri_idx++) {
        const auto& tri = volumetric_mesh.triangles[tri_idx];
        
        // Ensure consistent edge ordering (smaller vertex first)
        auto add_edge = [&](int v1, int v2) {
            if (v1 > v2) std::swap(v1, v2);
            edge_triangles[{v1, v2}].push_back(static_cast<int>(tri_idx));
        };
        
        add_edge(tri.v0, tri.v1);
        add_edge(tri.v1, tri.v2);
        add_edge(tri.v2, tri.v0);
    }
    
    std::cout << "\nEdge usage analysis:" << std::endl;
    int boundary_edges = 0;
    int manifold_edges = 0;
    int over_used_edges = 0;
    
    for (const auto& [edge, triangle_list] : edge_triangles) {
        int count = triangle_list.size();
        if (count == 1) {
            boundary_edges++;
        } else if (count == 2) {
            manifold_edges++;
        } else {
            over_used_edges++;
            std::cout << "Over-used edge (" << edge.first << ", " << edge.second 
                      << ") used by " << count << " triangles: ";
            for (int tri_idx : triangle_list) {
                const auto& tri = volumetric_mesh.triangles[tri_idx];
                std::cout << "T" << tri_idx << "(" << tri.v0 << "," << tri.v1 << "," << tri.v2 << ") ";
            }
            std::cout << std::endl;
            
            // Show vertex positions for this edge
            const auto& v1 = volumetric_mesh.vertices[edge.first];
            const auto& v2 = volumetric_mesh.vertices[edge.second];
            std::cout << "  Vertex " << edge.first << ": (" << v1.x << ", " << v1.y << ", " << v1.z << ")" << std::endl;
            std::cout << "  Vertex " << edge.second << ": (" << v2.x << ", " << v2.y << ", " << v2.z << ")" << std::endl;
        }
    }
    
    std::cout << "Boundary edges: " << boundary_edges << std::endl;
    std::cout << "Manifold edges: " << manifold_edges << std::endl;
    std::cout << "Over-used edges: " << over_used_edges << std::endl;
    
    // Show all vertices to understand the structure
    std::cout << "\nAll vertices:" << std::endl;
    for (size_t i = 0; i < volumetric_mesh.vertices.size(); i++) {
        const auto& v = volumetric_mesh.vertices[i];
        std::cout << "V" << i << ": (" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
    }
    
    return over_used_edges == 0;
}

int main() {
    bool passed = debug_manifold_issues();
    
    if (passed) {
        std::cout << "\n✅ No manifold issues found!" << std::endl;
        return 0;
    } else {
        std::cout << "\n❌ Manifold issues detected!" << std::endl;
        return 1;
    }
}