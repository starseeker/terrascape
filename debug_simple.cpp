#include "TerraScape.hpp"
#include <iostream>
#include <map>
#include <fstream>

int main() {
    // Create minimal 2x2 test terrain - should be easier to debug
    std::vector<float> elevations = {
        1, 1,
        1, 1
    };
    
    int width = 2, height = 2;
    
    std::cout << "Testing with minimal 2x2 terrain (all elevations = 1)" << std::endl;
    
    // Generate surface mesh
    TerraScape::MeshResult surface_mesh = TerraScape::region_growing_triangulation(
        elevations.data(), width, height, 1.0, nullptr);
    
    std::cout << "Surface mesh: " << surface_mesh.vertices.size() << " vertices, " << surface_mesh.triangles.size() << " triangles" << std::endl;
    
    // Show surface mesh
    std::cout << "\nSurface vertices:" << std::endl;
    for (size_t i = 0; i < surface_mesh.vertices.size(); i++) {
        const auto& v = surface_mesh.vertices[i];
        std::cout << "  " << i << ": (" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
    }
    
    std::cout << "\nSurface triangles:" << std::endl;
    for (size_t i = 0; i < surface_mesh.triangles.size(); i++) {
        const auto& t = surface_mesh.triangles[i];
        std::cout << "  " << i << ": " << t.v0 << "-" << t.v1 << "-" << t.v2 << std::endl;
    }
    
    // Generate volumetric mesh with base at z=0
    TerraScape::MeshResult volumetric_mesh = TerraScape::make_volumetric_mesh(surface_mesh, 0.0f);
    
    std::cout << "\nVolumetric mesh: " << volumetric_mesh.vertices.size() << " vertices, " << volumetric_mesh.triangles.size() << " triangles" << std::endl;
    
    // Show additional vertices (base vertices)
    std::cout << "\nBase vertices (added for volumetric):" << std::endl;
    for (size_t i = surface_mesh.vertices.size(); i < volumetric_mesh.vertices.size(); i++) {
        const auto& v = volumetric_mesh.vertices[i];
        std::cout << "  " << i << ": (" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
    }
    
    // Show additional triangles (base + walls)
    std::cout << "\nAdditional triangles (base + walls):" << std::endl;
    for (size_t i = surface_mesh.triangles.size(); i < volumetric_mesh.triangles.size(); i++) {
        const auto& t = volumetric_mesh.triangles[i];
        std::cout << "  " << i << ": " << t.v0 << "-" << t.v1 << "-" << t.v2 << std::endl;
    }
    
    // Check boundary edges
    std::map<std::pair<int, int>, int> edge_count;
    for (const auto& tri : volumetric_mesh.triangles) {
        auto add_edge = [&](int a, int b) {
            if (a > b) std::swap(a, b);
            edge_count[{a, b}]++;
        };
        add_edge(tri.v0, tri.v1);
        add_edge(tri.v1, tri.v2);
        add_edge(tri.v2, tri.v0);
    }
    
    std::cout << "\nBoundary edges (used only once):" << std::endl;
    for (const auto& [edge, count] : edge_count) {
        if (count == 1) {
            const auto& v1 = volumetric_mesh.vertices[edge.first];
            const auto& v2 = volumetric_mesh.vertices[edge.second];
            std::cout << "  " << edge.first << "-" << edge.second
                      << ": (" << v1.x << "," << v1.y << "," << v1.z << ") - "
                      << "(" << v2.x << "," << v2.y << "," << v2.z << ")" << std::endl;
        }
    }
    
    return 0;
}