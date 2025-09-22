#include "TerraScape.hpp"
#include <iostream>
#include <map>

// Function to analyze edge usage in a mesh
void analyze_edge_usage(const TerraScape::MeshResult& mesh, const std::string& mesh_name) {
    std::map<std::pair<int, int>, int> edge_count;
    
    // Count edge usage
    for (const auto& tri : mesh.triangles) {
        auto add_edge = [&](int a, int b) {
            if (a > b) std::swap(a, b);
            edge_count[{a, b}]++;
        };
        
        add_edge(tri.v0, tri.v1);
        add_edge(tri.v1, tri.v2);
        add_edge(tri.v2, tri.v0);
    }
    
    // Analyze edge usage
    int edges_used_once = 0;
    int edges_used_twice = 0;
    int edges_used_more = 0;
    int max_usage = 0;
    
    for (const auto& [edge, count] : edge_count) {
        if (count == 1) edges_used_once++;
        else if (count == 2) edges_used_twice++;
        else edges_used_more++;
        
        max_usage = std::max(max_usage, count);
    }
    
    std::cout << "\n=== Edge Usage Analysis for " << mesh_name << " ===" << std::endl;
    std::cout << "Total vertices: " << mesh.vertices.size() << std::endl;
    std::cout << "Total triangles: " << mesh.triangles.size() << std::endl;
    std::cout << "Total edges: " << edge_count.size() << std::endl;
    std::cout << "Edges used once (boundary): " << edges_used_once << std::endl;
    std::cout << "Edges used twice: " << edges_used_twice << std::endl;
    std::cout << "Edges used more than twice: " << edges_used_more << std::endl;
    std::cout << "Maximum edge usage: " << max_usage << std::endl;
    
    if (mesh_name.find("Surface") != std::string::npos) {
        if (edges_used_more == 0 && edges_used_once > 0) {
            std::cout << "✓ FIXED: Surface mesh is properly non-manifold (heightfield)" << std::endl;
        } else if (edges_used_more > 0) {
            std::cout << "✓ GREAT: Surface mesh allows edges used >2 times (true heightfield)" << std::endl;
        } else {
            std::cout << "? UNKNOWN: Surface mesh appears to be manifold (might be wrong)" << std::endl;
        }
    } else if (mesh_name.find("Volumetric") != std::string::npos) {
        if (edges_used_more == 0 && edges_used_once == 0) {
            std::cout << "✓ PERFECT: Volumetric mesh is a closed manifold" << std::endl;
        } else {
            std::cout << "⚠ WARNING: Volumetric mesh has boundary edges (not closed)" << std::endl;
        }
    }
}

int main() {
    // Create test terrain - a 4x4 grid with a pyramid shape
    std::vector<float> elevations = {
        0,  1,  1,  0,
        1,  5,  5,  1,
        1,  5,  5,  1,
        0,  1,  1,  0
    };
    
    int width = 4, height = 4;
    
    std::cout << "Testing manifold behavior with pyramid terrain (4x4)" << std::endl;
    
    // Generate surface mesh (should be non-manifold heightfield)
    TerraScape::MeshResult surface_mesh = TerraScape::region_growing_triangulation(
        elevations.data(), width, height, 0.8, nullptr);
    
    analyze_edge_usage(surface_mesh, "Surface Mesh");
    
    // Generate volumetric mesh (should be closed manifold)
    TerraScape::MeshResult volumetric_mesh = TerraScape::make_volumetric_mesh(surface_mesh, 0.0f);
    
    analyze_edge_usage(volumetric_mesh, "Volumetric Mesh");
    
    return 0;
}