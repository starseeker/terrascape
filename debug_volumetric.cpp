#include "TerraScape.hpp"
#include <iostream>
#include <map>
#include <fstream>

// Function to save a mesh to OBJ for visualization
void save_mesh_obj(const TerraScape::MeshResult& mesh, const std::string& filename) {
    std::ofstream file(filename);
    for (const auto& v : mesh.vertices) {
        file << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    for (const auto& t : mesh.triangles) {
        file << "f " << (t.v0 + 1) << " " << (t.v1 + 1) << " " << (t.v2 + 1) << "\n";
    }
    std::cout << "Saved " << filename << " with " << mesh.vertices.size() << " vertices, " << mesh.triangles.size() << " triangles\n";
}

// Function to analyze boundary edges and mesh topology
void analyze_volumetric_topology(const TerraScape::MeshResult& mesh, const std::string& mesh_name) {
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
    
    // Find boundary edges and analyze vertex locations
    std::vector<std::pair<int, int>> boundary_edges;
    for (const auto& [edge, count] : edge_count) {
        if (count == 1) {
            boundary_edges.push_back(edge);
        }
    }
    
    std::cout << "\n=== Volumetric Topology Analysis for " << mesh_name << " ===" << std::endl;
    std::cout << "Total vertices: " << mesh.vertices.size() << std::endl;
    std::cout << "Total triangles: " << mesh.triangles.size() << std::endl;
    std::cout << "Total edges: " << edge_count.size() << std::endl;
    std::cout << "Boundary edges: " << boundary_edges.size() << std::endl;
    
    if (boundary_edges.size() == 0) {
        std::cout << "✓ PERFECT: Volumetric mesh is a closed manifold (no boundary edges)" << std::endl;
    } else {
        std::cout << "⚠ ISSUE: Volumetric mesh has " << boundary_edges.size() << " boundary edges" << std::endl;
        
        // Analyze where the boundary edges are located
        std::map<float, int> z_levels;
        for (const auto& edge : boundary_edges) {
            float z1 = mesh.vertices[edge.first].z;
            float z2 = mesh.vertices[edge.second].z;
            z_levels[z1]++;
            z_levels[z2]++;
        }
        
        std::cout << "Boundary edges by Z level:" << std::endl;
        for (const auto& [z, count] : z_levels) {
            std::cout << "  Z=" << z << ": " << count << " edge endpoints" << std::endl;
        }
        
        // Check if boundary edges are on the perimeter
        std::cout << "\nFirst few boundary edges:" << std::endl;
        for (int i = 0; i < std::min(5, (int)boundary_edges.size()); i++) {
            const auto& edge = boundary_edges[i];
            const auto& v1 = mesh.vertices[edge.first];
            const auto& v2 = mesh.vertices[edge.second];
            std::cout << "  Edge " << edge.first << "-" << edge.second 
                      << ": (" << v1.x << "," << v1.y << "," << v1.z << ") - "
                      << "(" << v2.x << "," << v2.y << "," << v2.z << ")" << std::endl;
        }
    }
}

int main() {
    // Create simple 3x3 test terrain
    std::vector<float> elevations = {
        1, 2, 1,
        2, 4, 2,
        1, 2, 1
    };
    
    int width = 3, height = 3;
    
    std::cout << "Testing volumetric mesh topology with 3x3 terrain" << std::endl;
    
    // Generate surface mesh
    TerraScape::MeshResult surface_mesh = TerraScape::region_growing_triangulation(
        elevations.data(), width, height, 0.9, nullptr);
    
    save_mesh_obj(surface_mesh, "debug_surface.obj");
    
    // Generate volumetric mesh with base at z=0
    TerraScape::MeshResult volumetric_mesh = TerraScape::make_volumetric_mesh(surface_mesh, 0.0f);
    
    save_mesh_obj(volumetric_mesh, "debug_volumetric.obj");
    
    analyze_volumetric_topology(volumetric_mesh, "Debug Volumetric Mesh");
    
    return 0;
}