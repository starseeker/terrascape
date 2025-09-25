#include "TerraScape.hpp"
#include <iostream>

int main() {
    using namespace TerraScape;
    
    // Read terrain
    TerrainData terrain;
    if (!readTerrainFile("../crater.pgm", terrain)) {
        std::cerr << "Failed to read terrain file" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded terrain: " << terrain.width << "x" << terrain.height << " cells" << std::endl;
    
    // Create mesh using simplified approach
    TerrainMesh mesh;
    SimplificationParams params;
    params.error_threshold = 0.1;
    params.min_triangle_reduction = 70;
    
    triangulateTerrainVolumeSimplified(terrain, mesh, params);
    
    std::cout << "Generated mesh: " << mesh.vertices.size() << " vertices, " << mesh.triangles.size() << " triangles" << std::endl;
    
    // Do detailed validation with debug info
    MeshStats stats;
    
    // Check edge manifold property
    std::unordered_map<Edge, int, EdgeHash> edge_count;
    
    for (const auto& triangle : mesh.triangles) {
        Edge e1(triangle.vertices[0], triangle.vertices[1]);
        Edge e2(triangle.vertices[1], triangle.vertices[2]);
        Edge e3(triangle.vertices[2], triangle.vertices[0]);
        
        edge_count[e1]++;
        edge_count[e2]++;
        edge_count[e3]++;
    }
    
    // Analyze edge distribution
    std::map<int, int> count_distribution;
    for (const auto& pair : edge_count) {
        count_distribution[pair.second]++;
    }
    
    std::cout << "Edge count distribution:" << std::endl;
    for (const auto& pair : count_distribution) {
        std::cout << "  Edges with count " << pair.first << ": " << pair.second << std::endl;
    }
    
    // Count non-manifold edges
    stats.is_manifold = true;  // Start with true
    for (const auto& pair : edge_count) {
        if (pair.second > 2) {  // Non-manifold edge
            stats.non_manifold_edges++;
            stats.is_manifold = false;
            std::cout << "Found non-manifold edge: v" << pair.first.v0 << "-v" << pair.first.v1 
                      << " (count=" << pair.second << ")" << std::endl;
        }
    }
    
    std::cout << "Manifold check result:" << std::endl;
    std::cout << "  Is manifold: " << (stats.is_manifold ? "yes" : "no") << std::endl;
    std::cout << "  Non-manifold edges: " << stats.non_manifold_edges << std::endl;
    
    return 0;
}