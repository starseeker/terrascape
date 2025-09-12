#include "TerraScape.hpp"
#include <iostream>
#include <vector>
#include <iomanip>

int main() {
    std::cout << "=== TerraScape Simulation of Simplicity Determinism Test ===" << std::endl;
    
    // Test completely flat grid - this should trigger SoS perturbation
    std::vector<float> flat_data(25, 42.0f);  // 5x5 grid, all same elevation
    
    // Run the same test multiple times to verify deterministic behavior
    std::vector<TerraScape::MeshResult> results;
    
    for (int run = 0; run < 5; ++run) {
        std::cout << "Run " << (run + 1) << ": ";
        auto result = TerraScape::grid_to_mesh(5, 5, flat_data.data(), 0.1f, 100);
        results.push_back(result);
        std::cout << "Generated " << result.vertices.size() << " vertices, " 
                  << result.triangles.size() << " triangles" << std::endl;
    }
    
    // Verify all results are identical
    bool all_identical = true;
    for (int run = 1; run < 5; ++run) {
        // Check vertices
        if (results[0].vertices.size() != results[run].vertices.size()) {
            all_identical = false;
            break;
        }
        
        for (size_t i = 0; i < results[0].vertices.size(); ++i) {
            const auto& v0 = results[0].vertices[i];
            const auto& vr = results[run].vertices[i];
            
            if (std::abs(v0.x - vr.x) > 1e-15f || 
                std::abs(v0.y - vr.y) > 1e-15f || 
                std::abs(v0.z - vr.z) > 1e-15f) {
                all_identical = false;
                std::cout << "Vertex " << i << " differs: (" << v0.x << "," << v0.y << "," << v0.z 
                          << ") vs (" << vr.x << "," << vr.y << "," << vr.z << ")" << std::endl;
                break;
            }
        }
        
        // Check triangles
        if (results[0].triangles.size() != results[run].triangles.size()) {
            all_identical = false;
            break;
        }
        
        for (size_t i = 0; i < results[0].triangles.size(); ++i) {
            const auto& t0 = results[0].triangles[i];
            const auto& tr = results[run].triangles[i];
            
            if (t0.v0 != tr.v0 || t0.v1 != tr.v1 || t0.v2 != tr.v2) {
                all_identical = false;
                std::cout << "Triangle " << i << " differs: (" << t0.v0 << "," << t0.v1 << "," << t0.v2 
                          << ") vs (" << tr.v0 << "," << tr.v1 << "," << tr.v2 << ")" << std::endl;
                break;
            }
        }
        
        if (!all_identical) break;
    }
    
    if (all_identical) {
        std::cout << "✓ SUCCESS: All runs produced identical results!" << std::endl;
        std::cout << "  Simulation of Simplicity ensures deterministic behavior" << std::endl;
    } else {
        std::cout << "✗ FAILURE: Results differ between runs!" << std::endl;
        std::cout << "  Simulation of Simplicity implementation may have issues" << std::endl;
        return 1;
    }
    
    // Show sample vertex coordinates to demonstrate precision
    std::cout << "Sample vertices from first run:" << std::endl;
    for (size_t i = 0; i < std::min(size_t(10), results[0].vertices.size()); ++i) {
        const auto& v = results[0].vertices[i];
        std::cout << std::fixed << std::setprecision(15) 
                  << "  Vertex " << i << ": (" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
    }
    
    return 0;
}