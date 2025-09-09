#include "TerraScape.hpp"
#include <chrono>
#include <iostream>
#include <vector>
#include <fstream>

// Simple performance test for different strategies and datasets
int main() {
    std::cout << "=== TerraScape Performance Test ===" << std::endl;
    
    // Test with different grid sizes
    std::vector<std::pair<int, int>> test_sizes = {
        {50, 50},     // Small: 2.5K points
        {100, 100},   // Medium: 10K points  
        {200, 200},   // Large: 40K points
        {300, 300}    // Very Large: 90K points
    };
    
    for (const auto& size : test_sizes) {
        int width = size.first;
        int height = size.second;
        int total_points = width * height;
        
        std::cout << "\n--- Testing " << width << "x" << height << " grid (" 
                  << total_points << " points) ---" << std::endl;
        
        // Generate synthetic terrain data (single peak)
        std::vector<float> elevations(total_points);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float fx = static_cast<float>(x) / (width - 1);
                float fy = static_cast<float>(y) / (height - 1);
                float distance = std::sqrt((fx - 0.5f) * (fx - 0.5f) + (fy - 0.5f) * (fy - 0.5f));
                elevations[y * width + x] = std::max(0.0f, 100.0f * (1.0f - distance * 2.0f));
            }
        }
        
        // Test different strategies
        std::vector<TerraScape::MeshRefineStrategy> strategies = {
            TerraScape::MeshRefineStrategy::AUTO,
            TerraScape::MeshRefineStrategy::SPARSE,
            TerraScape::MeshRefineStrategy::HYBRID
        };
        
        for (auto strategy : strategies) {
            std::string strategy_name;
            switch (strategy) {
                case TerraScape::MeshRefineStrategy::AUTO: strategy_name = "AUTO"; break;
                case TerraScape::MeshRefineStrategy::SPARSE: strategy_name = "SPARSE"; break;
                case TerraScape::MeshRefineStrategy::HYBRID: strategy_name = "HYBRID"; break;
                case TerraScape::MeshRefineStrategy::HEAP: strategy_name = "HEAP"; break;
            }
            
            auto start_time = std::chrono::high_resolution_clock::now();
            
            auto mesh = TerraScape::grid_to_mesh(
                width, height, elevations.data(),
                2.0f,    // error threshold
                1000     // point limit
            );
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            
            std::cout << "  " << strategy_name << ": " << duration.count() << "ms, "
                      << mesh.vertices.size() << " vertices, " 
                      << mesh.triangles.size() << " triangles" << std::endl;
        }
    }
    
    return 0;
}