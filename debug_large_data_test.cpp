#include "TerraScape.hpp"
#include <chrono>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>

// Debug test to identify the large data handling issue
int main() {
    std::cout << "=== Debug Large Data Test ===\n";
    
    // Test progressively larger sizes to find the breaking point
    std::vector<std::pair<int, int>> test_sizes = {
        {100, 100},    // 10K points - should work
        {200, 200},    // 40K points - should work  
        {300, 200},    // 60K points - test
        {400, 300},    // 120K points - test
        {500, 300}     // 150K points - might fail
    };
    
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> noise(-10.0f, 10.0f);
    
    for (const auto& size : test_sizes) {
        int width = size.first;
        int height = size.second;
        int total_points = width * height;
        
        std::cout << "\nTesting " << width << "x" << height << " (" << total_points << " points):\n";
        
        try {
            // Generate simple terrain
            std::cout << "  Generating terrain data...";
            std::vector<float> elevations(total_points);
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    float fx = static_cast<float>(x) / (width - 1);
                    float fy = static_cast<float>(y) / (height - 1);
                    float dist = std::sqrt((fx - 0.5f) * (fx - 0.5f) + (fy - 0.5f) * (fy - 0.5f));
                    elevations[y * width + x] = std::max(0.0f, 1000.0f * (1.0f - dist * 2.0f)) + noise(rng);
                }
            }
            std::cout << " OK\n";
            
            // Test different strategies
            std::vector<std::pair<TerraScape::MeshRefineStrategy, std::string>> strategies = {
                {TerraScape::MeshRefineStrategy::SPARSE, "SPARSE"},
                {TerraScape::MeshRefineStrategy::AUTO, "AUTO"},
                {TerraScape::MeshRefineStrategy::HYBRID, "HYBRID"}
            };
            
            for (const auto& strategy_pair : strategies) {
                auto strategy = strategy_pair.first;
                auto strategy_name = strategy_pair.second;
                
                std::cout << "  Testing " << strategy_name << " strategy...";
                auto start_time = std::chrono::high_resolution_clock::now();
                
                try {
                    auto mesh = TerraScape::grid_to_mesh(
                        width, height, elevations.data(),
                        10.0f, // reasonable error threshold
                        1000,  // reasonable point limit
                        strategy
                    );
                    
                    auto end_time = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                    
                    std::cout << " SUCCESS (" << duration.count() << "ms, " 
                              << mesh.vertices.size() << " vertices)\n";
                              
                } catch (const std::exception& e) {
                    std::cout << " EXCEPTION: " << e.what() << "\n";
                } catch (...) {
                    std::cout << " UNKNOWN EXCEPTION\n";
                }
            }
                      
        } catch (const std::exception& e) {
            std::cout << " EXCEPTION: " << e.what() << "\n";
            break;
        } catch (...) {
            std::cout << " UNKNOWN EXCEPTION\n";
            break;
        }
    }
    
    std::cout << "\nDebug test complete.\n";
    return 0;
}