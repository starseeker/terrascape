#include "TerraScape.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>

// Simple PGM reader
bool readPGM(const char* filename, int& width, int& height, std::vector<float>& elevations) {
    std::ifstream file(filename);
    if (!file) return false;
    
    char magicP, magicNum;
    int maxval;
    file >> magicP >> magicNum >> width >> height >> maxval;
    
    if (magicP != 'P' || magicNum != '2') return false;
    
    elevations.resize(width * height);
    for (int i = 0; i < width * height; ++i) {
        float val;
        file >> val;
        elevations[i] = val;
    }
    return true;
}

int main() {
    std::cout << "=== TerraScape Strategy Comparison Test ===" << std::endl;
    
    int width, height;
    std::vector<float> elevations;
    
    // Read crater data
    if (!readPGM("crater.pgm", width, height, elevations)) {
        std::cerr << "Failed to read crater.pgm" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << width << "x" << height << " (" << width*height << " points)" << std::endl;
    
    // Test different strategies
    std::vector<std::pair<TerraScape::MeshRefineStrategy, std::string>> strategies = {
        {TerraScape::MeshRefineStrategy::SPARSE, "SPARSE"},
        {TerraScape::MeshRefineStrategy::HYBRID, "HYBRID (optimized)"},
        {TerraScape::MeshRefineStrategy::AUTO, "AUTO"}
    };
    
    for (const auto& [strategy, name] : strategies) {
        std::cout << "\n--- Testing " << name << " strategy ---" << std::endl;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        auto mesh = TerraScape::grid_to_mesh(
            width, height, elevations.data(),
            40.0f,   // error threshold
            1000,    // point limit
            strategy
        );
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        std::cout << "Result: " << duration.count() << "ms, "
                  << mesh.vertices.size() << " vertices, " 
                  << mesh.triangles.size() << " triangles" << std::endl;
        
        if (mesh.triangles.empty()) {
            std::cout << "Warning: No triangles generated (triangulation may have failed)" << std::endl;
        }
    }
    
    return 0;
}