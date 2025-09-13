#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include "greedy_cuts.hpp"

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
        int val;
        file >> val;
        elevations[i] = static_cast<float>(val);
    }
    
    return true;
}

int main() {
    int width, height;
    std::vector<float> elevations;
    
    if (!readPGM("crater.pgm", width, height, elevations)) {
        std::cerr << "Failed to read crater.pgm" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded crater.pgm: " << width << "x" << height << " pixels" << std::endl;
    
    terrascape::GreedyCutsOptions opt;
    opt.base_error_threshold = 0.2;
    opt.use_region_growing = true;  // Enable region-growing approach
    opt.region_merge_threshold = 10.0; // Allow larger height differences for testing
    
    terrascape::Mesh mesh;
    
    auto start = std::chrono::high_resolution_clock::now();
    terrascape::triangulateGreedyCuts(elevations.data(), width, height, nullptr, opt, mesh);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    std::cout << "Region-growing triangulation completed in " << duration << "ms" << std::endl;
    std::cout << "Vertices: " << mesh.vertices.size() << std::endl;
    std::cout << "Triangles: " << mesh.triangles.size() << std::endl;
    
    return 0;
}