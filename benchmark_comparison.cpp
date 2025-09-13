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

void runTest(const std::string& name, terrascape::GreedyCutsOptions& opt, 
             const std::vector<float>& elevations, int width, int height) {
    std::cout << "\n=== " << name << " ===" << std::endl;
    
    terrascape::Mesh mesh;
    
    auto start = std::chrono::high_resolution_clock::now();
    terrascape::triangulateGreedyCuts(elevations.data(), width, height, nullptr, opt, mesh);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    std::cout << "Triangulation completed in " << duration << "ms" << std::endl;
    std::cout << "Vertices: " << mesh.vertices.size() << std::endl;
    std::cout << "Triangles: " << mesh.triangles.size() << std::endl;
    std::cout << "Triangles per second: " << (mesh.triangles.size() * 1000 / std::max(1L, duration)) << std::endl;
}

int main() {
    int width, height;
    std::vector<float> elevations;
    
    if (!readPGM("crater.pgm", width, height, elevations)) {
        std::cerr << "Failed to read crater.pgm" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded crater.pgm: " << width << "x" << height << " pixels" << std::endl;
    
    // Test 1: Original greedy cuts with pre-computed complexity
    terrascape::GreedyCutsOptions opt1;
    opt1.base_error_threshold = 0.2;
    opt1.use_precomputed_complexity = true;
    opt1.use_region_growing = false;
    runTest("Greedy Cuts with Pre-computed Complexity", opt1, elevations, width, height);
    
    // Test 2: Region-growing approach  
    terrascape::GreedyCutsOptions opt2;
    opt2.base_error_threshold = 0.2;
    opt2.use_region_growing = true;
    opt2.region_merge_threshold = 10.0;
    runTest("Region-Growing Approach", opt2, elevations, width, height);
    
    // Test 3: Region-growing with tighter threshold
    terrascape::GreedyCutsOptions opt3;
    opt3.base_error_threshold = 0.2;
    opt3.use_region_growing = true;
    opt3.region_merge_threshold = 5.0;
    runTest("Region-Growing (Tighter Threshold)", opt3, elevations, width, height);
    
    return 0;
}