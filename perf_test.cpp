#include <iostream>
#include <chrono>
#include <fstream>
#include <vector>
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
        float val;
        file >> val;
        elevations[i] = val;
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
    
    std::cout << "Loaded crater.pgm: " << width << "x" << height << " = " << (width*height) << " points" << std::endl;
    
    // Test different configurations
    std::vector<std::pair<std::string, terrascape::GreedyCutsOptions>> configs = {
        {"Default with optimizations", [](){
            terrascape::GreedyCutsOptions opt;
            opt.base_error_threshold = 1.0;
            opt.use_localized_error = false; // Disable for performance
            opt.max_initial_iterations = 32000; // 2000 * 16 for 154K points
            return opt;
        }()},
        {"Relaxed for speed", [](){
            terrascape::GreedyCutsOptions opt;
            opt.base_error_threshold = 2.0; // Higher threshold
            opt.use_localized_error = false; 
            opt.max_initial_iterations = 16000; 
            opt.min_angle_deg = 5.0;
            opt.max_aspect_ratio = 15.0;
            opt.min_area = 0.05;
            return opt;
        }()},
        {"Very fast", [](){
            terrascape::GreedyCutsOptions opt;
            opt.base_error_threshold = 5.0; // Much higher threshold
            opt.use_localized_error = false; 
            opt.max_initial_iterations = 8000; 
            opt.min_angle_deg = 3.0;
            opt.max_aspect_ratio = 20.0;
            opt.min_area = 0.01;
            opt.max_refinement_passes = 1;
            return opt;
        }()}
    };
    
    for (const auto& [name, opt] : configs) {
        std::cout << "\n=== Testing: " << name << " ===" << std::endl;
        
        terrascape::Mesh mesh;
        
        auto start = std::chrono::high_resolution_clock::now();
        terrascape::triangulateGreedyCuts(elevations.data(), width, height, nullptr, opt, mesh);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "  Time: " << duration.count() << "ms" << std::endl;
        std::cout << "  Vertices: " << mesh.vertices.size() << std::endl;
        std::cout << "  Triangles: " << mesh.triangles.size() << std::endl;
        std::cout << "  Reduction: " << (100.0 * (1.0 - double(mesh.triangles.size()) / (2.0 * (width-1) * (height-1)))) << "%" << std::endl;
    }
    
    return 0;
}