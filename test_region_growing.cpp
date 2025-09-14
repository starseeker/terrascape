#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <cstring>
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

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --abs-tolerance <mm>      Absolute tolerance in millimeters (default: 0.1)" << std::endl;
    std::cout << "  --rel-tolerance <frac>    Relative tolerance as fraction (default: 0.01)" << std::endl;
    std::cout << "  --norm-tolerance <deg>    Normal angle tolerance in degrees (default: 15.0)" << std::endl;
    std::cout << "  --volume-delta <pct>      Max volume delta percentage (default: 10.0)" << std::endl;
    std::cout << "  --base-error <val>        Base error threshold (default: 0.2)" << std::endl;
    std::cout << "  --region-threshold <val>  Region merge threshold (default: 10.0)" << std::endl;
    std::cout << "  --help                    Show this help message" << std::endl;
}

int main(int argc, char* argv[]) {
    // Default tolerance values
    terrascape::GreedyCutsOptions opt;
    opt.base_error_threshold = 0.2;
    opt.use_region_growing = true;
    opt.region_merge_threshold = 10.0;
    // BRL-CAD tolerance defaults are already set in the struct
    
    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--abs-tolerance") == 0 && i + 1 < argc) {
            opt.abs_tolerance_mm = atof(argv[++i]);
        } else if (strcmp(argv[i], "--rel-tolerance") == 0 && i + 1 < argc) {
            opt.rel_tolerance = atof(argv[++i]);
        } else if (strcmp(argv[i], "--norm-tolerance") == 0 && i + 1 < argc) {
            opt.norm_tolerance_deg = atof(argv[++i]);
        } else if (strcmp(argv[i], "--volume-delta") == 0 && i + 1 < argc) {
            opt.volume_delta_pct = atof(argv[++i]);
        } else if (strcmp(argv[i], "--base-error") == 0 && i + 1 < argc) {
            opt.base_error_threshold = atof(argv[++i]);
        } else if (strcmp(argv[i], "--region-threshold") == 0 && i + 1 < argc) {
            opt.region_merge_threshold = atof(argv[++i]);
        }
    }
    
    int width, height;
    std::vector<float> elevations;
    
    if (!readPGM("crater.pgm", width, height, elevations)) {
        std::cerr << "Failed to read crater.pgm" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded crater.pgm: " << width << "x" << height << " pixels" << std::endl;
    
    // Print all settings used for this run
    std::cout << "\n=== BRL-CAD Tolerance Settings ===" << std::endl;
    std::cout << "Absolute tolerance: " << opt.abs_tolerance_mm << " mm" << std::endl;
    std::cout << "Relative tolerance: " << opt.rel_tolerance << " (fraction)" << std::endl;
    std::cout << "Normal tolerance: " << opt.norm_tolerance_deg << " degrees" << std::endl;
    std::cout << "Volume delta tolerance: " << opt.volume_delta_pct << "%" << std::endl;
    std::cout << "Base error threshold: " << opt.base_error_threshold << std::endl;
    std::cout << "Region merge threshold: " << opt.region_merge_threshold << std::endl;
    std::cout << "===============================" << std::endl;
    
    terrascape::Mesh mesh;
    
    auto start = std::chrono::high_resolution_clock::now();
    terrascape::triangulateGreedyCuts(elevations.data(), width, height, nullptr, opt, mesh);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    std::cout << "\nRegion-growing triangulation completed in " << duration << "ms" << std::endl;
    std::cout << "Vertices: " << mesh.vertices.size() << std::endl;
    std::cout << "Triangles: " << mesh.triangles.size() << std::endl;
    
    return 0;
}