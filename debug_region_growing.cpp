#include "TerraScape.hpp"
#include <iostream>
#include <fstream>
#include <vector>

// Simple PGM reader
bool readPGM(const char* filename, int& width, int& height, std::vector<float>& elevations) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Error: Cannot open " << filename << std::endl;
        return false;
    }
    
    char magicP, magicNum;
    int maxval;
    
    file >> magicP >> magicNum >> width >> height >> maxval;
    
    if (magicP != 'P' || (magicNum != '2' && magicNum != '5')) {
        std::cerr << "Error: Not a valid PGM file" << std::endl;
        return false;
    }
    
    elevations.resize(width * height);
    
    if (magicNum == '2') {
        // Textual PGM
        for (int i = 0; i < width * height; ++i) {
            float val;
            file >> val;
            elevations[i] = val;
        }
    } else {
        // Binary PGM
        char newline;
        file.get(newline); // consume the newline after maxval
        for (int i = 0; i < width * height; ++i) {
            unsigned char val;
            file.read(reinterpret_cast<char*>(&val), 1);
            elevations[i] = float(val);
        }
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
    std::cout << "Total cells: " << (width * height) << std::endl;
    
    // Use exact same settings as the failing test_region_growing command
    TerraScape::RegionGrowingOptions opt;
    opt.abs_tolerance_mm = 0.1;     // default from struct
    opt.rel_tolerance = 0.01;       // default from struct  
    opt.base_error_threshold = 0.2;
    opt.region_merge_threshold = 10.0;
    opt.volume_delta_pct = 10.0;
    
    std::cout << "Before region_growing call:" << std::endl;
    std::cout << "  mesh_density: " << opt.mesh_density << std::endl;
    std::cout << "  abs_tolerance_mm: " << opt.abs_tolerance_mm << std::endl;
    std::cout << "  rel_tolerance: " << opt.rel_tolerance << std::endl;
    std::cout << "  region_merge_threshold: " << opt.region_merge_threshold << std::endl;
    std::cout << "  base_error_threshold: " << opt.base_error_threshold << std::endl;
    
    std::cout << "\n=== Calling region_growing_triangulation_advanced ===\n" << std::endl;
    
    auto mesh = TerraScape::region_growing_triangulation_advanced(elevations.data(), width, height, nullptr, opt);
    
    std::cout << "\n=== Results ===\n" << std::endl;
    std::cout << "Vertices: " << mesh.vertices.size() << std::endl;
    std::cout << "Triangles: " << mesh.triangles.size() << std::endl;
    std::cout << "Vertex density: " << (static_cast<double>(mesh.vertices.size()) / (width * height)) << std::endl;
    
    if (mesh.triangles.size() == 0 && mesh.vertices.size() > 0) {
        std::cout << "\n❌ PROBLEM: Got vertices but no triangles - triangulation failed!" << std::endl;
    }
    
    return 0;
}