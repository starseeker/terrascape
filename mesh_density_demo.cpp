#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <iomanip>
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

void runDensityTest(double density, const std::vector<float>& elevations, int width, int height) {
    std::cout << "\n=== Mesh Density: " << std::fixed << std::setprecision(2) << density;
    if (density <= 0.1) std::cout << " (Coarsest)";
    else if (density >= 0.9) std::cout << " (Finest)";
    else if (density == 0.5) std::cout << " (Medium)";
    std::cout << " ===" << std::endl;
    
    terrascape::Mesh mesh;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Use the simplified region-growing interface
    terrascape::triangulateRegionGrowing(elevations.data(), width, height, mesh, density);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    std::cout << "Processing time: " << duration << "ms" << std::endl;
    std::cout << "Vertices: " << mesh.vertices.size() << std::endl;
    std::cout << "Triangles: " << mesh.triangles.size() << std::endl;
    
    if (duration > 0) {
        std::cout << "Triangles per second: " << (mesh.triangles.size() * 1000 / duration) << std::endl;
    }
    
    // Calculate mesh resolution statistics
    if (!mesh.vertices.empty()) {
        double avg_vertex_spacing_x = static_cast<double>(width - 1) / std::sqrt(mesh.vertices.size());
        double avg_vertex_spacing_y = static_cast<double>(height - 1) / std::sqrt(mesh.vertices.size());
        double coverage_ratio = static_cast<double>(mesh.vertices.size()) / (width * height);
        
        std::cout << "Coverage ratio: " << std::fixed << std::setprecision(4) << (coverage_ratio * 100.0) << "%" << std::endl;
        std::cout << "Avg vertex spacing: " << std::setprecision(1) 
                  << avg_vertex_spacing_x << "x" << avg_vertex_spacing_y << " grid units" << std::endl;
    }
}

void runAdvancedTest(const std::vector<float>& elevations, int width, int height) {
    std::cout << "\n=== Advanced: Custom Parameters ===" << std::endl;
    
    terrascape::GreedyCutsOptions opt;
    opt.mesh_density = 0.7;  // High detail
    opt.use_region_growing = true;
    
    // Override some parameters for demonstration
    opt.region_merge_threshold = 2.0;  // Smaller regions
    opt.sampling_step = 2;             // Every other pixel
    opt.base_error_threshold = 0.5;    // Medium tolerance
    
    terrascape::Mesh mesh;
    
    auto start = std::chrono::high_resolution_clock::now();
    terrascape::triangulateGreedyCuts(elevations.data(), width, height, nullptr, opt, mesh);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    std::cout << "Processing time: " << duration << "ms" << std::endl;
    std::cout << "Vertices: " << mesh.vertices.size() << std::endl;
    std::cout << "Triangles: " << mesh.triangles.size() << std::endl;
    std::cout << "Custom region_merge_threshold: " << opt.region_merge_threshold << std::endl;
    std::cout << "Custom sampling_step: " << opt.sampling_step << std::endl;
}

int main() {
    int width, height;
    std::vector<float> elevations;
    
    if (!readPGM("crater.pgm", width, height, elevations)) {
        std::cerr << "Failed to read crater.pgm" << std::endl;
        return 1;
    }
    
    std::cout << "==================================================" << std::endl;
    std::cout << "         TERRASCAPE MESH DENSITY DEMONSTRATION" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "Loaded crater.pgm: " << width << "x" << height << " pixels (" 
              << (width * height) << " total points)" << std::endl;
    
    // Demonstrate the full range of density values
    std::vector<double> density_values = {
        0.0,   // Coarsest possible
        0.1,   // Very coarse
        0.3,   // Coarse
        0.5,   // Medium (default)
        0.7,   // Fine
        0.9,   // Very fine
        1.0    // Finest possible
    };
    
    for (double density : density_values) {
        runDensityTest(density, elevations, width, height);
    }
    
    // Show advanced usage with custom parameters
    runAdvancedTest(elevations, width, height);
    
    std::cout << "\n==================================================" << std::endl;
    std::cout << "SUMMARY:" << std::endl;
    std::cout << "- mesh_density 0.0: Coarsest mesh (large regions, sparse sampling)" << std::endl;
    std::cout << "- mesh_density 0.5: Balanced mesh (medium detail, good performance)" << std::endl;
    std::cout << "- mesh_density 1.0: Finest mesh (small regions, dense sampling)" << std::endl;
    std::cout << "- Use triangulateRegionGrowing(data, width, height, mesh, density)" << std::endl;
    std::cout << "  for simple interface with single density parameter" << std::endl;
    std::cout << "- Use triangulateGreedyCuts() for advanced control over all parameters" << std::endl;
    std::cout << "==================================================" << std::endl;
    
    return 0;
}