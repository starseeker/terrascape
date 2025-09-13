#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <cmath>
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

// Calculate triangle quality metrics
struct TriangleQuality {
    double min_angle_deg;
    double max_angle_deg;
    double aspect_ratio;
    double area;
    bool is_degenerate;
};

TriangleQuality analyzeTriangle(const std::array<double,3>& v0,
                               const std::array<double,3>& v1,
                               const std::array<double,3>& v2) {
    TriangleQuality quality;
    
    // Calculate area
    quality.area = 0.5 * std::abs((v1[0] - v0[0]) * (v2[1] - v0[1]) - 
                                 (v2[0] - v0[0]) * (v1[1] - v0[1]));
    
    // Check for degeneracy
    quality.is_degenerate = quality.area < 1e-12;
    
    if (quality.is_degenerate) {
        quality.min_angle_deg = 0.0;
        quality.max_angle_deg = 180.0;
        quality.aspect_ratio = std::numeric_limits<double>::infinity();
        return quality;
    }
    
    // Calculate edge lengths
    double a = std::hypot(v0[0] - v1[0], v0[1] - v1[1]);
    double b = std::hypot(v1[0] - v2[0], v1[1] - v2[1]);
    double c = std::hypot(v2[0] - v0[0], v2[1] - v0[1]);
    
    // Calculate aspect ratio
    double longest = std::max({a, b, c});
    double shortest = std::max(std::min({a, b, c}), 1e-12);
    quality.aspect_ratio = longest / shortest;
    
    // Calculate angles using law of cosines
    auto calc_angle = [](double a, double b, double c) -> double {
        double cos_angle = (a*a + b*b - c*c) / (2*a*b);
        cos_angle = std::clamp(cos_angle, -1.0, 1.0);
        return std::acos(cos_angle) * 180.0 / M_PI;
    };
    
    double angle_A = calc_angle(b, c, a);
    double angle_B = calc_angle(a, c, b);
    double angle_C = calc_angle(a, b, c);
    
    quality.min_angle_deg = std::min({angle_A, angle_B, angle_C});
    quality.max_angle_deg = std::max({angle_A, angle_B, angle_C});
    
    return quality;
}

void summarizeQuality(const terrascape::Mesh& mesh, const std::string& label) {
    std::vector<double> min_angles, aspect_ratios;
    int degenerate_count = 0;
    int poor_angle_count = 0;
    int poor_aspect_count = 0;
    int tiny_area_count = 0;
    
    for (const auto& tri : mesh.triangles) {
        const auto& v0 = mesh.vertices[tri[0]];
        const auto& v1 = mesh.vertices[tri[1]];
        const auto& v2 = mesh.vertices[tri[2]];
        
        TriangleQuality quality = analyzeTriangle(v0, v1, v2);
        
        if (quality.is_degenerate) {
            degenerate_count++;
            continue;
        }
        
        min_angles.push_back(quality.min_angle_deg);
        aspect_ratios.push_back(quality.aspect_ratio);
        
        // Count poor quality triangles
        if (quality.min_angle_deg < 20.0) poor_angle_count++;
        if (quality.aspect_ratio > 6.0) poor_aspect_count++;
        if (quality.area < 0.5) tiny_area_count++;
    }
    
    if (!min_angles.empty()) {
        std::sort(min_angles.begin(), min_angles.end());
        std::sort(aspect_ratios.begin(), aspect_ratios.end());
    }
    
    std::cout << label << ":" << std::endl;
    std::cout << "  Triangles: " << mesh.triangles.size() << std::endl;
    std::cout << "  Degenerate: " << degenerate_count << std::endl;
    std::cout << "  Poor angles (<20°): " << poor_angle_count << " (" 
              << (mesh.triangles.empty() ? 0.0 : 100.0 * poor_angle_count / mesh.triangles.size()) << "%)" << std::endl;
    std::cout << "  Poor aspect ratio (>6.0): " << poor_aspect_count << " (" 
              << (mesh.triangles.empty() ? 0.0 : 100.0 * poor_aspect_count / mesh.triangles.size()) << "%)" << std::endl;
    if (!min_angles.empty()) {
        std::cout << "  Min angle range: " << min_angles.front() << "° - " << min_angles.back() << "°" << std::endl;
        std::cout << "  Aspect ratio range: " << aspect_ratios.front() << " - " << aspect_ratios.back() << std::endl;
    }
    std::cout << std::endl;
}

int main() {
    int width, height;
    std::vector<float> elevations;
    
    if (!readPGM("crater.pgm", width, height, elevations)) {
        std::cerr << "Failed to read crater.pgm" << std::endl;
        return 1;
    }
    
    std::cout << "=== Triangle Quality Comparison: crater.pgm (" << width << "x" << height << ") ===" << std::endl;
    
    // Test at density 0.5 with different quality settings
    double test_density = 0.5;
    
    // Test with quality filtering disabled (original behavior)
    terrascape::GreedyCutsOptions opt_unfiltered;
    opt_unfiltered.mesh_density = test_density;
    opt_unfiltered.use_region_growing = true;
    opt_unfiltered.enable_quality_filtering = false;
    
    terrascape::Mesh mesh_unfiltered;
    auto start = std::chrono::high_resolution_clock::now();
    terrascape::triangulateRegionGrowing(elevations.data(), width, height, nullptr, opt_unfiltered, mesh_unfiltered);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_unfiltered = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    // Test with quality filtering enabled (improved behavior)
    terrascape::GreedyCutsOptions opt_filtered;
    opt_filtered.mesh_density = test_density;
    opt_filtered.use_region_growing = true;
    opt_filtered.enable_quality_filtering = true;
    
    terrascape::Mesh mesh_filtered;
    start = std::chrono::high_resolution_clock::now();
    terrascape::triangulateRegionGrowing(elevations.data(), width, height, nullptr, opt_filtered, mesh_filtered);
    end = std::chrono::high_resolution_clock::now();
    auto duration_filtered = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    std::cout << "\nResults for mesh_density = " << test_density << ":\n" << std::endl;
    
    std::cout << "Without Quality Filtering (time: " << duration_unfiltered << "ms):" << std::endl;
    summarizeQuality(mesh_unfiltered, "  Raw Triangulation");
    
    std::cout << "With Quality Filtering (time: " << duration_filtered << "ms):" << std::endl;
    summarizeQuality(mesh_filtered, "  Quality-Filtered Triangulation");
    
    // Performance comparison
    std::cout << "Performance Impact:" << std::endl;
    std::cout << "  Time increase: " << (duration_filtered - duration_unfiltered) << "ms" << std::endl;
    std::cout << "  Triangle count change: " << (int)mesh_filtered.triangles.size() - (int)mesh_unfiltered.triangles.size() << std::endl;
    
    return 0;
}