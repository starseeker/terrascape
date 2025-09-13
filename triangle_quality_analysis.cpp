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

void analyzeTriangleQuality(const terrascape::Mesh& mesh, const std::string& density_label) {
    std::cout << "\n=== Triangle Quality Analysis for " << density_label << " ===" << std::endl;
    std::cout << "Total triangles: " << mesh.triangles.size() << std::endl;
    
    if (mesh.triangles.empty()) {
        std::cout << "No triangles to analyze." << std::endl;
        return;
    }
    
    std::vector<double> min_angles, aspect_ratios, areas;
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
        areas.push_back(quality.area);
        
        // Count poor quality triangles
        if (quality.min_angle_deg < 20.0) poor_angle_count++;
        if (quality.aspect_ratio > 6.0) poor_aspect_count++;
        if (quality.area < 0.5) tiny_area_count++;
    }
    
    if (min_angles.empty()) {
        std::cout << "All triangles are degenerate!" << std::endl;
        return;
    }
    
    // Sort for statistics
    std::sort(min_angles.begin(), min_angles.end());
    std::sort(aspect_ratios.begin(), aspect_ratios.end());
    std::sort(areas.begin(), areas.end());
    
    // Calculate statistics
    auto percentile = [](const std::vector<double>& v, double p) -> double {
        if (v.empty()) return 0.0;
        size_t idx = static_cast<size_t>(p * (v.size() - 1));
        return v[idx];
    };
    
    std::cout << "\nMinimum Angles (degrees):" << std::endl;
    std::cout << "  Min: " << min_angles.front() << std::endl;
    std::cout << "  5th percentile: " << percentile(min_angles, 0.05) << std::endl;
    std::cout << "  Median: " << percentile(min_angles, 0.50) << std::endl;
    std::cout << "  95th percentile: " << percentile(min_angles, 0.95) << std::endl;
    std::cout << "  Max: " << min_angles.back() << std::endl;
    
    std::cout << "\nAspect Ratios:" << std::endl;
    std::cout << "  Min: " << aspect_ratios.front() << std::endl;
    std::cout << "  5th percentile: " << percentile(aspect_ratios, 0.05) << std::endl;
    std::cout << "  Median: " << percentile(aspect_ratios, 0.50) << std::endl;
    std::cout << "  95th percentile: " << percentile(aspect_ratios, 0.95) << std::endl;
    std::cout << "  Max: " << aspect_ratios.back() << std::endl;
    
    std::cout << "\nTriangle Areas:" << std::endl;
    std::cout << "  Min: " << areas.front() << std::endl;
    std::cout << "  5th percentile: " << percentile(areas, 0.05) << std::endl;
    std::cout << "  Median: " << percentile(areas, 0.50) << std::endl;
    std::cout << "  95th percentile: " << percentile(areas, 0.95) << std::endl;
    std::cout << "  Max: " << areas.back() << std::endl;
    
    std::cout << "\nQuality Issues:" << std::endl;
    std::cout << "  Degenerate triangles: " << degenerate_count << std::endl;
    std::cout << "  Poor angles (< 20°): " << poor_angle_count << " (" 
              << (100.0 * poor_angle_count / mesh.triangles.size()) << "%)" << std::endl;
    std::cout << "  Poor aspect ratio (> 6.0): " << poor_aspect_count << " (" 
              << (100.0 * poor_aspect_count / mesh.triangles.size()) << "%)" << std::endl;
    std::cout << "  Tiny area (< 0.5): " << tiny_area_count << " (" 
              << (100.0 * tiny_area_count / mesh.triangles.size()) << "%)" << std::endl;
}

int main() {
    int width, height;
    std::vector<float> elevations;
    
    if (!readPGM("crater.pgm", width, height, elevations)) {
        std::cerr << "Failed to read crater.pgm" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded crater.pgm: " << width << "x" << height << " pixels" << std::endl;
    
    // Test different density levels
    std::vector<double> test_densities = {0.0, 0.5, 1.0};
    
    for (double density : test_densities) {
        terrascape::Mesh mesh;
        
        auto start = std::chrono::high_resolution_clock::now();
        terrascape::triangulateRegionGrowing(elevations.data(), width, height, mesh, density);
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "DENSITY " << density << " RESULTS" << std::endl;
        std::cout << "Time: " << duration << "ms" << std::endl;
        std::cout << "Vertices: " << mesh.vertices.size() << std::endl;
        std::cout << "Triangles: " << mesh.triangles.size() << std::endl;
        
        analyzeTriangleQuality(mesh, "Density " + std::to_string(density));
    }
    
    return 0;
}