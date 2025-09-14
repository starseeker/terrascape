#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "TerraScape.hpp"

// Simple PGM reader (duplicated from main.cpp for independence)
bool readPGMToFloatArray(const char* filename, int& width, int& height, std::vector<float>& elevations) {
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

// Function to find the mesh vertex closest to a given grid position
int findClosestVertex(const TerraScape::MeshResult& mesh, int grid_x, int grid_y) {
    int closest_vertex = -1;
    double min_distance = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        double dx = mesh.vertices[i].x - grid_x;
        double dy = mesh.vertices[i].y - grid_y;
        double distance = dx*dx + dy*dy;
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_vertex = static_cast<int>(i);
        }
    }
    
    return closest_vertex;
}

int main() {
    std::cout << "=== TerraScape Height Field Mapping Validation ===" << std::endl;
    
    // Read the crater.pgm file
    int width, height;
    std::vector<float> elevations;
    if (!readPGMToFloatArray("crater.pgm", width, height, elevations)) {
        return 1;
    }
    
    std::cout << "Loaded crater.pgm: " << width << "x" << height << " pixels" << std::endl;
    
    // Calculate basic statistics
    float min_elev = *std::min_element(elevations.begin(), elevations.end());
    float max_elev = *std::max_element(elevations.begin(), elevations.end());
    float elev_range = max_elev - min_elev;
    
    std::cout << "Elevation range: " << min_elev << " to " << max_elev 
              << " (range: " << elev_range << ")" << std::endl;
    
    // Generate mesh using current TerraScape algorithm
    std::cout << "\nGenerating mesh..." << std::endl;
    TerraScape::MeshResult mesh = TerraScape::grid_to_mesh(width, height, elevations.data(), 1.0f);
    
    std::cout << "Generated mesh: " << mesh.vertices.size() << " vertices, " 
              << mesh.triangles.size() << " triangles" << std::endl;
    
    // Validate height field mapping by checking specific points
    std::cout << "\n=== Height Field Mapping Validation ===" << std::endl;
    
    // Test corner points
    std::vector<std::pair<int, int>> test_points = {
        {0, 0},           // Top-left corner
        {width-1, 0},     // Top-right corner
        {0, height-1},    // Bottom-left corner
        {width-1, height-1}, // Bottom-right corner
        {width/2, height/2}, // Center
        {width/4, height/4}, // Quarter point
        {3*width/4, 3*height/4} // Three-quarter point
    };
    
    bool mapping_accurate = true;
    double total_error = 0.0;
    int valid_tests = 0;
    
    for (const auto& point : test_points) {
        int gx = point.first;
        int gy = point.second;
        
        // Get original elevation from PGM
        size_t pgm_idx = gy * width + gx;
        float original_elevation = elevations[pgm_idx];
        
        // Find closest vertex in mesh
        int closest_vertex = findClosestVertex(mesh, gx, gy);
        if (closest_vertex == -1) continue;
        
        float mesh_elevation = mesh.vertices[closest_vertex].z;
        double error = std::abs(mesh_elevation - original_elevation);
        double relative_error = error / elev_range * 100.0;
        
        std::cout << "Point (" << gx << "," << gy << "): PGM=" << original_elevation 
                  << ", Mesh=" << mesh_elevation << ", Error=" << error 
                  << " (" << relative_error << "% of range)" << std::endl;
        
        total_error += error;
        valid_tests++;
        
        // Flag significant errors (>1% of elevation range)
        if (relative_error > 1.0) {
            std::cout << "  WARNING: High error at this point!" << std::endl;
            mapping_accurate = false;
        }
    }
    
    // Summary
    std::cout << "\n=== Validation Summary ===" << std::endl;
    if (valid_tests > 0) {
        double average_error = total_error / valid_tests;
        double average_relative_error = average_error / elev_range * 100.0;
        
        std::cout << "Average absolute error: " << average_error << std::endl;
        std::cout << "Average relative error: " << average_relative_error << "% of elevation range" << std::endl;
        
        if (mapping_accurate && average_relative_error < 0.5) {
            std::cout << "✓ PASS: Height field mapping appears accurate" << std::endl;
        } else if (average_relative_error < 2.0) {
            std::cout << "⚠ PARTIAL: Height field mapping has minor inaccuracies" << std::endl;
        } else {
            std::cout << "✗ FAIL: Height field mapping has significant errors" << std::endl;
        }
    }
    
    // Check mesh vertex elevation range
    std::cout << "\n=== Mesh Elevation Range Check ===" << std::endl;
    if (!mesh.vertices.empty()) {
        float mesh_min_z = mesh.vertices[0].z;
        float mesh_max_z = mesh.vertices[0].z;
        
        for (const auto& vertex : mesh.vertices) {
            mesh_min_z = std::min(mesh_min_z, vertex.z);
            mesh_max_z = std::max(mesh_max_z, vertex.z);
        }
        
        std::cout << "PGM elevation range: " << min_elev << " to " << max_elev << std::endl;
        std::cout << "Mesh elevation range: " << mesh_min_z << " to " << mesh_max_z << std::endl;
        
        double min_diff = std::abs(mesh_min_z - min_elev);
        double max_diff = std::abs(mesh_max_z - max_elev);
        
        if (min_diff < elev_range * 0.01 && max_diff < elev_range * 0.01) {
            std::cout << "✓ PASS: Mesh preserves elevation range accurately" << std::endl;
        } else {
            std::cout << "⚠ WARNING: Mesh elevation range differs from PGM data" << std::endl;
            std::cout << "  Min difference: " << min_diff << " (" << min_diff/elev_range*100 << "%)" << std::endl;
            std::cout << "  Max difference: " << max_diff << " (" << max_diff/elev_range*100 << "%)" << std::endl;
        }
    }
    
    return 0;
}