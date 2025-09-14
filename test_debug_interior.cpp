#include <iostream>
#include <vector>
#include <cmath>
#include "TerraScape.hpp"

// Debug version of the interior wall detection
void debug_interior_wall_detection(const float* elevations, int width, int height, float z_base) {
    std::cout << "Debug: Checking for zero-height interior points..." << std::endl;
    
    std::vector<std::pair<int, int>> zero_interior_points;
    
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            float center_elevation = elevations[y * width + x];
            
            std::cout << "  Point (" << x << "," << y << ") elevation: " << center_elevation;
            std::cout << ", z_base: " << z_base;
            std::cout << ", diff: " << std::abs(center_elevation - z_base);
            
            // Check if this point is at zero/base elevation
            if (std::abs(center_elevation - z_base) > 0.1f) {
                std::cout << " [NOT ZERO-HEIGHT]" << std::endl;
                continue;
            }
            
            std::cout << " [ZERO-HEIGHT]";
            
            // Check if it's surrounded by elevated terrain
            bool has_elevated_neighbor = false;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = x + dx, ny = y + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        float neighbor_elevation = elevations[ny * width + nx];
                        if (neighbor_elevation > z_base + 0.1f) {
                            has_elevated_neighbor = true;
                            std::cout << " - Found elevated neighbor at (" << nx << "," << ny << ") with elevation " << neighbor_elevation;
                            break;
                        }
                    }
                }
                if (has_elevated_neighbor) break;
            }
            
            if (has_elevated_neighbor) {
                std::cout << " [INTERIOR HOLE DETECTED]";
                zero_interior_points.push_back({x, y});
            } else {
                std::cout << " [NO ELEVATED NEIGHBORS]";
            }
            
            std::cout << std::endl;
        }
    }
    
    std::cout << "Found " << zero_interior_points.size() << " zero-height interior points:" << std::endl;
    for (const auto& point : zero_interior_points) {
        std::cout << "  (" << point.first << "," << point.second << ")" << std::endl;
    }
}

bool test_debug_mixed_elevation() {
    std::cout << "=== Debug Mixed Elevation Test ===" << std::endl;
    
    // Create terrain with both zero and non-zero heights in the surface mesh
    int width = 4, height = 4;
    std::vector<float> elevations = {
        1.0f, 1.0f, 1.0f, 1.0f,  // row 0: elevated
        1.0f, 0.0f, 0.0f, 1.0f,  // row 1: hole in center
        1.0f, 0.0f, 0.0f, 1.0f,  // row 2: hole in center
        1.0f, 1.0f, 1.0f, 1.0f   // row 3: elevated
    };
    
    float z_base = 0.0f; // Base at zero level
    
    std::cout << "Test terrain (4x4 grid with mixed elevations):" << std::endl;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::cout << elevations[y * width + x] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    
    // Debug the interior wall detection
    debug_interior_wall_detection(elevations.data(), width, height, z_base);
    
    return true;
}

int main() {
    test_debug_mixed_elevation();
    return 0;
}