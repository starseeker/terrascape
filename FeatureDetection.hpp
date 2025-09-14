#pragma once

/*
 * FeatureDetection.hpp - Terrain Feature Detection Module
 *
 * Provides gradient and curvature-based methods to identify terrain features
 * such as ridges, valleys, and terrain breaks. Used to inform mesh segmentation
 * and graph-based optimization in TerraScape.
 */

#include <vector>
#include <cmath>
#include <algorithm>
#include <array>

namespace TerraScape {

// Forward declaration for ElevationGrid structure compatibility
struct ElevationGrid {
    const float* data;
    int width;
    int height;
    
    ElevationGrid(const float* elevation_data, int w, int h) 
        : data(elevation_data), width(w), height(h) {}
    
    // Safe elevation access with bounds checking
    float get_elevation(int x, int y) const {
        if (x < 0 || x >= width || y < 0 || y >= height) {
            return 0.0f; // Return 0 for out-of-bounds
        }
        return data[y * width + x];
    }
};

namespace FeatureDetection {

// Feature detection parameters
struct FeatureDetectionOptions {
    // Gradient computation parameters
    double gradient_threshold = 0.1;      // Threshold for significant gradients
    bool use_sobel_operator = true;       // Use Sobel vs simple difference
    
    // Curvature computation parameters  
    double curvature_threshold = 0.05;    // Threshold for significant curvature
    bool enable_curvature = true;         // Enable curvature-based detection
    
    // Feature combination parameters
    double gradient_weight = 0.7;         // Weight for gradient in final feature map
    double curvature_weight = 0.3;        // Weight for curvature in final feature map
    
    // Smoothing parameters
    bool enable_smoothing = true;         // Enable Gaussian smoothing of feature map
    double smoothing_sigma = 1.0;         // Standard deviation for Gaussian smoothing
};

// Compute gradient magnitude using Sobel operator
inline std::vector<std::vector<double>> compute_gradient_map(const ElevationGrid& grid, 
                                                            const FeatureDetectionOptions& opts) {
    std::vector<std::vector<double>> gradient_map(grid.height, std::vector<double>(grid.width, 0.0));
    
    for (int y = 1; y < grid.height - 1; y++) {
        for (int x = 1; x < grid.width - 1; x++) {
            double gx, gy;
            
            if (opts.use_sobel_operator) {
                // Sobel operator for more robust gradient estimation
                gx = -1.0 * grid.get_elevation(x-1, y-1) + 1.0 * grid.get_elevation(x+1, y-1) +
                     -2.0 * grid.get_elevation(x-1, y  ) + 2.0 * grid.get_elevation(x+1, y  ) +
                     -1.0 * grid.get_elevation(x-1, y+1) + 1.0 * grid.get_elevation(x+1, y+1);
                
                gy = -1.0 * grid.get_elevation(x-1, y-1) - 2.0 * grid.get_elevation(x, y-1) - 1.0 * grid.get_elevation(x+1, y-1) +
                      1.0 * grid.get_elevation(x-1, y+1) + 2.0 * grid.get_elevation(x, y+1) + 1.0 * grid.get_elevation(x+1, y+1);
            } else {
                // Simple central difference
                gx = grid.get_elevation(x+1, y) - grid.get_elevation(x-1, y);
                gy = grid.get_elevation(x, y+1) - grid.get_elevation(x, y-1);
            }
            
            gradient_map[y][x] = std::sqrt(gx * gx + gy * gy);
        }
    }
    
    return gradient_map;
}

// Compute curvature using Laplacian operator
inline std::vector<std::vector<double>> compute_curvature_map(const ElevationGrid& grid,
                                                             const FeatureDetectionOptions& opts) {
    std::vector<std::vector<double>> curvature_map(grid.height, std::vector<double>(grid.width, 0.0));
    
    for (int y = 1; y < grid.height - 1; y++) {
        for (int x = 1; x < grid.width - 1; x++) {
            // Discrete Laplacian operator (4-connected)
            double center = grid.get_elevation(x, y);
            double laplacian = grid.get_elevation(x-1, y) + grid.get_elevation(x+1, y) + 
                              grid.get_elevation(x, y-1) + grid.get_elevation(x, y+1) - 4.0 * center;
            
            curvature_map[y][x] = std::abs(laplacian);
        }
    }
    
    return curvature_map;
}

// Apply Gaussian smoothing to feature map
inline void smooth_feature_map(std::vector<std::vector<double>>& feature_map, 
                              double sigma) {
    if (sigma <= 0.0) return;
    
    int height = static_cast<int>(feature_map.size());
    int width = static_cast<int>(feature_map[0].size());
    
    // Simple 3x3 Gaussian kernel approximation
    std::array<std::array<double, 3>, 3> kernel;
    double sum = 0.0;
    
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            double value = std::exp(-(dx*dx + dy*dy) / (2.0 * sigma * sigma));
            kernel[dy+1][dx+1] = value;
            sum += value;
        }
    }
    
    // Normalize kernel
    for (int dy = 0; dy < 3; dy++) {
        for (int dx = 0; dx < 3; dx++) {
            kernel[dy][dx] /= sum;
        }
    }
    
    // Apply convolution
    std::vector<std::vector<double>> smoothed(height, std::vector<double>(width, 0.0));
    
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            double filtered_value = 0.0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    filtered_value += feature_map[y + dy][x + dx] * kernel[dy + 1][dx + 1];
                }
            }
            smoothed[y][x] = filtered_value;
        }
    }
    
    feature_map = std::move(smoothed);
}

// Normalize feature map to [0, 1] range
inline void normalize_feature_map(std::vector<std::vector<double>>& feature_map) {
    double min_val = std::numeric_limits<double>::max();
    double max_val = std::numeric_limits<double>::lowest();
    
    // Find min and max values
    for (const auto& row : feature_map) {
        for (double val : row) {
            min_val = std::min(min_val, val);
            max_val = std::max(max_val, val);
        }
    }
    
    // Normalize to [0, 1] range
    if (max_val > min_val) {
        double range = max_val - min_val;
        for (auto& row : feature_map) {
            for (double& val : row) {
                val = (val - min_val) / range;
            }
        }
    }
}

// Main API function: compute feature map from elevation grid
inline std::vector<std::vector<double>> compute_feature_map(const ElevationGrid& grid,
                                                           const FeatureDetectionOptions& opts = FeatureDetectionOptions{}) {
    // Compute gradient-based features
    auto gradient_map = compute_gradient_map(grid, opts);
    
    // Compute curvature-based features if enabled
    std::vector<std::vector<double>> curvature_map;
    if (opts.enable_curvature) {
        curvature_map = compute_curvature_map(grid, opts);
    }
    
    // Combine gradient and curvature features
    std::vector<std::vector<double>> feature_map(grid.height, std::vector<double>(grid.width, 0.0));
    
    for (int y = 0; y < grid.height; y++) {
        for (int x = 0; x < grid.width; x++) {
            double feature_strength = opts.gradient_weight * gradient_map[y][x];
            
            if (opts.enable_curvature && !curvature_map.empty()) {
                feature_strength += opts.curvature_weight * curvature_map[y][x];
            }
            
            feature_map[y][x] = feature_strength;
        }
    }
    
    // Apply smoothing if enabled
    if (opts.enable_smoothing) {
        smooth_feature_map(feature_map, opts.smoothing_sigma);
    }
    
    // Normalize to [0, 1] range
    normalize_feature_map(feature_map);
    
    return feature_map;
}

// Utility function to identify strong features above threshold
inline std::vector<std::pair<int, int>> find_strong_features(const std::vector<std::vector<double>>& feature_map,
                                                           double threshold = 0.5) {
    std::vector<std::pair<int, int>> feature_points;
    
    int height = static_cast<int>(feature_map.size());
    int width = static_cast<int>(feature_map[0].size());
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (feature_map[y][x] > threshold) {
                feature_points.emplace_back(x, y);
            }
        }
    }
    
    return feature_points;
}

} // namespace FeatureDetection
} // namespace TerraScape