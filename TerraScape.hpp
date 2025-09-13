#pragma once

/*
 * TerraScape - Advanced Terrain Mesh Generation Library
 *
 * This library provides efficient grid-to-mesh conversion with advanced
 * triangulation and refinement algorithms.
 *
 * Greedy Cuts advancing front triangulation is used for grid heightfields,
 * with adaptive error control, mesh-quality constraints, and multi-pass refinement.
 */

#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <memory>
#include <iostream>
#include <unordered_set>
#include <set>
#include <map>
#include <string>
#include <stdexcept>
#include <cstdint>

#include "greedy_cuts.hpp"

#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#elif defined(__APPLE__)
#include <sys/types.h>
#include <sys/sysctl.h>
#endif

namespace TerraScape {

// --- Helper: RAM detection (platform-specific) ---
inline size_t get_available_ram_bytes() {
#if defined(_WIN32)
    MEMORYSTATUSEX status;
    status.dwLength = sizeof(status);
    GlobalMemoryStatusEx(&status);
    return status.ullAvailPhys;
#elif defined(__linux__)
    long pages = sysconf(_SC_AVPHYS_PAGES);
    long page_size = sysconf(_SC_PAGE_SIZE);
    return size_t(pages) * size_t(page_size);
#elif defined(__APPLE__)
    int64_t mem = 0;
    size_t len = sizeof(mem);
    sysctlbyname("hw.memsize", &mem, &len, NULL, 0);
    return size_t(mem);
#else
    // Fallback: Assume 1GB available
    return size_t(1) << 30;
#endif
}

// --- Core data structures ---
struct Vertex {
    float x, y, z;
};

struct Triangle {
    int v0, v1, v2;
};

struct MeshResult {
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
    bool is_volumetric = false;  // Indicates if this is a volumetric (closed manifold) mesh
};

// Result structure for volumetric mesh generation that separates positive and negative volumes
struct VolumetricMeshResult {
    MeshResult positive_volume;  // Mesh for areas where surface height > z_base
    MeshResult negative_volume;  // Mesh for areas where surface height < z_base (with reversed normals)
    bool has_positive_volume = false;
    bool has_negative_volume = false;
};

// --- Helper: Enhanced grid accessor (kept for potential future use) ---
template<typename T>
class Grid {
public:
    int width, height;
    const T* data;

    Grid(int w, int h, const T* d) : width(w), height(h), data(d) {}

    T operator()(int x, int y) const {
        return data[y * width + x];
    }

    // Bilinear interpolation
    float interp(float x, float y) const {
        int ix = std::max(0, std::min(int(x), width - 2));
        int iy = std::max(0, std::min(int(y), height - 2));
        float fx = x - ix, fy = y - iy;
        float h00 = float((*this)(ix, iy)), h10 = float((*this)(ix + 1, iy));
        float h01 = float((*this)(ix, iy + 1)), h11 = float((*this)(ix + 1, iy + 1));
        return h00 * (1 - fx) * (1 - fy) + h10 * fx * (1 - fy) +
               h01 * (1 - fx) * fy + h11 * fx * fy;
    }
};

// --- Utility: barycentric triangle routines (kept for potential future use) ---
inline bool point_in_triangle(float px, float py,
                              const Vertex& v0, const Vertex& v1, const Vertex& v2)
{
    float dX = px - v2.x;
    float dY = py - v2.y;
    float dX21 = v2.x - v1.x;
    float dY12 = v1.y - v2.y;
    float D = (v0.x - v2.x) * dY12 + (v0.y - v2.y) * dX21;
    if (std::abs(D) < 1e-20f) return false;
    float s = ((v0.x - v2.x) * dY + (v0.y - v2.y) * dX) / D;
    float t = (dY12 * dX + dX21 * dY) / D;
    return (s >= 0) && (t >= 0) && (s + t <= 1);
}

inline float barycentric_interp(float px, float py,
                                const Vertex& v0, const Vertex& v1, const Vertex& v2)
{
    float denom = ((v1.y - v2.y)*(v0.x - v2.x) + (v2.x - v1.x)*(v0.y - v2.y));
    if (std::abs(denom) < 1e-10f) return v0.z; // degenerate triangle

    float a = ((v1.y - v2.y)*(px - v2.x) + (v2.x - v1.x)*(py - v2.y)) / denom;
    float b = ((v2.y - v0.y)*(px - v2.x) + (v0.x - v2.x)*(py - v2.y)) / denom;
    float c = 1 - a - b;
    return a*v0.z + b*v1.z + c*v2.z;
}

// --- Helper structures for volumetric mesh generation ---
struct Edge {
    int v0, v1;

    Edge(int a, int b) : v0(std::min(a, b)), v1(std::max(a, b)) {}

    bool operator<(const Edge& other) const {
        return std::tie(v0, v1) < std::tie(other.v0, other.v1);
    }

    bool operator==(const Edge& other) const {
        return v0 == other.v0 && v1 == other.v1;
    }
};

// --- Input preprocessing implementation ---

template<typename T>
struct PreprocessingResult {
    std::vector<float> processed_elevations;  // Converted to float and preprocessed
    float adjusted_error_threshold;           // Clamped to reasonable minimum
    bool has_warnings = false;                // Whether any warnings were issued
    std::vector<std::string> warnings;       // Diagnostic messages
    bool is_degenerate = false;              // Whether input is degenerate (all flat/collinear)
    bool needs_jitter = false;               // Whether small jitter was added
    float scale_factor = 1.0f;               // Applied coordinate scaling
    float z_offset = 0.0f;                   // Applied Z offset for normalization
};

// Simulation of Simplicity functions for degenerate case handling
inline float simulation_of_simplicity_perturbation(int x, int y, int width, int height, float magnitude) {
    // Create a deterministic pseudo-random perturbation based on position
    uint32_t hash = static_cast<uint32_t>(x * 73856093 ^ y * 19349663 ^ width * 83492791 ^ height * 50331653);
    hash = ((hash >> 16) ^ hash) * 0x45d9f3b;
    hash = ((hash >> 16) ^ hash) * 0x45d9f3b;
    hash = (hash >> 16) ^ hash;

    // Convert hash to [-1, 1] range and scale by magnitude
    float normalized = (static_cast<float>(hash) / static_cast<float>(0xFFFFFFFF)) * 2.0f - 1.0f;
    return normalized * magnitude;
}

template<typename T>
PreprocessingResult<T> preprocess_input_data(
    int width, int height, const T* elevations,
    float& error_threshold, bool enable_jitter = true) {

    PreprocessingResult<T> result;
    result.processed_elevations.reserve(width * height);

    // Convert to float and find min/max, handling NaN/inf robustly
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    int invalid_count = 0;
    
    // First pass: convert and identify finite values
    for (int i = 0; i < width * height; ++i) {
        float z = static_cast<float>(elevations[i]);
        if (!std::isfinite(z)) {
            invalid_count++;
            z = 0.0f; // Temporary placeholder
        }
        result.processed_elevations.push_back(z);
        if (std::isfinite(z)) {
            min_z = std::min(min_z, z);
            max_z = std::max(max_z, z);
        }
    }
    
    // Handle case where all values are invalid
    if (invalid_count == width * height) {
        result.has_warnings = true;
        result.warnings.push_back("All elevation data invalid (NaN/inf), using zero elevation");
        std::fill(result.processed_elevations.begin(), result.processed_elevations.end(), 0.0f);
        min_z = max_z = 0.0f;
    } else if (invalid_count > 0) {
        // Replace invalid values with interpolated/reasonable values
        result.has_warnings = true;
        result.warnings.push_back("Found " + std::to_string(invalid_count) + 
                                 " invalid elevation values, replaced with interpolated data");
        
        // Simple strategy: replace invalid values with mean of valid values
        float mean_elevation = (min_z + max_z) * 0.5f;
        for (int i = 0; i < width * height; ++i) {
            if (!std::isfinite(static_cast<float>(elevations[i]))) {
                result.processed_elevations[i] = mean_elevation;
            }
        }
    }

    float z_range = max_z - min_z;

    // Check for degenerate input
    if (z_range < 1e-6f) {
        result.is_degenerate = true;
        result.warnings.push_back("Input elevation data is nearly flat");

        if (enable_jitter) {
            result.needs_jitter = true;
            // Add tiny jitter to break degeneracy
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int idx = y * width + x;
                    float jitter = simulation_of_simplicity_perturbation(x, y, width, height, 1e-4f);
                    result.processed_elevations[idx] += jitter;
                }
            }
            result.warnings.push_back("Added small jitter to break degeneracy");
        }
    }

    // Adjust error threshold
    result.adjusted_error_threshold = std::max(error_threshold, z_range * 1e-6f);
    if (result.adjusted_error_threshold != error_threshold) {
        result.has_warnings = true;
        result.warnings.push_back("Error threshold adjusted to prevent numerical issues");
    }

    return result;
}

// --- Volume validation helpers ---

/**
 * Calculate the expected volume under a height field relative to a base plane
 * Each cell represents a column with area 1x1 and height = elevation - z_base
 */
template<typename T>
inline double calculate_heightfield_volume(int width, int height, const T* elevations, float z_base = 0.0f) {
    double total_volume = 0.0;
    
    // If z_base is 0, use the minimum elevation as the effective base
    if (z_base == 0.0f) {
        T min_elev = *std::min_element(elevations, elevations + width * height);
        z_base = static_cast<float>(min_elev);
    }
    
    for (int i = 0; i < width * height; ++i) {
        float height_above_base = static_cast<float>(elevations[i]) - z_base;
        if (height_above_base > 0.0f) {
            total_volume += height_above_base; // Each cell has area 1.0
        }
    }
    return total_volume;
}

/**
 * Calculate the volume of a triangle mesh using the divergence theorem
 * Volume = (1/6) * sum over all triangles of: dot(vertex, normal) * area
 */
inline double calculate_mesh_volume(const MeshResult& mesh, float z_base = 0.0f) {
    double volume = 0.0;
    
    for (const auto& tri : mesh.triangles) {
        const auto& v0 = mesh.vertices[tri.v0];
        const auto& v1 = mesh.vertices[tri.v1]; 
        const auto& v2 = mesh.vertices[tri.v2];
        
        // Translate vertices relative to base plane
        std::array<double, 3> p0 = {v0.x, v0.y, v0.z - z_base};
        std::array<double, 3> p1 = {v1.x, v1.y, v1.z - z_base};
        std::array<double, 3> p2 = {v2.x, v2.y, v2.z - z_base};
        
        // Signed volume contribution of tetrahedron from origin
        volume += (p0[0] * (p1[1] * p2[2] - p1[2] * p2[1]) +
                   p1[0] * (p2[1] * p0[2] - p2[2] * p0[1]) +
                   p2[0] * (p0[1] * p1[2] - p0[2] * p1[1])) / 6.0;
    }
    
    return std::abs(volume);
}

// --- Greedy Cuts grid-to-mesh implementation ---

/**
 * grid_to_mesh_impl
 * - Wires in the header-only greedy_cuts.hpp advancing front triangulation
 * - Uses adaptive error thresholding and mesh-quality constraints internally
 *
 * Note: point_limit is not used by Greedy Cuts (which operates on the full grid vertex set).
 *       To reduce output size, adjust error_threshold (higher -> fewer triangles) or extend
 *       greedy_cuts options to enforce budgets.
 */
template<typename T>
inline MeshResult grid_to_mesh_impl(
    int width, int height, const T* elevations,
    float error_threshold, int /*point_limit*/) {

    // Preprocess input for robustness
    auto preprocessing = preprocess_input_data(width, height, elevations, error_threshold);

    if (preprocessing.has_warnings) {
        for (const auto& warning : preprocessing.warnings) {
            std::cerr << "TerraScape warning: " << warning << std::endl;
        }
    }

    // Configure Greedy Cuts options with terrain-appropriate parameters
    ::terrascape::GreedyCutsOptions gc_opt;
    
    // Calculate elevation range for adaptive parameter setting
    float min_elev = *std::min_element(preprocessing.processed_elevations.begin(), 
                                      preprocessing.processed_elevations.end());
    float max_elev = *std::max_element(preprocessing.processed_elevations.begin(), 
                                      preprocessing.processed_elevations.end());
    float elev_range = max_elev - min_elev;
    
    // Determine appropriate error threshold for terrain data
    float adaptive_error_threshold = preprocessing.adjusted_error_threshold;
    if (elev_range > 0.1f) {
        // If user provided a small threshold relative to elevation range, respect it
        // but provide guidance on reasonable ranges
        float relative_user_threshold = adaptive_error_threshold / elev_range;
        
        if (relative_user_threshold < 0.0001f) {
            // Very small threshold - use a minimum of 0.01% of range to avoid excessive detail
            adaptive_error_threshold = elev_range * 0.0001f;
            std::cerr << "TerraScape: Error threshold very small relative to elevation range, "
                      << "using minimum " << adaptive_error_threshold << std::endl;
        } else if (relative_user_threshold > 0.1f) {
            // Large threshold - warn but allow it
            std::cerr << "TerraScape: Large error threshold relative to elevation range, "
                      << "mesh may be very coarse" << std::endl;
        }
        
        std::cerr << "TerraScape: Using error threshold " << adaptive_error_threshold 
                  << " (" << (adaptive_error_threshold/elev_range*100.0f) << "% of elevation range: " 
                  << elev_range << ")" << std::endl;
    }
    
    gc_opt.base_error_threshold = adaptive_error_threshold;
    
    // Enable volume convergence for better terrain detail
    gc_opt.use_volume_convergence = true;
    gc_opt.volume_convergence_threshold = 0.005; // 0.5% volume change for convergence
    
    // Terrain-optimized parameters that balance quality and performance
    // These parameters automatically adjust based on elevation range and desired detail level
    float relative_threshold = adaptive_error_threshold / elev_range;
    
    if (relative_threshold < 0.001f) {
        // High detail mode - more triangles, longer processing
        gc_opt.slope_weight = 0.5;              // Reduced weight for more triangles
        gc_opt.curvature_weight = 1.0;          // Reduced weight for more triangles
        gc_opt.min_angle_deg = 5.0;             // Allow sharper angles for detail
        gc_opt.max_aspect_ratio = 20.0;         // Allow thinner triangles
        gc_opt.min_area = 0.001;                // Allow smaller triangles
        gc_opt.max_refinement_passes = 8;       // More passes for detail
        gc_opt.max_initial_iterations = 15000000;
        std::cerr << "TerraScape: Using high-detail terrain parameters" << std::endl;
    } else if (relative_threshold < 0.01f) {
        // Medium detail mode - balanced quality and performance
        gc_opt.slope_weight = 0.8;
        gc_opt.curvature_weight = 2.0;
        gc_opt.min_angle_deg = 8.0;
        gc_opt.max_aspect_ratio = 15.0;
        gc_opt.min_area = 0.01;
        gc_opt.max_refinement_passes = 6;
        gc_opt.max_initial_iterations = 10000000;
        std::cerr << "TerraScape: Using medium-detail terrain parameters" << std::endl;
    } else {
        // Low detail mode - fast processing, coarser mesh
        gc_opt.slope_weight = 1.0;
        gc_opt.curvature_weight = 3.0;
        gc_opt.min_angle_deg = 10.0;
        gc_opt.max_aspect_ratio = 12.0;
        gc_opt.min_area = 0.1;
        gc_opt.max_refinement_passes = 4;
        gc_opt.max_initial_iterations = 5000000;
        std::cerr << "TerraScape: Using low-detail terrain parameters" << std::endl;
    }

    // Run Greedy Cuts triangulation
    ::terrascape::Mesh gc_mesh;
    ::terrascape::triangulateGreedyCuts(
        preprocessing.processed_elevations.data(),
        width, height,
        nullptr, // NoData mask not yet propagated from preprocessing
        gc_opt,
        gc_mesh
    );

    MeshResult result;

    // If triangulation failed to produce any triangles, fall back to a simple grid mesh
    if (gc_mesh.triangles.empty()) {
        std::cerr << "Greedy Cuts produced no triangles - falling back to simple grid mesh" << std::endl;

        // Fallback to simple grid-based triangulation (sample every cell)
        // Create vertex grid
        std::vector<std::vector<int>> vertex_indices(height, std::vector<int>(width, -1));
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int v_idx = static_cast<int>(result.vertices.size());
                result.vertices.push_back({
                    static_cast<float>(x),
                    static_cast<float>(y),
                    preprocessing.processed_elevations[y * width + x]
                });
                vertex_indices[y][x] = v_idx;
            }
        }
        for (int y = 0; y < height - 1; ++y) {
            for (int x = 0; x < width - 1; ++x) {
                int v00 = vertex_indices[y][x];
                int v01 = vertex_indices[y][x+1];
                int v10 = vertex_indices[y+1][x];
                int v11 = vertex_indices[y+1][x+1];
                result.triangles.push_back({v00, v01, v11});
                result.triangles.push_back({v00, v11, v10});
            }
        }
        return result;
    }

    // Convert Greedy Cuts mesh to TerraScape MeshResult
    // Only include vertices that are actually used by triangles
    std::unordered_set<int> used_vertices;
    for (const auto& t : gc_mesh.triangles) {
        used_vertices.insert(t[0]);
        used_vertices.insert(t[1]);
        used_vertices.insert(t[2]);
    }
    
    // Create mapping from old vertex indices to new ones
    std::vector<int> vertex_mapping(gc_mesh.vertices.size(), -1);
    result.vertices.reserve(used_vertices.size());
    
    for (int old_idx : used_vertices) {
        vertex_mapping[old_idx] = static_cast<int>(result.vertices.size());
        const auto& v = gc_mesh.vertices[old_idx];
        result.vertices.push_back({static_cast<float>(v[0]),
                                   static_cast<float>(v[1]),
                                   static_cast<float>(v[2])});
    }
    
    // Update triangle indices to use new vertex mapping
    result.triangles.reserve(gc_mesh.triangles.size());
    for (const auto& t : gc_mesh.triangles) {
        result.triangles.push_back({vertex_mapping[t[0]], 
                                   vertex_mapping[t[1]], 
                                   vertex_mapping[t[2]]});
    }

    // Volume validation sanity check
    float min_elev_for_volume = *std::min_element(preprocessing.processed_elevations.begin(), 
                                      preprocessing.processed_elevations.end());
    double heightfield_volume = calculate_heightfield_volume(width, height, preprocessing.processed_elevations.data(), min_elev_for_volume);
    double mesh_volume = calculate_mesh_volume(result, min_elev_for_volume);
    double volume_ratio = (heightfield_volume > 1e-12) ? (mesh_volume / heightfield_volume) : 0.0;
    
    std::cerr << "TerraScape Volume Validation:" << std::endl;
    std::cerr << "  Elevation range: " << min_elev_for_volume << " to " << max_elev << std::endl;
    std::cerr << "  Height field volume (above min): " << heightfield_volume << std::endl;
    std::cerr << "  Mesh volume (above min): " << mesh_volume << std::endl;
    std::cerr << "  Volume ratio (mesh/heightfield): " << volume_ratio << std::endl;
    
    if (volume_ratio < 0.5 || volume_ratio > 2.0) {
        std::cerr << "  WARNING: Volume ratio outside reasonable range (0.5-2.0)" << std::endl;
        std::cerr << "           This suggests the mesh may not properly represent the terrain" << std::endl;
    }

    return result;
}

// --- Volumetric mesh implementations ---

inline std::vector<Edge> find_boundary_edges(const std::vector<Triangle>& triangles) {
    std::map<Edge, int> edge_count;

    // Count occurrences of each edge
    for (const Triangle& tri : triangles) {
        edge_count[Edge(tri.v0, tri.v1)]++;
        edge_count[Edge(tri.v1, tri.v2)]++;
        edge_count[Edge(tri.v2, tri.v0)]++;
    }

    // Boundary edges appear exactly once
    std::vector<Edge> boundary_edges;
    for (const auto& [edge, count] : edge_count) {
        if (count == 1) {
            boundary_edges.push_back(edge);
        }
    }

    return boundary_edges;
}

inline MeshResult make_volumetric_mesh(const MeshResult& surface_mesh, float z_base) {
    MeshResult volumetric_result;
    volumetric_result.is_volumetric = true;

    // Copy surface vertices
    volumetric_result.vertices = surface_mesh.vertices;

    // Create base vertices (same x,y but z = z_base)
    std::vector<int> base_vertex_mapping(surface_mesh.vertices.size());
    for (size_t i = 0; i < surface_mesh.vertices.size(); ++i) {
        const Vertex& v = surface_mesh.vertices[i];
        base_vertex_mapping[i] = static_cast<int>(volumetric_result.vertices.size());
        volumetric_result.vertices.push_back({v.x, v.y, z_base});
    }

    // Copy surface triangles
    volumetric_result.triangles = surface_mesh.triangles;

    // Add base triangles (with flipped winding for inward-facing normals)
    for (const Triangle& tri : surface_mesh.triangles) {
        int base_v0 = base_vertex_mapping[tri.v0];
        int base_v1 = base_vertex_mapping[tri.v1];
        int base_v2 = base_vertex_mapping[tri.v2];
        volumetric_result.triangles.push_back({base_v0, base_v2, base_v1}); // Flipped winding
    }

    // Find boundary edges and create side faces
    std::vector<Edge> boundary_edges = find_boundary_edges(surface_mesh.triangles);

    for (const Edge& edge : boundary_edges) {
        int surface_v0 = edge.v0;
        int surface_v1 = edge.v1;
        int base_v0 = base_vertex_mapping[surface_v0];
        int base_v1 = base_vertex_mapping[surface_v1];

        // Create two triangles for the side face
        volumetric_result.triangles.push_back({surface_v0, surface_v1, base_v1});
        volumetric_result.triangles.push_back({surface_v0, base_v1, base_v0});
    }

    return volumetric_result;
}

inline VolumetricMeshResult make_volumetric_mesh_separated(const MeshResult& surface_mesh, float z_base) {
    VolumetricMeshResult result;

    // Separate vertices above and below z_base
    std::vector<int> positive_vertices, negative_vertices;
    std::vector<int> positive_mapping(surface_mesh.vertices.size(), -1);
    std::vector<int> negative_mapping(surface_mesh.vertices.size(), -1);

    for (size_t i = 0; i < surface_mesh.vertices.size(); ++i) {
        const Vertex& v = surface_mesh.vertices[i];

        if (v.z > z_base) {
            positive_mapping[i] = static_cast<int>(result.positive_volume.vertices.size());
            result.positive_volume.vertices.push_back(v);
            positive_vertices.push_back(static_cast<int>(i));
        } else if (v.z < z_base) {
            negative_mapping[i] = static_cast<int>(result.negative_volume.vertices.size());
            result.negative_volume.vertices.push_back(v);
            negative_vertices.push_back(static_cast<int>(i));
        }
    }

    // Process triangles
    for (const Triangle& tri : surface_mesh.triangles) {
        const Vertex& v0 = surface_mesh.vertices[tri.v0];
        const Vertex& v1 = surface_mesh.vertices[tri.v1];
        const Vertex& v2 = surface_mesh.vertices[tri.v2];

        bool above0 = v0.z > z_base, above1 = v1.z > z_base, above2 = v2.z > z_base;
        bool below0 = v0.z < z_base, below1 = v1.z < z_base, below2 = v2.z < z_base;

        // Triangle entirely above z_base
        if (above0 && above1 && above2) {
            result.positive_volume.triangles.push_back({
                positive_mapping[tri.v0], positive_mapping[tri.v1], positive_mapping[tri.v2]
            });
        }
        // Triangle entirely below z_base
        else if (below0 && below1 && below2) {
            result.negative_volume.triangles.push_back({
                negative_mapping[tri.v0], negative_mapping[tri.v2], negative_mapping[tri.v1] // Flipped winding
            });
        }
        // Triangle crosses z_base - would need clipping (simplified for now)
    }

    result.has_positive_volume = !result.positive_volume.vertices.empty();
    result.has_negative_volume = !result.negative_volume.vertices.empty();

    // Make volumetric if we have volumes
    if (result.has_positive_volume) {
        result.positive_volume = make_volumetric_mesh(result.positive_volume, z_base);
    }
    if (result.has_negative_volume) {
        result.negative_volume = make_volumetric_mesh(result.negative_volume, z_base);
    }

    return result;
}

// --- Volumetric API implementations ---

template<typename T>
inline MeshResult grid_to_mesh_volumetric(
    int width, int height, const T* elevations,
    float z_base = 0.0f,
    float error_threshold = 1.0f, int point_limit = 10000) {

    MeshResult surface_mesh = grid_to_mesh_impl(width, height, elevations, error_threshold, point_limit);
    return make_volumetric_mesh(surface_mesh, z_base);
}

template<typename T>
inline VolumetricMeshResult grid_to_mesh_volumetric_separated(
    int width, int height, const T* elevations,
    float z_base = 0.0f,
    float error_threshold = 1.0f, int point_limit = 10000) {

    MeshResult surface_mesh = grid_to_mesh_impl(width, height, elevations, error_threshold, point_limit);
    return make_volumetric_mesh_separated(surface_mesh, z_base);
}

// --- Main API Functions ---

// Core mesh generation with Greedy Cuts grid-aware triangulation
template<typename T>
inline MeshResult grid_to_mesh(
    int width, int height, const T* elevations,
    float error_threshold = 1.0f, int point_limit = 10000)
{
    // Delegate to Greedy Cuts implementation
    return grid_to_mesh_impl(width, height, elevations, error_threshold, point_limit);
}

} // namespace TerraScape
