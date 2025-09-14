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
#include <array>

// BRL-CAD tolerance integration and region-growing triangulation now included directly

#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#elif defined(__APPLE__)
#include <sys/types.h>
#include <sys/sysctl.h>
#endif

namespace TerraScape {

// ================================= BRL-CAD Tolerance Integration =================================

// Region-growing triangulation options with BRL-CAD tolerance integration
struct RegionGrowingOptions {
  // PRIMARY INTERFACE: Single mesh density parameter (0.0 = coarsest, 1.0 = finest)
  double mesh_density = 0.5;                 // Controls overall mesh resolution and detail level
  
  // BRL-CAD TOLERANCE INTEGRATION: Precision control parameters
  double abs_tolerance_mm = 0.1;             // Absolute tolerance in millimeters
  double rel_tolerance = 0.01;               // Relative tolerance (unitless, as fraction)
  double norm_tolerance_deg = 15.0;          // Normal angle tolerance in degrees
  double volume_delta_pct = 10.0;            // Max allowed mesh/cell volume % difference
  
  // DERIVED PARAMETERS: Automatically calculated from mesh_density (can be overridden)
  double region_merge_threshold = 0.5;       // Height difference threshold for region merging
  int sampling_step = 1;                     // Point sampling step size for triangulation
  double base_error_threshold = 0.2;         // Error threshold for mesh approximation
  
  // QUALITY CONSTRAINTS: Mesh quality control
  double min_angle_deg = 20.0;               // Minimum triangle angle constraint
  double max_aspect_ratio = 6.0;             // Maximum aspect ratio (longest/shortest edge)
  double min_area = 0.5;                     // Minimum triangle area in grid units
  bool enable_quality_filtering = true;      // Enable triangle quality filtering during generation
  
  // ADVANCED PARAMETERS: Typically not modified by users
  double slope_weight = 3.0;                 // Weight for slope-based error calculation
  double curvature_weight = 10.0;            // Weight for curvature-based error calculation
  int max_refinement_passes = 2;             // Number of refinement passes
  int max_initial_iterations = 5'000'000;    // Safety cap for very large datasets
  bool early_exit_error_eval = true;         // Enable early-exit in error calculation
  bool treat_all_invalid_as_blocking = true; // How to handle masked areas
  double bary_eps = 1e-6;                    // Barycentric coordinate epsilon
  double det_eps = 1e-12;                    // Determinant epsilon for numerical stability
  double area_eps = 1e-12;                   // Area epsilon for numerical stability
  bool use_volume_convergence = false;       // Enable volume-based stopping criteria
  double volume_convergence_threshold = 0.01; // Volume change threshold for convergence
  int min_volume_passes = 2;                 // Minimum passes before checking volume convergence
  bool use_precomputed_complexity = true;    // Use pre-computed terrain complexity
  double complexity_scale_factor = 1.0;      // Complexity scaling factor
};

// Internal mesh structure for region-growing algorithm
struct InternalMesh {
  std::vector<std::array<double, 3>> vertices; // x,y,z (grid coords + elevation)
  std::vector<std::array<int, 3>> triangles;   // vertex indices
  
  // Helper functions to add vertices and triangles
  void add_vertex(double x, double y, double z) {
    vertices.emplace_back(std::array<double, 3>{x, y, z});
  }
  
  void add_triangle(int v0, int v1, int v2) {
    triangles.emplace_back(std::array<int, 3>{v0, v1, v2});
  }
};

// ================================= Helper Functions =================================

static inline size_t idx_row_major(int x, int y, int width) {
  return static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
}

static inline double hypot2d(double dx, double dy) {
  return std::hypot(dx, dy);
}

static inline double tri_area2d(double x0, double y0, double x1, double y1, double x2, double y2) {
  return 0.5 * std::abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0));
}

// BRL-CAD tolerance helper functions
static inline bool meets_tolerance_threshold(double height_diff, double center_elev, double abs_tolerance, double rel_tolerance) {
  return height_diff >= abs_tolerance || height_diff >= rel_tolerance * std::abs(center_elev);
}

static inline bool within_tolerance_threshold(double height_diff, double center_elev, double neighbor_elev, double abs_tolerance, double rel_tolerance) {
  double threshold = std::min(abs_tolerance, rel_tolerance * std::max(std::abs(center_elev), std::abs(neighbor_elev)));
  return height_diff <= threshold;
}

// Calculate mesh volume using divergence theorem (simple approximation)
static inline double calculate_internal_mesh_volume_simple(const InternalMesh& mesh) {
  double volume = 0.0;
  for (const auto& tri : mesh.triangles) {
    const auto& v0 = mesh.vertices[tri[0]];
    const auto& v1 = mesh.vertices[tri[1]];
    const auto& v2 = mesh.vertices[tri[2]];
    
    // Signed volume contribution using cross product
    double vol_contrib = (v0[0] * (v1[1] * v2[2] - v1[2] * v2[1]) +
                         v1[0] * (v2[1] * v0[2] - v2[2] * v0[1]) +
                         v2[0] * (v0[1] * v1[2] - v0[2] * v1[1])) / 6.0;
    volume += vol_contrib;
  }
  return std::abs(volume);
}

// Calculate theoretical cell volume (simple height field integration)
static inline double calculate_cell_volume_simple(const float* elevations, int width, int height, const uint8_t* mask = nullptr) {
  double volume = 0.0;
  double cell_area = 1.0; // Assuming unit grid cells
  
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
      if (mask && mask[idx] == 0) continue;
      volume += elevations[idx] * cell_area;
    }
  }
  return volume;
}

// Calculate region-growing parameters from mesh_density
static inline RegionGrowingOptions calculate_region_parameters(RegionGrowingOptions opt, int width, int height) {
  // Mesh density ranges from 0.0 (coarsest) to 1.0 (finest)
  double density = std::max(0.0, std::min(1.0, opt.mesh_density));
  
  // Calculate region merge threshold (larger = coarser regions)
  // At density 0.0: large regions with high threshold
  // At density 1.0: small regions with low threshold
  opt.region_merge_threshold = 50.0 * (1.0 - density) + 0.1 * density;
  
  // Calculate sampling step (larger = sparser sampling)
  // At density 0.0: very sparse sampling (coarse mesh)
  // At density 1.0: dense sampling (fine mesh)
  int max_step = std::max(1, std::min(width, height) / 10);  // Never more than 1/10 of smallest dimension
  opt.sampling_step = static_cast<int>(max_step * (1.0 - density)) + 1;
  
  // Calculate error threshold (larger = more tolerance, coarser mesh)
  // At density 0.0: high tolerance (coarse approximation)
  // At density 1.0: low tolerance (fine approximation)
  opt.base_error_threshold = 10.0 * (1.0 - density) + 0.01 * density;
  
  return opt;
}

// ================================= Region-Growing Triangulation Algorithm =================================

// Region-growing triangulation approach with BRL-CAD tolerance integration
static inline void triangulateRegionGrowing(const float* elevations,
                                            int width, int height,
                                            const uint8_t* mask,
                                            RegionGrowingOptions opt,  // Pass by value to allow modification
                                            InternalMesh& out_mesh)
{
  const int W = width, H = height;
  const size_t total_cells = static_cast<size_t>(W) * static_cast<size_t>(H);
  
  // Handle edge cases for small grids
  if (W <= 1 || H <= 1) {
    // For very small grids, create a minimal valid mesh
    out_mesh.vertices.clear();
    out_mesh.triangles.clear();
    
    if (W == 1 && H == 1) {
      // Single point - create a degenerate triangle
      out_mesh.add_vertex(0.0, 0.0, static_cast<double>(elevations[0]));
      return;
    } else if (W == 1 || H == 1) {
      // Single row/column - create vertices but no triangles
      for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
          size_t idx = idx_row_major(x, y, W);
          if (mask && mask[idx] == 0) continue;
          out_mesh.add_vertex(static_cast<double>(x),
                              static_cast<double>(y),
                              static_cast<double>(elevations[idx]));
        }
      }
      return;
    }
  }
  
  // Calculate region-growing parameters from mesh_density
  opt = calculate_region_parameters(opt, width, height);
  
  // Step 1: Find local minima and maxima
  std::vector<std::pair<int, int>> local_extrema;
  std::vector<int> region_labels(total_cells, -1); // -1 = unassigned
  
  auto get_elevation = [&](int x, int y) -> float {
    if (x < 0 || y < 0 || x >= W || y >= H) return std::numeric_limits<float>::max();
    if (mask && mask[idx_row_major(x, y, W)] == 0) return std::numeric_limits<float>::max();
    return elevations[idx_row_major(x, y, W)];
  };
  
  // Find local extrema (min/max within 3x3 neighborhood) with tolerance checking
  for (int y = 1; y < H-1; ++y) {
    for (int x = 1; x < W-1; ++x) {
      if (mask && mask[idx_row_major(x, y, W)] == 0) continue;
      
      float center = get_elevation(x, y);
      bool is_min = true, is_max = true;
      bool has_significant_diff = false;
      
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          float neighbor = get_elevation(x+dx, y+dy);
          if (neighbor <= center) is_max = false;
          if (neighbor >= center) is_min = false;
          
          // BRL-CAD tolerance: require difference from neighbors >= tolerance
          double height_diff = std::abs(neighbor - center);
          if (meets_tolerance_threshold(height_diff, center, opt.abs_tolerance_mm, opt.rel_tolerance)) {
            has_significant_diff = true;
          }
        }
      }
      
      // Only consider extrema that meet tolerance requirements
      if ((is_min || is_max) && has_significant_diff) {
        local_extrema.push_back({x, y});
      }
    }
  }
  
  // Step 2: Grow regions from extrema
  std::queue<std::tuple<int, int, int>> growth_queue; // x, y, region_id
  
  for (size_t i = 0; i < local_extrema.size(); ++i) {
    int x = local_extrema[i].first;
    int y = local_extrema[i].second;
    int region_id = static_cast<int>(i);
    region_labels[idx_row_major(x, y, W)] = region_id;
    growth_queue.push({x, y, region_id});
  }
  
  // Grow regions using breadth-first search
  std::vector<int> region_sizes(local_extrema.size(), 0);
  
  while (!growth_queue.empty()) {
    auto [x, y, region_id] = growth_queue.front();
    growth_queue.pop();
    
    region_sizes[region_id]++;
    float center_elev = get_elevation(x, y);
    
    // Check 4-connected neighbors
    for (auto [dx, dy] : std::vector<std::pair<int,int>>{{0,1}, {1,0}, {0,-1}, {-1,0}}) {
      int nx = x + dx, ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      if (mask && mask[idx_row_major(nx, ny, W)] == 0) continue;
      
      size_t nidx = idx_row_major(nx, ny, W);
      if (region_labels[nidx] != -1) continue; // Already assigned
      
      float neighbor_elev = get_elevation(nx, ny);
      float height_diff = std::abs(neighbor_elev - center_elev);
      
      // BRL-CAD tolerance: merge condition using min of thresholds
      double tolerance_threshold = std::min({
        opt.region_merge_threshold,
        opt.abs_tolerance_mm,
        opt.rel_tolerance * std::max(std::abs(center_elev), std::abs(neighbor_elev))
      });
      
      // Assign to region if height difference is within tolerance
      if (height_diff <= tolerance_threshold) {
        region_labels[nidx] = region_id;
        growth_queue.push({nx, ny, region_id});
      }
    }
  }
  
  // Step 3: Handle collision resolution - assign unassigned cells to closest region
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      size_t idx = idx_row_major(x, y, W);
      if (region_labels[idx] != -1) continue; // Already assigned
      if (mask && mask[idx] == 0) continue; // Masked
      
      float center_elev = get_elevation(x, y);
      int best_region = -1;
      float best_score = std::numeric_limits<float>::max();
      
      // Find best region based on neighboring cells
      for (auto [dx, dy] : std::vector<std::pair<int,int>>{{0,1}, {1,0}, {0,-1}, {-1,0}}) {
        int nx = x + dx, ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        
        size_t nidx = idx_row_major(nx, ny, W);
        int neighbor_region = region_labels[nidx];
        if (neighbor_region == -1) continue;
        
        float neighbor_elev = get_elevation(nx, ny);
        float height_diff = std::abs(neighbor_elev - center_elev);
        
        // Prefer region with smallest height difference, then smallest area
        float score = height_diff * 1000.0 + static_cast<float>(region_sizes[neighbor_region]);
        if (score < best_score) {
          best_score = score;
          best_region = neighbor_region;
        }
      }
      
      if (best_region != -1) {
        region_labels[idx] = best_region;
        region_sizes[best_region]++;
      }
    }
  }
  
  // Step 4: Triangulate each region selectively using representative points
  // Instead of full grid triangulation, sample points based on region complexity
  
  out_mesh.vertices.clear();
  out_mesh.triangles.clear();
  
  std::vector<int> vertex_map(total_cells, -1); // Maps grid position to vertex index
  
  // Sample representative points from each region
  for (size_t region_id = 0; region_id < local_extrema.size(); ++region_id) {
    if (region_sizes[region_id] == 0) continue;
    
    // Add the extremum point for this region
    int ex_x = local_extrema[region_id].first;
    int ex_y = local_extrema[region_id].second;
    size_t ex_idx = idx_row_major(ex_x, ex_y, W);
    
    if (vertex_map[ex_idx] == -1) {
      vertex_map[ex_idx] = static_cast<int>(out_mesh.vertices.size());
      out_mesh.add_vertex(static_cast<double>(ex_x),
                          static_cast<double>(ex_y),
                          static_cast<double>(elevations[ex_idx]));
    }
    
    // Sample additional points from this region based on calculated sampling step
    int base_sample_step = opt.sampling_step;
    int adaptive_step = std::max(base_sample_step, static_cast<int>(std::sqrt(region_sizes[region_id]) / 20));
    int sample_step = std::min(adaptive_step, base_sample_step * 3); // Cap at 3x base step
    
    for (int y = 0; y < H; y += sample_step) {
      for (int x = 0; x < W; x += sample_step) {
        size_t idx = idx_row_major(x, y, W);
        if (region_labels[idx] != static_cast<int>(region_id)) continue;
        if (mask && mask[idx] == 0) continue;
        if (vertex_map[idx] != -1) continue; // Already added
        
        // Check if this point adds complexity
        float center_elev = elevations[idx];
        float max_diff = 0.0;
        
        for (auto [dx, dy] : std::vector<std::pair<int,int>>{{0,1}, {1,0}, {0,-1}, {-1,0}}) {
          int nx = x + dx, ny = y + dy;
          if (nx >= 0 && ny >= 0 && nx < W && ny < H) {
            float neighbor_elev = elevations[idx_row_major(nx, ny, W)];
            max_diff = std::max(max_diff, std::abs(neighbor_elev - center_elev));
          }
        }
        
        // Add point if it represents significant height variation based on BRL-CAD tolerances
        bool meets_abs_tolerance = max_diff > opt.abs_tolerance_mm;
        bool meets_rel_tolerance = max_diff > opt.rel_tolerance * std::abs(center_elev);
        bool meets_base_threshold = max_diff > opt.base_error_threshold * 0.5;
        
        if (meets_abs_tolerance || meets_rel_tolerance || meets_base_threshold) {
          vertex_map[idx] = static_cast<int>(out_mesh.vertices.size());
          out_mesh.add_vertex(static_cast<double>(x),
                              static_cast<double>(y),
                              static_cast<double>(elevations[idx]));
        }
      }
    }
  }
  
  // Add boundary points to ensure mesh covers the entire area
  int boundary_step = std::max(opt.sampling_step, std::max(1, std::max(W, H) / (50 / opt.sampling_step)));
  
  for (int x = 0; x < W; x += boundary_step) {
    for (int y : {0, H-1}) {
      size_t idx = idx_row_major(x, y, W);
      if (mask && mask[idx] == 0) continue;
      if (vertex_map[idx] == -1) {
        vertex_map[idx] = static_cast<int>(out_mesh.vertices.size());
        out_mesh.add_vertex(static_cast<double>(x),
                            static_cast<double>(y),
                            static_cast<double>(elevations[idx]));
      }
    }
  }
  
  for (int y = 0; y < H; y += boundary_step) {
    for (int x : {0, W-1}) {
      size_t idx = idx_row_major(x, y, W);
      if (mask && mask[idx] == 0) continue;
      if (vertex_map[idx] == -1) {
        vertex_map[idx] = static_cast<int>(out_mesh.vertices.size());
        out_mesh.add_vertex(static_cast<double>(x),
                            static_cast<double>(y),
                            static_cast<double>(elevations[idx]));
      }
    }
  }
  
  // Simple grid-based triangulation for the sampled vertices
  for (int y = 0; y < H-1; ++y) {
    for (int x = 0; x < W-1; ++x) {
      // Look for available vertices in 2x2 grid cells
      std::vector<int> vertex_indices;
      
      for (auto [dx, dy] : std::vector<std::pair<int,int>>{{0,0}, {1,0}, {0,1}, {1,1}}) {
        int nx = x + dx, ny = y + dy;
        if (nx < W && ny < H) {
          size_t idx = idx_row_major(nx, ny, W);
          if (vertex_map[idx] != -1) {
            vertex_indices.push_back(vertex_map[idx]);
          }
        }
      }
      
      // Create triangles if we have enough vertices
      if (vertex_indices.size() >= 3) {
        if (vertex_indices.size() == 3) {
          out_mesh.add_triangle(vertex_indices[0], vertex_indices[1], vertex_indices[2]);
        } else if (vertex_indices.size() == 4) {
          // Create two triangles for a quad
          out_mesh.add_triangle(vertex_indices[0], vertex_indices[1], vertex_indices[2]);
          out_mesh.add_triangle(vertex_indices[1], vertex_indices[3], vertex_indices[2]);
        }
      }
    }
  }
  
  // BRL-CAD Volume Sanity Check: Compare mesh volume vs theoretical cell volume
  if (opt.volume_delta_pct > 0.0 && out_mesh.triangles.size() > 0) {
    double mesh_volume = calculate_internal_mesh_volume_simple(out_mesh);
    double cell_volume = calculate_cell_volume_simple(elevations, width, height, mask);
    
    if (cell_volume > 0.0) {
      double volume_ratio = std::abs(mesh_volume - cell_volume) / cell_volume * 100.0;
      
      if (volume_ratio > opt.volume_delta_pct) {
        std::cout << "WARNING: Volume delta exceeds tolerance (" << volume_ratio 
                  << "% > " << opt.volume_delta_pct << "%)" << std::endl;
        std::cout << "  Mesh volume: " << mesh_volume 
                  << ", Cell volume: " << cell_volume << std::endl;
        std::cout << "  Consider lowering tolerance thresholds for better accuracy" << std::endl;
      }
    }
  }
}

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
 * - Uses the consolidated region-growing triangulation with BRL-CAD tolerance integration
 * - Provides adaptive error thresholding and mesh-quality constraints internally
 */
template<typename T>
inline MeshResult grid_to_mesh_impl(
    int width, int height, const T* elevations,
    float error_threshold) {

    // Preprocess input for robustness
    auto preprocessing = preprocess_input_data(width, height, elevations, error_threshold);

    if (preprocessing.has_warnings) {
        for (const auto& warning : preprocessing.warnings) {
            std::cerr << "TerraScape warning: " << warning << std::endl;
        }
    }

    // Configure Region-Growing options with terrain-appropriate parameters
    RegionGrowingOptions rg_opt;
    
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
    
    rg_opt.base_error_threshold = adaptive_error_threshold;
    
    // Enable volume convergence for better terrain detail
    rg_opt.use_volume_convergence = true;
    rg_opt.volume_convergence_threshold = 0.005; // 0.5% volume change for convergence
    
    // Terrain-optimized parameters that balance quality and performance
    // These parameters automatically adjust based on elevation range and desired detail level
    float relative_threshold = adaptive_error_threshold / elev_range;
    
    if (relative_threshold < 0.001f) {
        // High detail mode - more triangles, longer processing
        rg_opt.slope_weight = 0.5;              // Reduced weight for more triangles
        rg_opt.curvature_weight = 1.0;          // Reduced weight for more triangles
        rg_opt.min_angle_deg = 5.0;             // Allow sharper angles for detail
        rg_opt.max_aspect_ratio = 20.0;         // Allow thinner triangles
        rg_opt.min_area = 0.001;                // Allow smaller triangles
        rg_opt.max_refinement_passes = 8;       // More passes for detail
        rg_opt.max_initial_iterations = 15000000;
        std::cerr << "TerraScape: Using high-detail terrain parameters" << std::endl;
    } else if (relative_threshold < 0.01f) {
        // Medium detail mode - balanced quality and performance
        rg_opt.slope_weight = 0.8;
        rg_opt.curvature_weight = 2.0;
        rg_opt.min_angle_deg = 8.0;
        rg_opt.max_aspect_ratio = 15.0;
        rg_opt.min_area = 0.01;
        rg_opt.max_refinement_passes = 6;
        rg_opt.max_initial_iterations = 10000000;
        std::cerr << "TerraScape: Using medium-detail terrain parameters" << std::endl;
    } else {
        // Low detail mode - fast processing, coarser mesh
        rg_opt.slope_weight = 1.0;
        rg_opt.curvature_weight = 3.0;
        rg_opt.min_angle_deg = 10.0;
        rg_opt.max_aspect_ratio = 12.0;
        rg_opt.min_area = 0.1;
        rg_opt.max_refinement_passes = 4;
        rg_opt.max_initial_iterations = 5000000;
        std::cerr << "TerraScape: Using low-detail terrain parameters" << std::endl;
    }

    // Volume validation sanity check - already performed by region-growing algorithm
    MeshResult result = region_growing_triangulation_advanced(
        preprocessing.processed_elevations.data(),
        width, height,
        nullptr, // NoData mask not yet propagated from preprocessing
        rg_opt
    );

    // Additional volume validation for TerraScape interface consistency
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
        volumetric_result.vertices.push_back(Vertex{v.x, v.y, z_base});
    }

    // Copy surface triangles
    volumetric_result.triangles = surface_mesh.triangles;

    // Add base triangles (with flipped winding for inward-facing normals)
    for (const Triangle& tri : surface_mesh.triangles) {
        int base_v0 = base_vertex_mapping[tri.v0];
        int base_v1 = base_vertex_mapping[tri.v1];
        int base_v2 = base_vertex_mapping[tri.v2];
        volumetric_result.triangles.push_back(Triangle{base_v0, base_v2, base_v1}); // Flipped winding
    }

    // Find boundary edges and create side faces
    std::vector<Edge> boundary_edges = find_boundary_edges(surface_mesh.triangles);

    for (const Edge& edge : boundary_edges) {
        int surface_v0 = edge.v0;
        int surface_v1 = edge.v1;
        int base_v0 = base_vertex_mapping[surface_v0];
        int base_v1 = base_vertex_mapping[surface_v1];

        // Create two triangles for the side face
        volumetric_result.triangles.push_back(Triangle{surface_v0, surface_v1, base_v1});
        volumetric_result.triangles.push_back(Triangle{surface_v0, base_v1, base_v0});
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
            result.positive_volume.triangles.push_back(Triangle{
                positive_mapping[tri.v0], positive_mapping[tri.v1], positive_mapping[tri.v2]
            });
        }
        // Triangle entirely below z_base
        else if (below0 && below1 && below2) {
            result.negative_volume.triangles.push_back(Triangle{
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
    float error_threshold = 1.0f) {

    MeshResult surface_mesh = grid_to_mesh_impl(width, height, elevations, error_threshold);
    return make_volumetric_mesh(surface_mesh, z_base);
}

template<typename T>
inline VolumetricMeshResult grid_to_mesh_volumetric_separated(
    int width, int height, const T* elevations,
    float z_base = 0.0f,
    float error_threshold = 1.0f) {

    MeshResult surface_mesh = grid_to_mesh_impl(width, height, elevations, error_threshold);
    return make_volumetric_mesh_separated(surface_mesh, z_base);
}

// ================================= Region-Growing API Functions =================================

// Function definitions moved to end of file after all struct definitions

// --- Main API Functions ---

// Core mesh generation with Region-Growing grid-aware triangulation
template<typename T>
inline MeshResult grid_to_mesh(
    int width, int height, const T* elevations,
    float error_threshold = 1.0f)
{
    // Delegate to Region-Growing implementation
    return grid_to_mesh_impl(width, height, elevations, error_threshold);
}

// ================================= Region-Growing API Function Implementations =================================

// Primary simplified interface using mesh density parameter
inline MeshResult region_growing_triangulation(const float* elevations,
                                              int width, int height,
                                              double mesh_density = 0.5,
                                              const uint8_t* mask = nullptr)
{
  RegionGrowingOptions opt;
  opt.mesh_density = mesh_density;
  
  InternalMesh internal_mesh;
  triangulateRegionGrowing(elevations, width, height, mask, opt, internal_mesh);
  
  // Convert InternalMesh to MeshResult
  MeshResult result;
  result.vertices.reserve(internal_mesh.vertices.size());
  result.triangles.reserve(internal_mesh.triangles.size());
  
  for (const auto& v : internal_mesh.vertices) {
    result.vertices.push_back(Vertex{static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])});
  }
  
  for (const auto& t : internal_mesh.triangles) {
    result.triangles.push_back(Triangle{t[0], t[1], t[2]});
  }
  
  return result;
}

// Full interface with all options (for advanced usage and BRL-CAD tolerance control)
inline MeshResult region_growing_triangulation_advanced(const float* elevations,
                                                        int width, int height,
                                                        const uint8_t* mask, // optional, nullptr if none
                                                        const RegionGrowingOptions& opt)
{
  InternalMesh internal_mesh;
  triangulateRegionGrowing(elevations, width, height, mask, opt, internal_mesh);
  
  // Convert InternalMesh to MeshResult
  MeshResult result;
  result.vertices.reserve(internal_mesh.vertices.size());
  result.triangles.reserve(internal_mesh.triangles.size());
  
  for (const auto& v : internal_mesh.vertices) {
    result.vertices.push_back(Vertex{static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])});
  }
  
  for (const auto& t : internal_mesh.triangles) {
    result.triangles.push_back(Triangle{t[0], t[1], t[2]});
  }
  
  return result;
}

} // namespace TerraScape
