/*
  Greedy Cuts Terrain Triangulation (Header-only)

  Features:
  - Advancing front with priority queue (stable, versioned nodes)
  - Adaptive thresholding (slope + 8-neighbor curvature)
  - Multi-pass refinement with interior-only point insertion
  - Mesh quality constraints (min angle, max aspect, min area)
  - Row-major indexing (y * width + x), double math for geometry
  - Optional NoData mask support (0 = masked/invalid, nonzero = valid)
  - Degeneracy checks, robust remainder “ear” fill (no holes)
  - Early-out error evaluation to cut compute cost
  - Header-only to avoid immediate CMake changes; include and call directly

  Usage:
    #include "greedy_cuts.hpp"
    terrascape::GreedyCutsOptions opts;
    opts.base_error_threshold = 0.2;
    // ... set other options as desired ...
    terrascape::Mesh mesh;
    terrascape::triangulateGreedyCuts(elevations.data(), width, height, nullptr, opts, mesh);

  License: same as repository (adjust as needed).
*/

#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace terrascape {

// --------------------------------- Types ---------------------------------

struct GreedyCutsOptions {
  // PRIMARY INTERFACE: Single mesh density parameter (0.0 = coarsest, 1.0 = finest)
  double mesh_density = 0.5;                 // Controls overall mesh resolution and detail level
  
  // ALGORITHM SELECTION: Region-growing is now the default approach
  bool use_region_growing = true;            // Use fast region-growing approach (default)
  bool use_advancing_front = false;          // Use slower advancing front method (legacy)
  
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
  int max_pq_size = 50000;                   // Priority queue size limit
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

struct Mesh {
  std::vector<std::array<double, 3>> vertices; // x,y,z (grid coords + elevation)
  std::vector<std::array<int, 3>> triangles;   // vertex indices
};

// --------------------------------- Helpers ---------------------------------

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
static inline double calculate_mesh_volume_simple(const Mesh& mesh) {
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

static inline bool barycentric_uvwt(const std::array<double,3>& a,
                                    const std::array<double,3>& b,
                                    const std::array<double,3>& c,
                                    double px, double py,
                                    double det_eps,
                                    double& u, double& v, double& w)
{
  double det = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1]);
  if (std::abs(det) < det_eps) return false;
  u = ((b[1] - c[1]) * (px - c[0]) + (c[0] - b[0]) * (py - c[1])) / det;
  v = ((c[1] - a[1]) * (px - c[0]) + (a[0] - c[0]) * (py - c[1])) / det;
  w = 1.0 - u - v;
  return (u >= 0.0 && v >= 0.0 && w >= 0.0);
}

static inline double triangle_min_angle_deg_2d(const std::array<double,3>& v0,
                                               const std::array<double,3>& v1,
                                               const std::array<double,3>& v2)
{
  auto angle_at = [](const std::array<double,3>& a,
                     const std::array<double,3>& b,
                     const std::array<double,3>& c) {
    double ux = b[0] - a[0], uy = b[1] - a[1];
    double vx = c[0] - a[0], vy = c[1] - a[1];
    double dot = ux * vx + uy * vy;
    double nu = std::hypot(ux, uy);
    double nv = std::hypot(vx, vy);
    double denom = (nu * nv > 0.0) ? (nu * nv) : 1.0;
    double ct = std::clamp(dot / denom, -1.0, 1.0);
    return std::acos(ct) * 180.0 / 3.14159265358979323846;
  };
  double a0 = angle_at(v0, v1, v2);
  double a1 = angle_at(v1, v2, v0);
  double a2 = angle_at(v2, v0, v1);
  return std::min({a0, a1, a2});
}

static inline double triangle_aspect_ratio_2d(const std::array<double,3>& v0,
                                              const std::array<double,3>& v1,
                                              const std::array<double,3>& v2)
{
  double a = std::hypot(v0[0] - v1[0], v0[1] - v1[1]);
  double b = std::hypot(v1[0] - v2[0], v1[1] - v2[1]);
  double c = std::hypot(v2[0] - v0[0], v2[1] - v0[1]);
  double longest = std::max({a, b, c});
  double shortest = std::max(std::min({a, b, c}), 1e-12);
  return longest / shortest;
}

static inline bool triangle_is_quality_2d(const std::array<double,3>& v0,
                                          const std::array<double,3>& v1,
                                          const std::array<double,3>& v2,
                                          double min_angle_deg,
                                          double max_aspect_ratio,
                                          double min_area)
{
  double area = tri_area2d(v0[0], v0[1], v1[0], v1[1], v2[0], v2[1]);
  if (area < min_area) return false;
  double min_angle = triangle_min_angle_deg_2d(v0, v1, v2);
  if (min_angle < min_angle_deg) return false;
  double aspect = triangle_aspect_ratio_2d(v0, v1, v2);
  if (aspect > max_aspect_ratio) return false;
  return true;
}

static inline double tri_local_slope(const std::array<double,3>& v0,
                                     const std::array<double,3>& v1,
                                     const std::array<double,3>& v2)
{
  auto edge_slope = [](const std::array<double,3>& a,
                       const std::array<double,3>& b) {
    double horiz = std::hypot(a[0]-b[0], a[1]-b[1]);
    if (horiz < 1e-12) return 0.0;
    return std::abs(a[2] - b[2]) / horiz;
  };
  double s0 = edge_slope(v0, v1);
  double s1 = edge_slope(v1, v2);
  double s2 = edge_slope(v2, v0);
  return std::max({s0, s1, s2});
}

static inline double tri_local_curvature_8n(int width, int height,
                                            const float* elev,
                                            int cx, int cy)
{
  // 8-neighbor discrete Laplacian magnitude (not normalized by spacing)
  auto E = [&](int x, int y) -> double {
    x = std::clamp(x, 0, width-1);
    y = std::clamp(y, 0, height-1);
    return static_cast<double>(elev[idx_row_major(x,y,width)]);
  };
  double c  = E(cx, cy);
  double n  = E(cx, cy-1);
  double s  = E(cx, cy+1);
  double e  = E(cx+1, cy);
  double w  = E(cx-1, cy);
  double ne = E(cx+1, cy-1);
  double nw = E(cx-1, cy-1);
  double se = E(cx+1, cy+1);
  double sw = E(cx-1, cy+1);
  double lap4 = (n + s + e + w - 4.0*c);
  double lap8 = (ne + nw + se + sw + n + s + e + w - 8.0*c);
  // blend or choose; we use |lap8| for stronger feature response
  return std::abs(lap8);
}

static inline double adaptive_threshold(double base,
                                        double slope,
                                        double slope_w,
                                        double curvature,
                                        double curvature_w)
{
  double denom = 1.0 + slope_w * slope + curvature_w * curvature;
  return std::clamp(base / denom, 1e-9, 1e9);
}

// Pre-compute terrain complexity map for all grid cells (optimized for performance)
static inline std::vector<double> precompute_terrain_complexity(const float* elevations,
                                                                int width, int height,
                                                                const uint8_t* mask = nullptr)
{
  std::vector<double> complexity_map(static_cast<size_t>(width) * static_cast<size_t>(height), 0.0);
  
  // Optimized single-pass computation focusing on essential features
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (mask && mask[idx_row_major(x, y, width)] == 0) {
        complexity_map[idx_row_major(x, y, width)] = 0.0;
        continue;
      }
      
      // Simple gradient calculation using immediate neighbors only
      auto E = [&](int ix, int iy) -> double {
        ix = std::clamp(ix, 0, width-1);
        iy = std::clamp(iy, 0, height-1);
        return static_cast<double>(elevations[idx_row_major(ix, iy, width)]);
      };
      
      double center = E(x, y);
      
      // Simplified gradient using only immediate cross neighbors (faster than Sobel)
      double gx = E(x+1, y) - E(x-1, y);
      double gy = E(x, y+1) - E(x, y-1);
      double gradient = std::sqrt(gx*gx + gy*gy) * 0.25; // normalized and scaled down
      
      // Simple curvature using immediate cross neighbors only
      double curvature = std::abs(E(x+1, y) + E(x-1, y) + E(x, y+1) + E(x, y-1) - 4.0*center) * 0.1;
      
      // Combine into complexity metric (keep it simple and fast)
      complexity_map[idx_row_major(x, y, width)] = gradient + curvature;
    }
  }
  
  return complexity_map;
}

// Calculate region-growing parameters from mesh_density
static inline GreedyCutsOptions calculate_region_parameters(GreedyCutsOptions opt, int width, int height) {
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

// Region-growing triangulation approach
static inline void triangulateRegionGrowing(const float* elevations,
                                            int width, int height,
                                            const uint8_t* mask,
                                            GreedyCutsOptions opt,  // Pass by value to allow modification
                                            Mesh& out_mesh)
{
  const int W = width, H = height;
  const size_t total_cells = static_cast<size_t>(W) * static_cast<size_t>(H);
  
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
      out_mesh.vertices.push_back({static_cast<double>(ex_x),
                                   static_cast<double>(ex_y),
                                   static_cast<double>(elevations[ex_idx])});
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
          out_mesh.vertices.push_back({static_cast<double>(x),
                                       static_cast<double>(y),
                                       static_cast<double>(elevations[idx])});
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
        out_mesh.vertices.push_back({static_cast<double>(x),
                                     static_cast<double>(y),
                                     static_cast<double>(elevations[idx])});
      }
    }
  }
  
  for (int y = 0; y < H; y += boundary_step) {
    for (int x : {0, W-1}) {
      size_t idx = idx_row_major(x, y, W);
      if (mask && mask[idx] == 0) continue;
      if (vertex_map[idx] == -1) {
        vertex_map[idx] = static_cast<int>(out_mesh.vertices.size());
        out_mesh.vertices.push_back({static_cast<double>(x),
                                     static_cast<double>(y),
                                     static_cast<double>(elevations[idx])});
      }
    }
  }
  
  // Improved triangulation using Delaunay-like approach for sparse vertices
  // First, try simple grid-based triangulation, then fall back to more general approach
  
  int initial_triangle_count = 0;
  
  // Simple grid-based triangulation for dense meshes
  for (int y = 0; y < H-1; ++y) {
    for (int x = 0; x < W-1; ++x) {
      // Look for available vertices in 2x2 grid cells
      std::vector<std::pair<int, int>> available_positions;
      std::vector<int> vertex_indices;
      
      for (auto [dx, dy] : std::vector<std::pair<int,int>>{{0,0}, {1,0}, {0,1}, {1,1}}) {
        int nx = x + dx, ny = y + dy;
        if (nx < W && ny < H) {
          size_t idx = idx_row_major(nx, ny, W);
          if (vertex_map[idx] != -1) {
            available_positions.push_back({nx, ny});
            vertex_indices.push_back(vertex_map[idx]);
          }
        }
      }
      
      // Create triangles if we have enough vertices
      if (vertex_indices.size() >= 3) {
        if (vertex_indices.size() == 3) {
          // Check triangle quality before adding (if quality filtering is enabled)
          if (!opt.enable_quality_filtering) {
            out_mesh.triangles.push_back({vertex_indices[0], vertex_indices[1], vertex_indices[2]});
            initial_triangle_count++;
          } else {
            const auto& v0 = out_mesh.vertices[vertex_indices[0]];
            const auto& v1 = out_mesh.vertices[vertex_indices[1]];
            const auto& v2 = out_mesh.vertices[vertex_indices[2]];
            if (triangle_is_quality_2d(v0, v1, v2, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area)) {
              out_mesh.triangles.push_back({vertex_indices[0], vertex_indices[1], vertex_indices[2]});
              initial_triangle_count++;
            }
          }
        } else if (vertex_indices.size() == 4) {
          // Create two triangles for a quad, but choose the better diagonal (if quality filtering enabled)
          if (!opt.enable_quality_filtering) {
            // Simple diagonal split without quality checks
            out_mesh.triangles.push_back({vertex_indices[0], vertex_indices[1], vertex_indices[2]});
            out_mesh.triangles.push_back({vertex_indices[1], vertex_indices[3], vertex_indices[2]});
            initial_triangle_count += 2;
          } else {
            const auto& v0 = out_mesh.vertices[vertex_indices[0]];
            const auto& v1 = out_mesh.vertices[vertex_indices[1]];
            const auto& v2 = out_mesh.vertices[vertex_indices[2]];
            const auto& v3 = out_mesh.vertices[vertex_indices[3]];
            
            // Try diagonal v0-v2 (triangles v0,v1,v2 and v0,v2,v3)
            bool diagonal1_quality = triangle_is_quality_2d(v0, v1, v2, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area) &&
                                     triangle_is_quality_2d(v0, v2, v3, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area);
            
            // Try diagonal v1-v3 (triangles v0,v1,v3 and v1,v2,v3)  
            bool diagonal2_quality = triangle_is_quality_2d(v0, v1, v3, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area) &&
                                     triangle_is_quality_2d(v1, v2, v3, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area);
            
            if (diagonal1_quality && !diagonal2_quality) {
              // Use diagonal v0-v2
              out_mesh.triangles.push_back({vertex_indices[0], vertex_indices[1], vertex_indices[2]});
              out_mesh.triangles.push_back({vertex_indices[0], vertex_indices[2], vertex_indices[3]});
              initial_triangle_count += 2;
            } else if (diagonal2_quality && !diagonal1_quality) {
              // Use diagonal v1-v3
              out_mesh.triangles.push_back({vertex_indices[0], vertex_indices[1], vertex_indices[3]});
              out_mesh.triangles.push_back({vertex_indices[1], vertex_indices[2], vertex_indices[3]});
              initial_triangle_count += 2;
            } else if (diagonal1_quality && diagonal2_quality) {
              // Both diagonals produce quality triangles, choose the one with better minimum angles
              double min_angle1 = std::min(triangle_min_angle_deg_2d(v0, v1, v2), triangle_min_angle_deg_2d(v0, v2, v3));
              double min_angle2 = std::min(triangle_min_angle_deg_2d(v0, v1, v3), triangle_min_angle_deg_2d(v1, v2, v3));
              
              if (min_angle1 >= min_angle2) {
                // Use diagonal v0-v2
                out_mesh.triangles.push_back({vertex_indices[0], vertex_indices[1], vertex_indices[2]});
                out_mesh.triangles.push_back({vertex_indices[0], vertex_indices[2], vertex_indices[3]});
              } else {
                // Use diagonal v1-v3
                out_mesh.triangles.push_back({vertex_indices[0], vertex_indices[1], vertex_indices[3]});
                out_mesh.triangles.push_back({vertex_indices[1], vertex_indices[2], vertex_indices[3]});
              }
              initial_triangle_count += 2;
            }
            // If neither diagonal produces quality triangles, skip this quad
          }
        }
      }
    }
  }
  
  // If we didn't get many triangles from grid approach, use a more general triangulation
  if (initial_triangle_count < out_mesh.vertices.size() / 10) {
    out_mesh.triangles.clear();
    
    // Create a simpler triangulation for better performance
    if (out_mesh.vertices.size() >= 3) {
      auto distance_squared = [&](int i, int j) -> double {
        double dx = out_mesh.vertices[i][0] - out_mesh.vertices[j][0];
        double dy = out_mesh.vertices[i][1] - out_mesh.vertices[j][1];
        return dx*dx + dy*dy;
      };
      
      // For performance, limit the triangulation complexity
      std::set<std::tuple<int,int,int>> added_triangles;
      int max_triangles = static_cast<int>(out_mesh.vertices.size()) * 8; // Reasonable limit
      
      for (size_t i = 0; i < out_mesh.vertices.size() && out_mesh.triangles.size() < max_triangles; ++i) {
        std::vector<std::pair<double, int>> nearby_vertices;
        
        // Only consider vertices within reasonable distance
        double max_search_dist = (W + H) * 0.2;
        max_search_dist *= max_search_dist; // Square it
        
        for (size_t j = i + 1; j < out_mesh.vertices.size(); ++j) {
          double dist_sq = distance_squared(static_cast<int>(i), static_cast<int>(j));
          if (dist_sq < max_search_dist) {
            nearby_vertices.push_back({dist_sq, static_cast<int>(j)});
          }
        }
        
        // Sort by distance and take closest vertices (limit to improve performance)
        std::sort(nearby_vertices.begin(), nearby_vertices.end());
        if (nearby_vertices.size() > 8) {
          nearby_vertices.resize(8); // Limit for performance
        }
        
        // Create triangles with nearby vertices, but only if they meet quality constraints
        for (size_t j = 0; j < nearby_vertices.size(); ++j) {
          for (size_t k = j+1; k < nearby_vertices.size(); ++k) {
            int v0 = static_cast<int>(i);
            int v1 = nearby_vertices[j].second;
            int v2 = nearby_vertices[k].second;
            
            // Check triangle quality before adding
            if (!opt.enable_quality_filtering) {
              // Add triangle without quality checks
              std::array<int, 3> triangle = {v0, v1, v2};
              std::sort(triangle.begin(), triangle.end());
              
              auto tri_tuple = std::make_tuple(triangle[0], triangle[1], triangle[2]);
              
              // Add triangle if not already added
              if (added_triangles.find(tri_tuple) == added_triangles.end()) {
                out_mesh.triangles.push_back({triangle[0], triangle[1], triangle[2]});
                added_triangles.insert(tri_tuple);
                
                // Break early if we have enough triangles
                if (out_mesh.triangles.size() >= max_triangles) break;
              }
            } else {
              // Check triangle quality before adding
              const auto& vertex0 = out_mesh.vertices[v0];
              const auto& vertex1 = out_mesh.vertices[v1];
              const auto& vertex2 = out_mesh.vertices[v2];
              
              if (!triangle_is_quality_2d(vertex0, vertex1, vertex2, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area)) {
                continue; // Skip poor quality triangles
              }
              
              // Sort vertices to create canonical triangle representation
              std::array<int, 3> triangle = {v0, v1, v2};
              std::sort(triangle.begin(), triangle.end());
              
              auto tri_tuple = std::make_tuple(triangle[0], triangle[1], triangle[2]);
              
              // Add triangle if not already added
              if (added_triangles.find(tri_tuple) == added_triangles.end()) {
                out_mesh.triangles.push_back({triangle[0], triangle[1], triangle[2]});
                added_triangles.insert(tri_tuple);
                
                // Break early if we have enough triangles
                if (out_mesh.triangles.size() >= max_triangles) break;
              }
            }
          }
          if (out_mesh.triangles.size() >= max_triangles) break;
        }
      }
    }
  }
  
  // Optional post-processing: triangle quality improvement pass
  // This helps resolve cases where quality filtering may have left gaps
  if (opt.enable_quality_filtering && out_mesh.triangles.size() < out_mesh.vertices.size() / 5) {
    // If we don't have enough triangles, relax quality constraints and fill gaps
    std::cout << "Applying relaxed quality constraints to ensure mesh coverage..." << std::endl;
    
    GreedyCutsOptions relaxed_opt = opt;
    relaxed_opt.min_angle_deg = std::max(5.0, opt.min_angle_deg * 0.5);  // Halve angle requirement
    relaxed_opt.max_aspect_ratio = opt.max_aspect_ratio * 2.0;            // Double aspect ratio allowance
    relaxed_opt.min_area = opt.min_area * 0.1;                            // Reduce minimum area
    
    std::set<std::tuple<int,int,int>> existing_triangles;
    for (const auto& tri : out_mesh.triangles) {
      std::array<int, 3> sorted_tri = {tri[0], tri[1], tri[2]};
      std::sort(sorted_tri.begin(), sorted_tri.end());
      existing_triangles.insert(std::make_tuple(sorted_tri[0], sorted_tri[1], sorted_tri[2]));
    }
    
    // Try to add more triangles with relaxed constraints
    for (size_t i = 0; i < out_mesh.vertices.size() && out_mesh.triangles.size() < out_mesh.vertices.size(); ++i) {
      std::vector<std::pair<double, int>> nearby_vertices;
      
      double max_search_dist = (W + H) * 0.3; // Slightly larger search area
      max_search_dist *= max_search_dist;
      
      for (size_t j = i + 1; j < out_mesh.vertices.size(); ++j) {
        double dx = out_mesh.vertices[i][0] - out_mesh.vertices[j][0];
        double dy = out_mesh.vertices[i][1] - out_mesh.vertices[j][1];
        double dist_sq = dx*dx + dy*dy;
        if (dist_sq < max_search_dist) {
          nearby_vertices.push_back({dist_sq, static_cast<int>(j)});
        }
      }
      
      std::sort(nearby_vertices.begin(), nearby_vertices.end());
      if (nearby_vertices.size() > 6) {
        nearby_vertices.resize(6);
      }
      
      for (size_t j = 0; j < nearby_vertices.size(); ++j) {
        for (size_t k = j+1; k < nearby_vertices.size(); ++k) {
          int v0 = static_cast<int>(i);
          int v1 = nearby_vertices[j].second;
          int v2 = nearby_vertices[k].second;
          
          std::array<int, 3> triangle = {v0, v1, v2};
          std::sort(triangle.begin(), triangle.end());
          auto tri_tuple = std::make_tuple(triangle[0], triangle[1], triangle[2]);
          
          if (existing_triangles.find(tri_tuple) != existing_triangles.end()) {
            continue; // Already exists
          }
          
          const auto& vertex0 = out_mesh.vertices[v0];
          const auto& vertex1 = out_mesh.vertices[v1];
          const auto& vertex2 = out_mesh.vertices[v2];
          
          if (triangle_is_quality_2d(vertex0, vertex1, vertex2, relaxed_opt.min_angle_deg, 
                                   relaxed_opt.max_aspect_ratio, relaxed_opt.min_area)) {
            out_mesh.triangles.push_back({triangle[0], triangle[1], triangle[2]});
            existing_triangles.insert(tri_tuple);
            
            if (out_mesh.triangles.size() >= out_mesh.vertices.size()) break;
          }
        }
        if (out_mesh.triangles.size() >= out_mesh.vertices.size()) break;
      }
    }
  }
  
  // BRL-CAD Volume Sanity Check: Compare mesh volume vs theoretical cell volume
  if (opt.volume_delta_pct > 0.0 && out_mesh.triangles.size() > 0) {
    double mesh_volume = calculate_mesh_volume_simple(out_mesh);
    double cell_volume = calculate_cell_volume_simple(elevations, width, height, mask);
    
    if (cell_volume > 0.0) {
      double volume_ratio = std::abs(mesh_volume - cell_volume) / cell_volume * 100.0;
      
      if (volume_ratio > opt.volume_delta_pct) {
        std::cout << "WARNING: Volume delta exceeds tolerance (" << volume_ratio 
                  << "% > " << opt.volume_delta_pct << "%)" << std::endl;
        std::cout << "  Mesh volume: " << mesh_volume 
                  << ", Cell volume: " << cell_volume << std::endl;
        std::cout << "  Consider lowering tolerance thresholds for better accuracy" << std::endl;
        
        // Optionally trigger forced refinement by reducing thresholds
        // For now, we just warn - forced refinement could be implemented here
      }
    }
  }
}

// Error evaluation with optional early-exit and mask
struct TriError {
  double error = -1.0;
  int max_x = -1;
  int max_y = -1;
  bool any_valid = false;
};

// Scans triangle's bounding box with adaptive sampling; early-exits if err > stop_at (if stop_at > 0)
static inline TriError triangle_error_and_argmax(const std::array<double,3>& v0,
                                                 const std::array<double,3>& v1,
                                                 const std::array<double,3>& v2,
                                                 const float* elev,
                                                 const uint8_t* mask,
                                                 int width, int height,
                                                 const GreedyCutsOptions& opt,
                                                 double stop_at = -1.0)
{
  double minx = std::max(0.0, std::floor(std::min({v0[0], v1[0], v2[0]})));
  double maxx = std::min(static_cast<double>(width-1), std::ceil(std::max({v0[0], v1[0], v2[0]})));
  double miny = std::max(0.0, std::floor(std::min({v0[1], v1[1], v2[1]})));
  double maxy = std::min(static_cast<double>(height-1), std::ceil(std::max({v0[1], v1[1], v2[1]})));

  TriError out;
  if (maxx < minx || maxy < miny) return out;

  // quick coarse checks: corners + center
  auto eval_point = [&](int sx, int sy) {
    if (sx < 0 || sy < 0 || sx >= width || sy >= height) return;
    if (mask && mask[idx_row_major(sx,sy,width)] == 0) return;
    double u, v, w;
    if (!barycentric_uvwt(v0, v1, v2, static_cast<double>(sx), static_cast<double>(sy), opt.det_eps, u, v, w))
      return;
    if (u < 0.0 || v < 0.0 || w < 0.0) return;
    out.any_valid = true;
    double tri_z = u * v0[2] + v * v1[2] + w * v2[2];
    double grid_z = static_cast<double>(elev[idx_row_major(sx, sy, width)]);
    double err = std::abs(tri_z - grid_z);
    if (err > out.error) { out.error = err; out.max_x = sx; out.max_y = sy; }
  };

  int ix0 = static_cast<int>(minx), ix1 = static_cast<int>(maxx);
  int iy0 = static_cast<int>(miny), iy1 = static_cast<int>(maxy);
  eval_point(ix0, iy0);
  eval_point(ix1, iy0);
  eval_point(ix0, iy1);
  eval_point(ix1, iy1);
  eval_point((ix0+ix1)/2, (iy0+iy1)/2);

  // If early coarse exceeded stop_at, early return
  if (opt.early_exit_error_eval && stop_at > 0.0 && out.error > stop_at) return out;

  // Adaptive sampling: use coarser sampling for large triangles to improve performance
  int bbox_width = ix1 - ix0 + 1;
  int bbox_height = iy1 - iy0 + 1;
  int total_pixels = bbox_width * bbox_height;
  
  // For large triangles, use coarser sampling
  int sample_step = 1;
  if (total_pixels > 10000) {
    sample_step = 4; // Sample every 4th pixel for very large triangles
  } else if (total_pixels > 2500) {
    sample_step = 2; // Sample every 2nd pixel for large triangles
  }

  // Full scan with adaptive step
  for (int sy = iy0; sy <= iy1; sy += sample_step) {
    for (int sx = ix0; sx <= ix1; sx += sample_step) {
      eval_point(sx, sy);
      if (opt.early_exit_error_eval && stop_at > 0.0 && out.error > stop_at) return out;
    }
  }
  return out;
}

// --------------------------------- Advancing Front ---------------------------------

struct FrontNode {
  int vid = -1;     // vertex id into vertex array
  int prev = -1;    // index into nodes vector
  int next = -1;    // index into nodes vector
  int version = 0;  // increment when topology around this node changes
  bool alive = false;
};

// PQ item stores the middle node index and its version to avoid stale use
struct PQItem {
  int mid_node = -1;
  int version = 0;
  double priority_error = 0.0;
  // max-heap on error
  bool operator<(const PQItem& other) const {
    return priority_error < other.priority_error;
  }
};

// Build rectangular front nodes in CCW order: top row (left->right), right col, bottom row (right->left), left col
static inline void init_rect_front(int width, int height,
                                   std::vector<FrontNode>& nodes,
                                   std::deque<int>& front_ring)
{
  const int n = width * height; (void)n; // not used directly here
  auto push_node = [&](int vid) {
    int idx = static_cast<int>(nodes.size());
    nodes.push_back(FrontNode{vid, idx-1, -1, 0, true});
    if (!front_ring.empty()) {
      int prev_idx = front_ring.back();
      nodes[prev_idx].next = idx;
      nodes[idx].prev = prev_idx;
    }
    front_ring.push_back(idx);
  };

  // Top
  for (int x = 0; x < width; ++x) push_node(static_cast<int>(idx_row_major(x, 0, width)));
  // Right
  for (int y = 1; y < height; ++y) push_node(static_cast<int>(idx_row_major(width-1, y, width)));
  // Bottom
  for (int x = width - 2; x >= 0; --x) push_node(static_cast<int>(idx_row_major(x, height-1, width)));
  // Left
  for (int y = height - 2; y > 0; --y) push_node(static_cast<int>(idx_row_major(0, y, width)));

  // Close ring
  if (!front_ring.empty()) {
    int first = front_ring.front();
    int last = front_ring.back();
    nodes[first].prev = last;
    nodes[last].next = first;
  }
}

static inline void erase_node(std::vector<FrontNode>& nodes, int node_idx) {
  FrontNode& m = nodes[node_idx];
  if (!m.alive) return;
  FrontNode& p = nodes[m.prev];
  FrontNode& n = nodes[m.next];
  // relink
  p.next = m.next;
  n.prev = m.prev;
  // bump versions of neighbors so queued items referencing them become stale
  p.version++;
  n.version++;
  m.alive = false;
}

// --------------------------------- Main API ---------------------------------

// Primary simplified interface using mesh density parameter
inline void triangulateRegionGrowing(const float* elevations,
                                     int width, int height,
                                     Mesh& out_mesh,
                                     double mesh_density = 0.5,
                                     const uint8_t* mask = nullptr)
{
  GreedyCutsOptions opt;
  opt.mesh_density = mesh_density;
  opt.use_region_growing = true;
  triangulateRegionGrowing(elevations, width, height, mask, opt, out_mesh);
}

// Full interface with all options (for backward compatibility and advanced usage)
inline void triangulateGreedyCuts(const float* elevations,
                                  int width, int height,
                                  const uint8_t* mask, // optional, nullptr if none
                                  const GreedyCutsOptions& opt,
                                  Mesh& out_mesh)
{
  assert(width > 1 && height > 1);
  
  // Default to region-growing approach (much faster and simpler)
  if (opt.use_region_growing) {
    return triangulateRegionGrowing(elevations, width, height, mask, opt, out_mesh);
  }
  
  // Legacy advancing front approach (slower but maintained for compatibility)
  if (!opt.use_advancing_front) {
    std::cerr << "Warning: Neither region-growing nor advancing front enabled. Defaulting to region-growing." << std::endl;
    GreedyCutsOptions region_opt = opt;
    region_opt.use_region_growing = true;
    return triangulateRegionGrowing(elevations, width, height, mask, region_opt, out_mesh);
  }
  
  const int W = width, H = height;

  // Pre-compute terrain complexity map if enabled
  std::vector<double> complexity_map;
  if (opt.use_precomputed_complexity) {
    complexity_map = precompute_terrain_complexity(elevations, W, H, mask);
  }

  // Prepare vertices (grid x,y with z from elevations)
  out_mesh.vertices.resize(static_cast<size_t>(W) * static_cast<size_t>(H));
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      size_t vid = idx_row_major(x,y,W);
      out_mesh.vertices[vid] = {static_cast<double>(x),
                                static_cast<double>(y),
                                static_cast<double>(elevations[vid])};
    }
  }
  out_mesh.triangles.clear();
  out_mesh.triangles.reserve(static_cast<size_t>(W-1) * static_cast<size_t>(H-1) * 2);

  // Initialize front ring
  std::vector<FrontNode> nodes;
  nodes.reserve((W+H)*2);
  std::deque<int> front_ring;
  init_rect_front(W, H, nodes, front_ring);

  // Priority queue seeded by all available consecutive triples
  std::priority_queue<PQItem> pq;

  auto enqueue_node = [&](int mid) {
    if (!nodes[mid].alive) return;
    int p = nodes[mid].prev, n = nodes[mid].next;
    if (p == -1 || n == -1) return;
    const auto& A = out_mesh.vertices[nodes[p].vid];
    const auto& B = out_mesh.vertices[nodes[mid].vid];
    const auto& C = out_mesh.vertices[nodes[n].vid];
    // skip degenerate
    if (tri_area2d(A[0],A[1],B[0],B[1],C[0],C[1]) < opt.area_eps) return;
    // Skip if PQ is getting too large for performance
    if (pq.size() >= static_cast<size_t>(opt.max_pq_size)) return;
    // Compute error (no threshold yet, PQ is by error)
    auto terr = triangle_error_and_argmax(A, B, C, elevations, mask, W, H, opt, -1.0);
    double e = terr.any_valid ? terr.error : 0.0;
    pq.push(PQItem{mid, nodes[mid].version, e});
  };

  // Seed PQ
  if (!front_ring.empty()) {
    int start = front_ring.front();
    int curr = start;
    do {
      enqueue_node(curr);
      curr = nodes[curr].next;
    } while (curr != start && pq.size() < static_cast<size_t>(opt.max_initial_iterations));
  }

  // Advancing front
  int safety_iters = 0;
  auto local_threshold = [&](const std::array<double,3>& A,
                             const std::array<double,3>& B,
                             const std::array<double,3>& C){
    if (opt.use_precomputed_complexity && !complexity_map.empty()) {
      // Use pre-computed complexity from terrain map
      double cx = (A[0]+B[0]+C[0])/3.0;
      double cy = (A[1]+B[1]+C[1])/3.0;
      int icx = std::clamp(static_cast<int>(std::llround(cx)), 0, W-1);
      int icy = std::clamp(static_cast<int>(std::llround(cy)), 0, H-1);
      double complexity = complexity_map[idx_row_major(icx, icy, W)] * opt.complexity_scale_factor;
      
      // Split complexity back into slope and curvature components for threshold calculation
      double slope = complexity * 0.9; // most of complexity is slope
      double curvature = complexity * 0.1; // small portion is curvature
      return adaptive_threshold(opt.base_error_threshold, slope, opt.slope_weight, curvature, opt.curvature_weight);
    } else {
      // Fallback to direct calculation
      double cx = (A[0]+B[0]+C[0])/3.0;
      double cy = (A[1]+B[1]+C[1])/3.0;
      int icx = std::clamp(static_cast<int>(std::llround(cx)), 1, W-2);
      int icy = std::clamp(static_cast<int>(std::llround(cy)), 1, H-2);
      double slope = tri_local_slope(A,B,C);
      double curv  = tri_local_curvature_8n(W,H,elevations,icx,icy);
      return adaptive_threshold(opt.base_error_threshold, slope, opt.slope_weight, curv, opt.curvature_weight);
    }
  };

  while (!pq.empty() && safety_iters++ < opt.max_initial_iterations) {
    PQItem it = pq.top(); pq.pop();
    int mid = it.mid_node;
    if (mid < 0 || mid >= static_cast<int>(nodes.size())) continue;
    if (!nodes[mid].alive) continue;
    if (nodes[mid].version != it.version) continue; // stale

    int pv = nodes[mid].prev;
    int nv = nodes[mid].next;
    if (pv < 0 || nv < 0) continue;
    if (!nodes[pv].alive || !nodes[nv].alive) continue;

    const auto& A = out_mesh.vertices[nodes[pv].vid];
    const auto& B = out_mesh.vertices[nodes[mid].vid];
    const auto& C = out_mesh.vertices[nodes[nv].vid];
    if (tri_area2d(A[0],A[1],B[0],B[1],C[0],C[1]) < opt.area_eps) {
      erase_node(nodes, mid);
      continue;
    }

    double thresh = local_threshold(A,B,C);
    auto terr = triangle_error_and_argmax(A, B, C, elevations, mask, W, H, opt,
                                          opt.early_exit_error_eval ? thresh : -1.0);
    if (!terr.any_valid) {
      // if triangle covers only invalid samples and that's blocking, skip it (advance by removing mid to avoid stalling)
      if (opt.treat_all_invalid_as_blocking) { erase_node(nodes, mid); }
      continue;
    }

    if (terr.error <= thresh && triangle_is_quality_2d(A,B,C, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area)) {
      // accept triangle (pv,mid,nv)
      out_mesh.triangles.push_back({nodes[pv].vid, nodes[mid].vid, nodes[nv].vid});
      // remove mid
      int left = nodes[mid].prev;
      int right = nodes[mid].next;
      erase_node(nodes, mid);
      // enqueue affected neighbors (left, right)
      if (nodes[left].alive) enqueue_node(left);
      if (nodes[right].alive) enqueue_node(right);
    } else {
      // Not acceptable now; skip. Other PQ items may make progress.
      // To avoid infinite stalling, we could re-enqueue with updated version to revisit later.
      // Re-enqueue if still alive
      enqueue_node(mid);
    }
  }

  // Remainder fill using quality-aware “ear clipping” pass; never erase without adding a triangle
  if (!front_ring.empty()) {
    // Find any alive node to start
    int start = -1;
    for (size_t i = 0; i < nodes.size(); ++i) if (nodes[i].alive) { start = static_cast<int>(i); break; }
    if (start != -1) {
      int curr = start;
      int count_guard = 0;
      while (count_guard < (W+H)*4) {
        // Count alive nodes to know when done
        int alive_count = 0;
        int probe = curr;
        do {
          if (nodes[probe].alive) alive_count++;
          probe = nodes[probe].next;
        } while (probe != curr);
        if (alive_count < 3) break;

        bool ear_found = false;
        int tries = 0;
        int ear = curr;
        do {
          int p = nodes[ear].prev;
          int n = nodes[ear].next;
          const auto& A = out_mesh.vertices[nodes[p].vid];
          const auto& B = out_mesh.vertices[nodes[ear].vid];
          const auto& C = out_mesh.vertices[nodes[n].vid];
          if (tri_area2d(A[0],A[1],B[0],B[1],C[0],C[1]) >= opt.min_area &&
              triangle_is_quality_2d(A,B,C, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area))
          {
            out_mesh.triangles.push_back({nodes[p].vid, nodes[ear].vid, nodes[n].vid});
            int next_after = nodes[ear].next;
            erase_node(nodes, ear);
            curr = next_after;
            ear_found = true;
            break;
          }
          ear = nodes[ear].next;
        } while (++tries < alive_count);

        if (!ear_found) {
          // Relax: accept the ear with the largest area to make progress
          double best_area = -1.0;
          int best_ear = -1;
          int p = curr;
          for (int t = 0; t < alive_count; ++t) {
            if (!nodes[p].alive) { p = nodes[p].next; continue; }
            int pr = nodes[p].prev, nx = nodes[p].next;
            const auto& A = out_mesh.vertices[nodes[pr].vid];
            const auto& B = out_mesh.vertices[nodes[p].vid];
            const auto& C = out_mesh.vertices[nodes[nx].vid];
            double ar = tri_area2d(A[0],A[1],B[0],B[1],C[0],C[1]);
            if (ar > best_area) { best_area = ar; best_ear = p; }
            p = nodes[p].next;
          }
          if (best_ear != -1) {
            int pr = nodes[best_ear].prev, nx = nodes[best_ear].next;
            out_mesh.triangles.push_back({nodes[pr].vid, nodes[best_ear].vid, nodes[nx].vid});
            int na = nodes[best_ear].next;
            erase_node(nodes, best_ear);
            curr = na;
          } else {
            break; // nothing else to do
          }
        }
        count_guard++;
      }
    }
  }

  // ----------------- Multi-pass refinement with interior insertion -----------------
  auto refine_once = [&](std::vector<std::array<int,3>>& tris_in,
                         std::vector<std::array<int,3>>& tris_out) {
    bool refined_any = false;
    tris_out.clear();
    tris_out.reserve(tris_in.size()*2);

    for (const auto& t : tris_in) {
      const auto& A = out_mesh.vertices[t[0]];
      const auto& B = out_mesh.vertices[t[1]];
      const auto& C = out_mesh.vertices[t[2]];
      if (tri_area2d(A[0],A[1],B[0],B[1],C[0],C[1]) < opt.min_area) {
        continue;
      }
      // Local threshold calculation
      double cx = (A[0]+B[0]+C[0])/3.0;
      double cy = (A[1]+B[1]+C[1])/3.0;
      double thresh;
      if (opt.use_precomputed_complexity && !complexity_map.empty()) {
        // Use pre-computed complexity
        int icx = std::clamp(static_cast<int>(std::llround(cx)), 0, W-1);
        int icy = std::clamp(static_cast<int>(std::llround(cy)), 0, H-1);
        double complexity = complexity_map[idx_row_major(icx, icy, W)] * opt.complexity_scale_factor;
        double slope = complexity * 0.9;
        double curvature = complexity * 0.1;
        thresh = adaptive_threshold(opt.base_error_threshold, slope, opt.slope_weight, curvature, opt.curvature_weight);
      } else {
        // Fallback to direct calculation
        int icx = std::clamp(static_cast<int>(std::llround(cx)), 1, W-2);
        int icy = std::clamp(static_cast<int>(std::llround(cy)), 1, H-2);
        double slope = tri_local_slope(A,B,C);
        double curv  = tri_local_curvature_8n(W,H,elevations,icx,icy);
        thresh = adaptive_threshold(opt.base_error_threshold, slope, opt.slope_weight, curv, opt.curvature_weight);
      }

      // Error with early-exit - be more aggressive about early termination in refinement
      double early_exit_threshold = thresh * 2.0; // Accept if error is within 2x threshold 
      auto terr = triangle_error_and_argmax(A, B, C, elevations, mask, W, H, opt,
                                            opt.early_exit_error_eval ? early_exit_threshold : -1.0);
      if (!terr.any_valid) {
        // Skip masked-only triangles
        continue;
      }
      if (terr.error <= thresh) {
        if (triangle_is_quality_2d(A,B,C, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area))
          tris_out.push_back(t);
        continue;
      }

      // Find best point within triangle (allow edge points but avoid triangle vertices)
      double best_err = -1.0;
      int best_x = -1, best_y = -1;
      double minx = std::max(0.0, std::floor(std::min({A[0], B[0], C[0]})));
      double maxx = std::min(static_cast<double>(W-1), std::ceil(std::max({A[0], B[0], C[0]})));
      double miny = std::max(0.0, std::floor(std::min({A[1], B[1], C[1]})));
      double maxy = std::min(static_cast<double>(H-1), std::ceil(std::max({A[1], B[1], C[1]})));
      for (int sy = static_cast<int>(miny); sy <= static_cast<int>(maxy); ++sy) {
        for (int sx = static_cast<int>(minx); sx <= static_cast<int>(maxx); ++sx) {
          if (mask && mask[idx_row_major(sx,sy,W)] == 0) continue;
          
          // Skip if this point is already a triangle vertex
          if ((std::abs(sx - A[0]) < opt.bary_eps && std::abs(sy - A[1]) < opt.bary_eps) ||
              (std::abs(sx - B[0]) < opt.bary_eps && std::abs(sy - B[1]) < opt.bary_eps) ||
              (std::abs(sx - C[0]) < opt.bary_eps && std::abs(sy - C[1]) < opt.bary_eps)) {
            continue;
          }
          
          double u,v,w;
          if (!barycentric_uvwt(A,B,C, static_cast<double>(sx), static_cast<double>(sy), opt.det_eps, u,v,w)) continue;
          
          // Accept points that are inside the triangle (including edge points)
          // Only reject if point is too close to a triangle vertex (corner)
          bool too_close_to_vertex = (u >= 1.0 - opt.bary_eps && v <= opt.bary_eps && w <= opt.bary_eps) ||
                                    (v >= 1.0 - opt.bary_eps && u <= opt.bary_eps && w <= opt.bary_eps) ||
                                    (w >= 1.0 - opt.bary_eps && u <= opt.bary_eps && v <= opt.bary_eps);
          if (too_close_to_vertex) continue;
          
          double tri_z = u*A[2] + v*B[2] + w*C[2];
          double grid_z = static_cast<double>(elevations[idx_row_major(sx,sy,W)]);
          double err = std::abs(tri_z - grid_z);
          if (err > best_err) { best_err = err; best_x = sx; best_y = sy; }
        }
      }
      if (best_x == -1 || best_y == -1) {
        // No interior valid point found; keep or drop based on quality
        if (triangle_is_quality_2d(A,B,C, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area))
          tris_out.push_back(t);
        continue;
      }

      // Insert point and split into fan of three triangles
      int new_vid = static_cast<int>(idx_row_major(best_x, best_y, W));
      std::array<double,3> P = {static_cast<double>(best_x),
                                static_cast<double>(best_y),
                                static_cast<double>(elevations[idx_row_major(best_x,best_y,W)])};
      
      // Only keep sub-tris that pass quality
      std::array<std::array<int,3>,3> candidates = {{
        {t[0], t[1], new_vid},
        {t[1], t[2], new_vid},
        {t[2], t[0], new_vid}
      }};
      for (const auto& ct : candidates) {
        const auto& V0 = out_mesh.vertices[ct[0]];
        const auto& V1 = out_mesh.vertices[ct[1]];
        const auto& V2 = out_mesh.vertices[ct[2]];
        if (triangle_is_quality_2d(V0,V1,V2, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area))
          tris_out.push_back(ct);
      }
      refined_any = true;
    }
    return refined_any;
  };

  // Calculate initial mesh volume for convergence tracking
  auto calculate_mesh_volume = [&](const std::vector<std::array<int,3>>& triangles) -> double {
    double volume = 0.0;
    for (const auto& t : triangles) {
      const auto& v0 = out_mesh.vertices[t[0]];
      const auto& v1 = out_mesh.vertices[t[1]];
      const auto& v2 = out_mesh.vertices[t[2]];
      // Signed volume contribution of tetrahedron from origin
      volume += (v0[0] * (v1[1] * v2[2] - v1[2] * v2[1]) +
                 v1[0] * (v2[1] * v0[2] - v2[2] * v0[1]) +
                 v2[0] * (v0[1] * v1[2] - v0[2] * v1[1])) / 6.0;
    }
    return std::abs(volume);
  };

  // Run refinement passes with volume convergence check
  std::vector<std::array<int,3>> work = out_mesh.triangles;
  std::vector<std::array<int,3>> next;
  double prev_volume = calculate_mesh_volume(work);
  
  for (int pass = 0; pass < opt.max_refinement_passes; ++pass) {
    bool did_refine = refine_once(work, next);
    if (!did_refine) { break; }
    
    // Check volume convergence if enabled
    if (opt.use_volume_convergence && pass >= opt.min_volume_passes) {
      double curr_volume = calculate_mesh_volume(next);
      double volume_change = std::abs(curr_volume - prev_volume) / std::max(prev_volume, 1e-12);
      if (volume_change < opt.volume_convergence_threshold) {
        std::cerr << "Volume converged after " << (pass + 1) << " passes (change: " 
                  << (volume_change * 100.0) << "%)" << std::endl;
        work.swap(next);
        break;
      }
      prev_volume = curr_volume;
    }
    
    work.swap(next);
  }
  out_mesh.triangles.swap(work);
}

} // namespace terrascape
