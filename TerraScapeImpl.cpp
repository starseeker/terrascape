#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic push /* start new diagnostic pragma */
#  pragma GCC diagnostic ignored "-Wfloat-equal"
#elif defined(__clang__)
#  pragma clang diagnostic push /* start new diagnostic pragma */
#  pragma clang diagnostic ignored "-Wfloat-equal"
#endif

#include "TerraScapeImpl.h"
#include "TerraScape.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <chrono>
#include <map>
#include <set>
#include <tuple>
#include <random>
#include <limits>

namespace TerraScape {

// Forward declarations for Simulation of Simplicity functions
static float simulation_of_simplicity_perturbation(int x, int y, int width, int height, float magnitude);
static int simulation_of_simplicity_orientation(float ax, float ay, int ai, float bx, float by, int bi, float cx, float cy, int ci);
static bool detect_regular_pattern(const float* elevations, int width, int height);
static bool detect_collinear_rows(const float* elevations, int width, int height);

// Helper function for point-in-triangle test with Simulation of Simplicity support
static bool pointInTriangle(float px, float py, 
                           const detria::PointF& v0, const detria::PointF& v1, const detria::PointF& v2) {
    float dX = px - v2.x;
    float dY = py - v2.y;
    float dX21 = v2.x - v1.x;
    float dY12 = v1.y - v2.y;
    float D = (v0.x - v2.x) * dY12 + (v0.y - v2.y) * dX21;
    
    const float DEGENERACY_THRESHOLD = 1e-10f;
    
    if (std::abs(D) < DEGENERACY_THRESHOLD) {
        // Degenerate triangle - use Simulation of Simplicity
        // For degenerate triangles, we use lexicographic ordering based on coordinates
        // to make a consistent decision
        
        // Create pseudo-indices based on coordinates for SoS ordering
        // This ensures deterministic behavior for degenerate cases
        auto coord_to_index = [](float x, float y) -> int {
            // Convert coordinates to a deterministic integer index
            // This is a simplified approach for SoS tie-breaking
            return static_cast<int>(x * 1000000.0f) + static_cast<int>(y * 1000000.0f) * 1000000;
        };
        
        int idx_p = coord_to_index(px, py);
        int idx_0 = coord_to_index(v0.x, v0.y);
        int idx_1 = coord_to_index(v1.x, v1.y);
        int idx_2 = coord_to_index(v2.x, v2.y);
        
        // Use SoS orientation test for degenerate case
        int orientation = simulation_of_simplicity_orientation(
            v0.x, v0.y, idx_0,
            v1.x, v1.y, idx_1,
            px, py, idx_p
        );
        
        // For degenerate triangles, we conservatively return false unless
        // the SoS ordering strongly suggests inclusion
        return (orientation > 0) && (idx_p > std::min({idx_0, idx_1, idx_2}));
    }
    
    // Non-degenerate case: use standard barycentric coordinate test
    float s = ((v0.x - v2.x) * dY + (v0.y - v2.y) * dX) / D;
    float t = (dY12 * dX + dX21 * dY) / D;
    return (s >= -1e-6f) && (t >= -1e-6f) && (s + t <= 1.0f + 1e-6f);
}

static float barycentricInterp(float px, float py,
                              const detria::PointF& v0, const detria::PointF& v1, const detria::PointF& v2,
                              float z0, float z1, float z2) {
    float denom = ((v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y));
    
    const float DEGENERACY_THRESHOLD = 1e-10f;
    
    if (std::abs(denom) < DEGENERACY_THRESHOLD) {
        // Degenerate triangle - use Simulation of Simplicity approach
        // For degenerate triangles, use lexicographic ordering to choose a vertex
        
        // Find the lexicographically smallest vertex
        const detria::PointF* vertices[3] = {&v0, &v1, &v2};
        float z_values[3] = {z0, z1, z2};
        
        // Sort by lexicographic order (y first, then x)
        int min_idx = 0;
        for (int i = 1; i < 3; ++i) {
            if (vertices[i]->y < vertices[min_idx]->y || 
                (vertices[i]->y == vertices[min_idx]->y && vertices[i]->x < vertices[min_idx]->x)) {
                min_idx = i;
            }
        }
        
        // Return the z-value of the lexicographically smallest vertex
        // This provides a deterministic fallback for degenerate cases
        return z_values[min_idx];
    }
    
    // Non-degenerate case: use standard barycentric interpolation
    float w1 = ((v1.y - v2.y) * (px - v2.x) + (v2.x - v1.x) * (py - v2.y)) / denom;
    float w2 = ((v2.y - v0.y) * (px - v2.x) + (v0.x - v2.x) * (py - v2.y)) / denom;
    float w3 = 1.0f - w1 - w2;
    return w1 * z0 + w2 * z1 + w3 * z2;
}

// DetriaTriangulationManager implementation
DetriaTriangulationManager::DetriaTriangulationManager() 
    : triangulation_(std::make_unique<TriangulationT>())
    , is_triangulated_(false)
    , triangulation_version_(0) {
}

DetriaTriangulationManager::~DetriaTriangulationManager() = default;

void DetriaTriangulationManager::initializeBoundary(float minX, float minY, float maxX, float maxY,
                                                   float z00, float z10, float z11, float z01) {
    points_.clear();
    point_z_coords_.clear();
    boundary_indices_.clear();
    is_triangulated_ = false;
    
    // Add corner points
    points_.emplace_back(detria::PointF{minX, minY});    // 0: bottom-left
    points_.emplace_back(detria::PointF{maxX, minY});    // 1: bottom-right  
    points_.emplace_back(detria::PointF{maxX, maxY});    // 2: top-right
    points_.emplace_back(detria::PointF{minX, maxY});    // 3: top-left
    
    point_z_coords_.push_back(z00);
    point_z_coords_.push_back(z10);
    point_z_coords_.push_back(z11);
    point_z_coords_.push_back(z01);
    
    // Set boundary outline (counter-clockwise)
    boundary_indices_ = {0, 1, 2, 3};
    
    // Setup triangulation
    triangulation_->setPoints(points_);
    triangulation_->addOutline(boundary_indices_);
    
    // Perform initial triangulation
    is_triangulated_ = triangulation_->triangulate(true); // Delaunay
    
    if (!is_triangulated_) {
        std::cerr << "Warning: Initial boundary triangulation failed\n";
    }
}

uint32_t DetriaTriangulationManager::addPoint(float x, float y, float z) {
    uint32_t index = static_cast<uint32_t>(points_.size());
    points_.emplace_back(detria::PointF{x, y});
    point_z_coords_.push_back(z);
    
    // Mark that retriangulation is needed
    is_triangulated_ = false;
    
    return index;
}

float DetriaTriangulationManager::getPointZ(uint32_t index) const {
    if (index >= point_z_coords_.size()) return 0.0f;
    return point_z_coords_[index];
}

bool DetriaTriangulationManager::retriangulate() {
    if (is_triangulated_) return true;
    
    // Clear previous triangulation to prevent outline accumulation
    triangulation_->clear();
    
    // Invalidate spatial acceleration
    spatial_accel_.is_built = false;
    
    // Reset triangulation with all points
    triangulation_->setPoints(points_);
    triangulation_->addOutline(boundary_indices_);
    
    is_triangulated_ = triangulation_->triangulate(true); // Delaunay
    
    if (!is_triangulated_) {
        std::cerr << "Triangulation failed with " << points_.size() << " points. Error: " 
                  << triangulation_->getErrorMessage() << "\n";
    } else {
        // Increment version after successful triangulation
        triangulation_version_++;
    }
    
    return is_triangulated_;
}

void DetriaTriangulationManager::buildSpatialAcceleration() {
    if (!is_triangulated_ || spatial_accel_.is_built) return;
    
    // Determine bounds from points
    if (points_.empty()) return;
    
    spatial_accel_.minX = spatial_accel_.maxX = points_[0].x;
    spatial_accel_.minY = spatial_accel_.maxY = points_[0].y;
    
    for (const auto& point : points_) {
        spatial_accel_.minX = std::min(spatial_accel_.minX, point.x);
        spatial_accel_.maxX = std::max(spatial_accel_.maxX, point.x);
        spatial_accel_.minY = std::min(spatial_accel_.minY, point.y);
        spatial_accel_.maxY = std::max(spatial_accel_.maxY, point.y);
    }
    
    // Add small padding to handle edge cases
    float padding = 0.01f;
    spatial_accel_.minX -= padding;
    spatial_accel_.maxX += padding;
    spatial_accel_.minY -= padding;
    spatial_accel_.maxY += padding;
    
    // Choose grid resolution based on number of triangles
    // Aim for roughly 4-16 triangles per cell on average
    int triangle_count = 0;
    triangulation_->forEachTriangle([&](detria::Triangle<uint32_t>) { triangle_count++; });
    
    int target_cells = std::max(4, triangle_count / 8);  // 8 triangles per cell on average
    int grid_size = std::max(4, static_cast<int>(std::sqrt(target_cells)));
    
    spatial_accel_.gridWidth = grid_size;
    spatial_accel_.gridHeight = grid_size;
    
    // Initialize grid
    spatial_accel_.grid.assign(spatial_accel_.gridHeight, 
                               std::vector<std::vector<uint32_t>>(spatial_accel_.gridWidth));
    
    // Add triangles to grid cells
    uint32_t triangle_index = 0;
    triangulation_->forEachTriangle([&](detria::Triangle<uint32_t> triangle) {
        if (triangle.x >= points_.size() || triangle.y >= points_.size() || triangle.z >= points_.size()) {
            triangle_index++;
            return; // Invalid triangle indices
        }
        
        const auto& v0 = points_[triangle.x];
        const auto& v1 = points_[triangle.y]; 
        const auto& v2 = points_[triangle.z];
        
        // Find bounding box of triangle
        float minX = std::min({v0.x, v1.x, v2.x});
        float maxX = std::max({v0.x, v1.x, v2.x});
        float minY = std::min({v0.y, v1.y, v2.y});
        float maxY = std::max({v0.y, v1.y, v2.y});
        
        // Convert to grid coordinates
        float cellWidth = (spatial_accel_.maxX - spatial_accel_.minX) / spatial_accel_.gridWidth;
        float cellHeight = (spatial_accel_.maxY - spatial_accel_.minY) / spatial_accel_.gridHeight;
        
        int minCellX = std::max(0, static_cast<int>((minX - spatial_accel_.minX) / cellWidth));
        int maxCellX = std::min(spatial_accel_.gridWidth - 1, static_cast<int>((maxX - spatial_accel_.minX) / cellWidth));
        int minCellY = std::max(0, static_cast<int>((minY - spatial_accel_.minY) / cellHeight));
        int maxCellY = std::min(spatial_accel_.gridHeight - 1, static_cast<int>((maxY - spatial_accel_.minY) / cellHeight));
        
        // Add triangle to overlapping cells
        for (int cy = minCellY; cy <= maxCellY; cy++) {
            for (int cx = minCellX; cx <= maxCellX; cx++) {
                spatial_accel_.grid[cy][cx].push_back(triangle_index);
            }
        }
        
        triangle_index++;
    });
    
    spatial_accel_.is_built = true;
    
    std::cout << "Built spatial acceleration: " << spatial_accel_.gridWidth << "x" << spatial_accel_.gridHeight 
              << " grid for " << triangle_index << " triangles\n";
}

std::vector<uint32_t> DetriaTriangulationManager::findContainingTriangle(float x, float y) const {
    if (!is_triangulated_) return {};
    
    // Build spatial acceleration if not already built
    const_cast<DetriaTriangulationManager*>(this)->buildSpatialAcceleration();
    
    if (!spatial_accel_.is_built) {
        // Fallback to linear search if spatial acceleration failed
        std::vector<uint32_t> result;
        bool found = false;
        
        triangulation_->forEachTriangle([&](detria::Triangle<uint32_t> triangle) {
            if (found) return; // Already found, skip
            
            if (triangle.x >= points_.size() || triangle.y >= points_.size() || triangle.z >= points_.size()) {
                return; // Invalid triangle indices
            }
            
            const auto& v0 = points_[triangle.x];
            const auto& v1 = points_[triangle.y]; 
            const auto& v2 = points_[triangle.z];
            
            if (pointInTriangle(x, y, v0, v1, v2)) {
                result = {triangle.x, triangle.y, triangle.z};
                found = true;
            }
        });
        
        return result;
    }
    
    // Use spatial acceleration for fast lookup
    float cellWidth = (spatial_accel_.maxX - spatial_accel_.minX) / spatial_accel_.gridWidth;
    float cellHeight = (spatial_accel_.maxY - spatial_accel_.minY) / spatial_accel_.gridHeight;
    
    int cellX = static_cast<int>((x - spatial_accel_.minX) / cellWidth);
    int cellY = static_cast<int>((y - spatial_accel_.minY) / cellHeight);
    
    // Clamp to valid range
    cellX = std::max(0, std::min(cellX, spatial_accel_.gridWidth - 1));
    cellY = std::max(0, std::min(cellY, spatial_accel_.gridHeight - 1));
    
    // Check triangles in the cell
    const auto& candidates = spatial_accel_.grid[cellY][cellX];
    
    uint32_t triangle_index = 0;
    std::vector<uint32_t> result;
    
    triangulation_->forEachTriangle([&](detria::Triangle<uint32_t> triangle) {
        // Only check triangles in the candidate list
        bool should_check = std::find(candidates.begin(), candidates.end(), triangle_index) != candidates.end();
        triangle_index++;
        
        if (!should_check) return;
        
        if (triangle.x >= points_.size() || triangle.y >= points_.size() || triangle.z >= points_.size()) {
            return; // Invalid triangle indices
        }
        
        const auto& v0 = points_[triangle.x];
        const auto& v1 = points_[triangle.y]; 
        const auto& v2 = points_[triangle.z];
        
        if (pointInTriangle(x, y, v0, v1, v2)) {
            result = {triangle.x, triangle.y, triangle.z};
            return; // Found it, can exit early
        }
    });
    
    return result;
}

std::vector<std::vector<uint32_t>> DetriaTriangulationManager::getAllTriangles() const {
    std::vector<std::vector<uint32_t>> triangles;
    
    if (!is_triangulated_) return triangles;
    
    triangulation_->forEachTriangle([&](detria::Triangle<uint32_t> triangle) {
        if (triangle.x < points_.size() && triangle.y < points_.size() && triangle.z < points_.size()) {
            triangles.push_back({triangle.x, triangle.y, triangle.z});
        }
    });
    
    return triangles;
}

MeshResult DetriaTriangulationManager::toMeshResult() const {
    MeshResult result;
    
    // Convert points to vertices
    for (size_t i = 0; i < points_.size(); ++i) {
        const auto& pt = points_[i];
        float z = (i < point_z_coords_.size()) ? point_z_coords_[i] : 0.0f;
        result.vertices.push_back({pt.x, pt.y, z});
    }
    
    // Convert triangles with consistent CCW winding
    auto triangles = getAllTriangles();
    for (const auto& tri : triangles) {
        if (tri.size() == 3) {
            uint32_t v0_idx = tri[0];
            uint32_t v1_idx = tri[1]; 
            uint32_t v2_idx = tri[2];
            
            // Ensure vertices are valid
            if (v0_idx >= points_.size() || v1_idx >= points_.size() || v2_idx >= points_.size()) {
                continue;
            }
            
            const auto& v0 = points_[v0_idx];
            const auto& v1 = points_[v1_idx];
            const auto& v2 = points_[v2_idx];
            
            // Compute signed area to determine winding order
            // Signed area = (x1-x0)(y2-y0) - (y1-y0)(x2-x0)
            float signed_area = (v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x);
            
            // If negative, triangle is clockwise - swap v1 and v2 to make it CCW
            if (signed_area < 0.0f) {
                result.triangles.push_back({static_cast<int>(v0_idx), 
                                           static_cast<int>(v2_idx),  // Swapped
                                           static_cast<int>(v1_idx)}); // Swapped
            } else {
                result.triangles.push_back({static_cast<int>(v0_idx), 
                                           static_cast<int>(v1_idx), 
                                           static_cast<int>(v2_idx)});
            }
        }
    }
    
    return result;
}

// GreedyMeshRefiner implementation
GreedyMeshRefiner::GreedyMeshRefiner(DetriaTriangulationManager* manager, 
                                   float error_threshold, int point_limit, int batch_size)
    : triangulation_manager_(manager)
    , error_threshold_(error_threshold)
    , point_limit_(point_limit)
    , batch_size_(batch_size)
    , grid_width_(0)
    , grid_height_(0)
    , total_candidate_updates_(0)
    , total_error_calculations_(0) {
}

// CandidateSpatialIndex implementation
void GreedyMeshRefiner::CandidateSpatialIndex::build(int grid_w, int grid_h, 
                                                    float min_x, float min_y, float max_x, float max_y) {
    gridWidth = std::max(8, std::min(64, grid_w / 16)); // Adaptive grid size
    gridHeight = std::max(8, std::min(64, grid_h / 16));
    minX = min_x;
    minY = min_y;
    maxX = max_x;
    maxY = max_y;
    cellWidth = (maxX - minX) / gridWidth;
    cellHeight = (maxY - minY) / gridHeight;
    
    grid.assign(gridHeight, std::vector<std::vector<int>>(gridWidth));
    is_built = true;
}

void GreedyMeshRefiner::CandidateSpatialIndex::addCandidate(int key, float x, float y) {
    if (!is_built) return;
    
    int cellX = std::max(0, std::min(gridWidth - 1, static_cast<int>((x - minX) / cellWidth)));
    int cellY = std::max(0, std::min(gridHeight - 1, static_cast<int>((y - minY) / cellHeight)));
    
    grid[cellY][cellX].push_back(key);
}

std::vector<int> GreedyMeshRefiner::CandidateSpatialIndex::getCandidatesInRadius(float x, float y, float radius) const {
    std::vector<int> result;
    if (!is_built) return result;
    
    int min_cellX = std::max(0, static_cast<int>((x - radius - minX) / cellWidth));
    int max_cellX = std::min(gridWidth - 1, static_cast<int>((x + radius - minX) / cellWidth));
    int min_cellY = std::max(0, static_cast<int>((y - radius - minY) / cellHeight));
    int max_cellY = std::min(gridHeight - 1, static_cast<int>((y + radius - minY) / cellHeight));
    
    for (int cy = min_cellY; cy <= max_cellY; ++cy) {
        for (int cx = min_cellX; cx <= max_cellX; ++cx) {
            result.insert(result.end(), grid[cy][cx].begin(), grid[cy][cx].end());
        }
    }
    
    return result;
}

void GreedyMeshRefiner::CandidateSpatialIndex::clear() {
    grid.clear();
    is_built = false;
}

template<typename T>
void GreedyMeshRefiner::initializeCandidatesFromGrid(int width, int height, const T* elevations) {
    grid_width_ = width;
    grid_height_ = height;
    
    // Clear existing data
    grid_candidates_.clear();
    candidate_heap_ = std::priority_queue<CandidatePoint>();
    spatial_index_.clear();
    
    std::cout << "Starting optimized incremental point insertion" << std::endl;
    
    // Build spatial index for candidates
    spatial_index_.build(width, height, 0.0f, 0.0f, 
                        static_cast<float>(width-1), static_cast<float>(height-1));
    
    // Use aggressive sampling for large grids to avoid memory explosion
    int max_initial_candidates = 10000; // Reduce further for stability
    int sample_step = 1;
    
    // Calculate sampling step to stay under limit
    int total_points = width * height;
    if (total_points > max_initial_candidates) {
        sample_step = static_cast<int>(std::sqrt(static_cast<double>(total_points) / max_initial_candidates));
        sample_step = std::max(2, sample_step); // Minimum step of 2 for large grids
    }
    
    // Ensure triangulation is up to date
    if (!triangulation_manager_->retriangulate()) {
        std::cerr << "Warning: Triangulation failed, cannot initialize candidates\n";
        return;
    }
    
    int active_candidates = 0;
    
    // Generate initial candidate set with smart sampling
    for (int y = sample_step; y < height - sample_step; y += sample_step) {
        for (int x = sample_step; x < width - sample_step; x += sample_step) {
            CandidatePoint candidate = createCandidateFromGrid(x, y, width, height, elevations);
            
            int key = y * width + x;
            grid_candidates_[key] = candidate;
            spatial_index_.addCandidate(key, candidate.world_x, candidate.world_y);
            
            // Only add to heap if error is significant
            if (candidate.error > error_threshold_) {
                candidate_heap_.push(candidate);
                active_candidates++;
            }
        }
    }
    
    std::cout << "Initialized " << grid_candidates_.size() << " grid candidates (step=" << sample_step 
              << "), " << active_candidates << " in active heap" << std::endl;
}

template<typename T>
void GreedyMeshRefiner::generateCandidatesInRegion(int width, int height, const T* elevations,
                                                  int min_x, int min_y, int max_x, int max_y) {
    // Generate additional candidates in a specific region when needed
    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            if (x < 0 || x >= width || y < 0 || y >= height) continue;
            
            int key = y * width + x;
            if (grid_candidates_.find(key) != grid_candidates_.end()) continue; // Already exists
            if (inserted_keys_.find(key) != inserted_keys_.end()) continue; // Already inserted
            
            CandidatePoint candidate = createCandidateFromGrid(x, y, width, height, elevations);
            grid_candidates_[key] = candidate;
            spatial_index_.addCandidate(key, candidate.world_x, candidate.world_y);
            
            if (candidate.error > error_threshold_) {
                candidate_heap_.push(candidate);
            }
        }
    }
}

template<typename T>
int GreedyMeshRefiner::refineIncrementally(int width, int height, const T* elevations) {
    int points_added = 0;
    std::vector<CandidatePoint> batch_candidates;
    
    std::cout << "Starting batch insertion with batch size: " << batch_size_ << "\n";
    
    while (hasViableCandidates() && 
           triangulation_manager_->getPointCount() < static_cast<size_t>(point_limit_)) {
        
        // Collect a batch of candidates
        batch_candidates.clear();
        
        while (batch_candidates.size() < static_cast<size_t>(batch_size_) && 
               hasViableCandidates() && 
               triangulation_manager_->getPointCount() + batch_candidates.size() < static_cast<size_t>(point_limit_)) {
            
            // Get the best candidate
            CandidatePoint best = candidate_heap_.top();
            candidate_heap_.pop();
            
            if (best.error < error_threshold_) {
                break; // All remaining candidates have acceptable error
            }
            
            // Check if candidate is stale (triangulation version has changed)
            if (best.tri_version != triangulation_manager_->getVersion()) {
                // Recalculate with current triangulation
                CandidatePoint updated = createCandidateFromGrid(best.x, best.y, width, height, elevations);
                if (updated.error > error_threshold_) {
                    candidate_heap_.push(updated);
                }
                continue; // Try next candidate
            }
            
            // Check for duplicate insertion guard
            int key = best.y * width + best.x;
            if (inserted_keys_.find(key) != inserted_keys_.end()) {
                continue; // Skip already inserted point
            }
            
            // Verify this candidate is still valid (hasn't been invalidated by previous insertions)
            auto it = grid_candidates_.find(key);
            if (it == grid_candidates_.end() || it->second.needs_update) {
                // Recalculate this candidate
                if (it != grid_candidates_.end()) {
                    CandidatePoint updated = createCandidateFromGrid(best.x, best.y, width, height, elevations);
                    it->second = updated;
                    if (updated.error > error_threshold_) {
                        candidate_heap_.push(updated);
                    }
                }
                continue; // Try next candidate
            }
            
            // Add to batch
            batch_candidates.push_back(best);
        }
        
        // If no valid candidates in batch, exit
        if (batch_candidates.empty()) {
            break;
        }
        
        // Add all points in the batch
        for (const auto& candidate : batch_candidates) {
            float actual_z = static_cast<float>(elevations[candidate.y * width + candidate.x]);
            triangulation_manager_->addPoint(candidate.world_x, candidate.world_y, actual_z);
            
            // Mark this point as inserted to prevent duplicates
            int key = candidate.y * width + candidate.x;
            inserted_keys_.insert(key);
            
            // Remove this candidate from our tracking
            grid_candidates_.erase(key);
        }
        
        // Retriangulate once for the entire batch
        if (!triangulation_manager_->retriangulate()) {
            std::cerr << "Warning: Retriangulation failed after adding batch of " << batch_candidates.size() << " points" << std::endl;
            std::cerr << "Implementing progressive fallback strategy..." << std::endl;
            
            // Progressive fallback: try smaller batches or reduce point density
            if (batch_size_ > 8) {
                batch_size_ = std::max(4, batch_size_ / 2); // Reduce batch size
                std::cout << "Reduced batch size to " << batch_size_ << " for stability" << std::endl;
            } else {
                // If batch size is already small, stop adding points to preserve stability
                std::cout << "Stopping point insertion to maintain triangulation stability" << std::endl;
                break;
            }
        }
        
        points_added += batch_candidates.size();
        
        // Update affected candidates for all inserted points
        for (const auto& candidate : batch_candidates) {
            updateAffectedCandidates(width, height, elevations, candidate.world_x, candidate.world_y);
        }
        
        if (points_added % 10 == 0) {
            std::cout << "Added " << points_added << " points in batches (last batch: " << batch_candidates.size() << ")\n";
        }
    }
    
    return points_added;
}

template<typename T>
void GreedyMeshRefiner::updateAffectedCandidates(int width, int height, const T* elevations, 
                                                float inserted_x, float inserted_y) {
    total_candidate_updates_++;
    
    // Use much smaller, more targeted radius to reduce update overhead
    float search_radius = std::min(6.0f, std::max(width, height) / 60.0f); // Reduced further
    
    // Cap the number of candidates to update per insertion for stability
    int max_updates_per_insertion = 50;
    
    // Use spatial index for efficient lookup
    auto affected_keys = spatial_index_.getCandidatesInRadius(inserted_x, inserted_y, search_radius);
    
    int updated_count = 0;
    for (int key : affected_keys) {
        if (updated_count >= max_updates_per_insertion) break; // Safety limit
        
        auto it = grid_candidates_.find(key);
        if (it != grid_candidates_.end()) {
            // Check distance more precisely to avoid unnecessary updates
            float dx = it->second.world_x - inserted_x;
            float dy = it->second.world_y - inserted_y;
            float distance_sq = dx * dx + dy * dy;
            
            // Only update if really close to the insertion point
            if (distance_sq <= search_radius * search_radius) {
                // Recalculate this candidate's error
                CandidatePoint updated = createCandidateFromGrid(it->second.x, it->second.y, width, height, elevations);
                it->second = updated;
                it->second.needs_update = false;
                
                // Add to heap if error is still significant
                if (updated.error > error_threshold_) {
                    candidate_heap_.push(updated);
                }
                updated_count++;
            }
        }
    }
    
    // Generate additional candidates in the immediate vicinity if needed
    if (updated_count < 10) { // If very few candidates were affected, generate more
        int region_size = 3; // Small region around insertion point
        generateCandidatesInRegion(width, height, elevations,
                                 static_cast<int>(inserted_x) - region_size,
                                 static_cast<int>(inserted_y) - region_size,
                                 static_cast<int>(inserted_x) + region_size,
                                 static_cast<int>(inserted_y) + region_size);
    }
    
    if (updated_count > 0) {
        std::cout << "Updated " << updated_count << " affected candidates" << std::endl;
    }
}

template<typename T>
GreedyMeshRefiner::CandidatePoint GreedyMeshRefiner::createCandidateFromGrid(int x, int y, int width, int height, 
                                                         const T* elevations) {
    total_error_calculations_++;
    
    CandidatePoint candidate;
    candidate.x = x;
    candidate.y = y;
    candidate.world_x = static_cast<float>(x);
    candidate.world_y = static_cast<float>(y);
    candidate.needs_update = false;
    candidate.tri_version = triangulation_manager_->getVersion();
    
    // Find containing triangle
    auto triangle = triangulation_manager_->findContainingTriangle(candidate.world_x, candidate.world_y);
    candidate.containing_triangle = triangle;
    
    if (triangle.empty()) {
        candidate.error = 0.0f;
    } else {
        candidate.error = calculateError(x, y, elevations, width, height, triangle);
    }
    
    return candidate;
}

GreedyMeshRefiner::CandidatePoint GreedyMeshRefiner::getBestCandidate() const {
    if (candidate_heap_.empty()) {
        return CandidatePoint{}; // Default constructed
    }
    return candidate_heap_.top();
}

bool GreedyMeshRefiner::hasViableCandidates() const {
    return !candidate_heap_.empty() && candidate_heap_.top().error >= error_threshold_;
}

std::vector<std::pair<int, int>> GreedyMeshRefiner::findAffectedGridPoints(float inserted_x, float inserted_y, 
                                                                          float search_radius) const {
    std::vector<std::pair<int, int>> affected_points;
    
    int min_x = std::max(0, static_cast<int>(inserted_x - search_radius));
    int max_x = std::min(grid_width_ - 1, static_cast<int>(inserted_x + search_radius));
    int min_y = std::max(0, static_cast<int>(inserted_y - search_radius));
    int max_y = std::min(grid_height_ - 1, static_cast<int>(inserted_y + search_radius));
    
    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            float dx = static_cast<float>(x) - inserted_x;
            float dy = static_cast<float>(y) - inserted_y;
            float distance = std::sqrt(dx * dx + dy * dy);
            
            if (distance <= search_radius) {
                affected_points.emplace_back(x, y);
            }
        }
    }
    
    return affected_points;
}

template<typename T>
float GreedyMeshRefiner::calculateError(int gx, int gy, const T* elevations, int width, int,
                                       const std::vector<uint32_t>& triangle) const {
    if (triangle.size() != 3) return 0.0f;
    
    // Get actual elevation
    float actual = static_cast<float>(elevations[gy * width + gx]);
    
    // Get interpolated elevation from triangle
    float approx = barycentricInterpolation(static_cast<float>(gx), static_cast<float>(gy), triangle);
    
    return std::abs(actual - approx);
}

float GreedyMeshRefiner::barycentricInterpolation(float px, float py, 
                                                 const std::vector<uint32_t>& triangle) const {
    if (triangle.size() != 3) return 0.0f;
    
    const auto& v0 = triangulation_manager_->getPoint(triangle[0]);
    const auto& v1 = triangulation_manager_->getPoint(triangle[1]);
    const auto& v2 = triangulation_manager_->getPoint(triangle[2]);
    
    float z0 = triangulation_manager_->getPointZ(triangle[0]);
    float z1 = triangulation_manager_->getPointZ(triangle[1]);
    float z2 = triangulation_manager_->getPointZ(triangle[2]);
    
    return barycentricInterp(px, py, v0, v1, v2, z0, z1, z2);
}

// Explicit template instantiations for common types
template void GreedyMeshRefiner::initializeCandidatesFromGrid<float>(int width, int height, const float* elevations);
template void GreedyMeshRefiner::initializeCandidatesFromGrid<double>(int width, int height, const double* elevations);
template void GreedyMeshRefiner::initializeCandidatesFromGrid<int>(int width, int height, const int* elevations);

template int GreedyMeshRefiner::refineIncrementally<float>(int width, int height, const float* elevations);
template int GreedyMeshRefiner::refineIncrementally<double>(int width, int height, const double* elevations);
template int GreedyMeshRefiner::refineIncrementally<int>(int width, int height, const int* elevations);

template void GreedyMeshRefiner::updateAffectedCandidates<float>(int width, int height, const float* elevations, float inserted_x, float inserted_y);
template void GreedyMeshRefiner::updateAffectedCandidates<double>(int width, int height, const double* elevations, float inserted_x, float inserted_y);
template void GreedyMeshRefiner::updateAffectedCandidates<int>(int width, int height, const int* elevations, float inserted_x, float inserted_y);

// --- Simulation of Simplicity Implementation ---

/**
 * Detect regular patterns in the elevation grid that might cause
 * triangulation degeneracy (e.g., checkerboard patterns, regular waves)
 */
static bool detect_regular_pattern(const float* elevations, int width, int height) {
    if (width < 3 || height < 3) return false;
    
    // Check for simple repeating patterns in small regions
    const int SAMPLE_SIZE = std::min(8, std::min(width, height));
    int pattern_matches = 0;
    int total_checks = 0;
    
    for (int y = 1; y < height - 1 && y < SAMPLE_SIZE; ++y) {
        for (int x = 1; x < width - 1 && x < SAMPLE_SIZE; ++x) {
            float current = elevations[y * width + x];
            float left = elevations[y * width + (x - 1)];
            float right = elevations[y * width + (x + 1)];
            float up = elevations[(y - 1) * width + x];
            float down = elevations[(y + 1) * width + x];
            
            // Check if this point follows a regular pattern with its neighbors
            bool horizontal_pattern = (std::abs(current - left) < 1e-10f && std::abs(current - right) < 1e-10f);
            bool vertical_pattern = (std::abs(current - up) < 1e-10f && std::abs(current - down) < 1e-10f);
            
            if (horizontal_pattern || vertical_pattern) {
                pattern_matches++;
            }
            total_checks++;
        }
    }
    
    // If more than 80% of sampled points follow a regular pattern, it's likely degenerate
    return total_checks > 0 && (static_cast<float>(pattern_matches) / total_checks > 0.8f);
}

/**
 * Detect collinear configurations along rows or columns that might
 * cause issues during triangulation
 */
static bool detect_collinear_rows(const float* elevations, int width, int height) {
    if (width < 3 || height < 3) return false;
    
    const float COLLINEAR_THRESHOLD = 1e-10f;
    int collinear_rows = 0;
    int collinear_cols = 0;
    
    // Check rows for collinearity
    for (int y = 0; y < height; ++y) {
        bool row_collinear = true;
        for (int x = 2; x < width; ++x) {
            float z0 = elevations[y * width + (x - 2)];
            float z1 = elevations[y * width + (x - 1)];
            float z2 = elevations[y * width + x];
            
            // Check if three consecutive points are collinear (constant slope)
            float slope1 = z1 - z0;
            float slope2 = z2 - z1;
            
            if (std::abs(slope1 - slope2) > COLLINEAR_THRESHOLD) {
                row_collinear = false;
                break;
            }
        }
        if (row_collinear) collinear_rows++;
    }
    
    // Check columns for collinearity  
    for (int x = 0; x < width; ++x) {
        bool col_collinear = true;
        for (int y = 2; y < height; ++y) {
            float z0 = elevations[(y - 2) * width + x];
            float z1 = elevations[(y - 1) * width + x];
            float z2 = elevations[y * width + x];
            
            // Check if three consecutive points are collinear (constant slope)
            float slope1 = z1 - z0;
            float slope2 = z2 - z1;
            
            if (std::abs(slope1 - slope2) > COLLINEAR_THRESHOLD) {
                col_collinear = false;
                break;
            }
        }
        if (col_collinear) collinear_cols++;
    }
    
    // If more than 50% of rows or columns are collinear, it's problematic
    return (collinear_rows > height / 2) || (collinear_cols > width / 2);
}

/**
 * Create deterministic perturbation using Simulation of Simplicity principles.
 * 
 * This function implements a simplified version of Edelsbrunner & Mücke's 
 * Simulation of Simplicity approach for handling degenerate cases in 
 * computational geometry.
 * 
 * Key properties:
 * - Deterministic: Same input always produces same output
 * - Lexicographic ordering: Uses point position for tie-breaking
 * - Minimal perturbation: Only breaks exact degeneracies
 * - Preserves relative ordering where possible
 * 
 * @param x,y Grid coordinates of the point
 * @param width,height Grid dimensions for normalization
 * @param magnitude Base perturbation magnitude
 * @return Deterministic perturbation value
 */
static float simulation_of_simplicity_perturbation(int x, int y, int width, int height, float magnitude) {
    // Use lexicographic ordering: first by y-coordinate, then by x-coordinate
    // This creates a deterministic hierarchy for tie-breaking
    
    // Normalize coordinates to [0,1] range for consistent scaling
    float normalized_x = static_cast<float>(x) / static_cast<float>(width - 1);
    float normalized_y = static_cast<float>(y) / static_cast<float>(height - 1);
    
    // Create lexicographic ordering using powers of epsilon
    // Point (x,y) gets perturbation: magnitude * (y + x*epsilon)
    // This ensures that for any two points, one will have strictly larger perturbation
    const float EPSILON = 1e-6f;  // Small value for lexicographic separation
    
    // Primary ordering by y-coordinate, secondary by x-coordinate
    float lexicographic_value = normalized_y + normalized_x * EPSILON;
    
    // Scale by magnitude and add small coordinate-dependent term
    // This ensures that even points with identical primary coordinates get unique perturbations
    float perturbation = magnitude * lexicographic_value;
    
    // Add a tiny coordinate-dependent term to ensure uniqueness
    // Use a simple hash-like function that's deterministic
    int coord_hash = (x * 73856093) ^ (y * 19349663);  // Large primes for good distribution
    float hash_contribution = magnitude * 1e-3f * static_cast<float>(coord_hash % 1000) / 1000.0f;
    
    return perturbation + hash_contribution;
}

/**
 * Enhanced geometric predicate that handles exact degeneracies using
 * Simulation of Simplicity principles.
 * 
 * When a standard geometric test returns exactly zero (indicating degeneracy),
 * this function uses lexicographic tie-breaking to provide a consistent,
 * deterministic result.
 */
static int simulation_of_simplicity_orientation(
    float ax, float ay, int ai,  // Point A with index ai
    float bx, float by, int bi,  // Point B with index bi  
    float cx, float cy, int ci   // Point C with index ci
) {
    // Standard orientation test using determinant
    float det = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    
    const float EPSILON = 1e-12f;  // Tolerance for exact zero
    
    if (std::abs(det) > EPSILON) {
        // Non-degenerate case: return standard result
        return (det > 0) ? 1 : -1;  // 1 = counterclockwise, -1 = clockwise
    }
    
    // Degenerate case: apply Simulation of Simplicity tie-breaking
    // Use lexicographic ordering of point indices for deterministic result
    
    // Create a deterministic ordering based on indices
    std::vector<int> indices = {ai, bi, ci};
    std::sort(indices.begin(), indices.end());
    
    // Use the smallest index to break ties deterministically
    // This ensures the same result for the same set of points regardless of order
    int tie_breaker = indices[0];
    
    if (tie_breaker == ai) return 1;   // A wins -> counterclockwise
    if (tie_breaker == bi) return -1;  // B wins -> clockwise  
    return 1;  // C wins -> counterclockwise (default)
}

// --- Input Preprocessing Implementation ---

template<typename T>
PreprocessingResult preprocess_input_data(
    int width, int height, const T* elevations, 
    float& error_threshold, bool enable_jitter)
{
    PreprocessingResult result;
    result.processed_elevations.reserve(size_t(width) * size_t(height));
    
    // Step 1: Convert to float and find min/max
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    int invalid_count = 0;
    
    for (int i = 0; i < width * height; ++i) {
        float z = static_cast<float>(elevations[i]);
        if (!std::isfinite(z)) {
            invalid_count++;
            z = 0.0f; // Temporary fallback, will be fixed below
        }
        result.processed_elevations.push_back(z);
        if (std::isfinite(z)) {
            min_z = std::min(min_z, z);
            max_z = std::max(max_z, z);
        }
    }
    
    // Step 2: Handle invalid values (NaN, inf) by interpolation
    if (invalid_count > 0) {
        for (int i = 0; i < width * height; ++i) {
            float& z = result.processed_elevations[i];
            if (!std::isfinite(static_cast<float>(elevations[i]))) {
                // Replace invalid values with interpolated values from neighbors
                float replacement = min_z; // Default fallback
                int valid_neighbors = 0;
                
                int y = i / width;
                int x = i % width;
                
                // Check 8-connected neighbors
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            int neighbor_idx = ny * width + nx;
                            float neighbor_z = static_cast<float>(elevations[neighbor_idx]);
                            if (std::isfinite(neighbor_z)) {
                                replacement += neighbor_z;
                                valid_neighbors++;
                            }
                        }
                    }
                }
                
                if (valid_neighbors > 0) {
                    z = replacement / (valid_neighbors + 1); // Include the default fallback
                } else {
                    z = min_z;  // Fallback to minimum valid value
                }
            }
        }
        
        result.warnings.push_back("Replaced " + std::to_string(invalid_count) + 
                                 " invalid values (NaN/inf) with interpolated values");
        result.has_warnings = true;
        
        // Recalculate min/max after fixing invalid values
        min_z = *std::min_element(result.processed_elevations.begin(), result.processed_elevations.end());
        max_z = *std::max_element(result.processed_elevations.begin(), result.processed_elevations.end());
    }
    
    // Step 3: Detect various forms of degeneracy and apply Simulation of Simplicity
    float elevation_range = max_z - min_z;
    const float PERFECTLY_FLAT_THRESHOLD = 0.0f; // Perfectly flat data
    const float NEARLY_FLAT_THRESHOLD = 1e-12f;  // Nearly flat data that might cause numerical issues
    
    // Check for perfect flatness
    bool is_perfectly_flat = (elevation_range <= PERFECTLY_FLAT_THRESHOLD);
    
    // Check for near-flatness that could cause numerical instability
    bool is_nearly_flat = (elevation_range > 0.0f && elevation_range <= NEARLY_FLAT_THRESHOLD);
    
    // Check for regular grid patterns that might cause degeneracy in triangulation
    bool has_regular_pattern = detect_regular_pattern(result.processed_elevations.data(), width, height);
    
    // Check for collinear configurations along rows/columns
    bool has_collinear_rows = detect_collinear_rows(result.processed_elevations.data(), width, height);
    
    if (is_perfectly_flat || is_nearly_flat || has_regular_pattern || has_collinear_rows) {
        result.is_degenerate = true;
        
        if (is_perfectly_flat) {
            result.warnings.push_back("Grid is perfectly flat (elevation range: " + 
                                     std::to_string(elevation_range) + ")");
        } else if (is_nearly_flat) {
            result.warnings.push_back("Grid is nearly flat with potential numerical issues (elevation range: " + 
                                     std::to_string(elevation_range) + ")");
        }
        
        if (has_regular_pattern) {
            result.warnings.push_back("Detected regular pattern that may cause triangulation degeneracy");
        }
        
        if (has_collinear_rows) {
            result.warnings.push_back("Detected collinear rows/columns that may cause triangulation issues");
        }
        
        result.has_warnings = true;
        
        if (enable_jitter) {
            // Apply Simulation of Simplicity (SoS) inspired deterministic perturbation
            // This ensures reproducible results while breaking exact coplanarity
            result.needs_jitter = true;
            
            // Use different perturbation magnitudes based on severity
            float perturbation_magnitude;
            if (is_perfectly_flat) {
                perturbation_magnitude = 1e-10f;  // Minimal for perfectly flat
            } else if (is_nearly_flat) {
                perturbation_magnitude = elevation_range * 1e-6f;  // Proportional to existing variation
            } else {
                perturbation_magnitude = std::max(1e-12f, elevation_range * 1e-9f);  // Very small for other cases
            }
            
            for (int i = 0; i < width * height; ++i) {
                int y = i / width;
                int x = i % width;
                
                // Create deterministic perturbation using lexicographic ordering
                // Each point gets a unique infinitesimal perturbation based on its position
                // This breaks ties consistently and deterministically
                float perturbation = simulation_of_simplicity_perturbation(x, y, width, height, perturbation_magnitude);
                result.processed_elevations[i] += perturbation;
            }
            
            result.warnings.push_back("Applied Simulation of Simplicity perturbation for deterministic degeneracy handling");
            
            // Recalculate min/max after perturbation
            min_z = *std::min_element(result.processed_elevations.begin(), result.processed_elevations.end());
            max_z = *std::max_element(result.processed_elevations.begin(), result.processed_elevations.end());
        }
    }
    
    // Step 4: Clamp error threshold to reasonable minimum (more conservative)
    const float MIN_ERROR_THRESHOLD = 1e-8f; // Smaller minimum to avoid changing most inputs
    result.adjusted_error_threshold = error_threshold;
    
    if (error_threshold <= 0.0f) {
        result.adjusted_error_threshold = MIN_ERROR_THRESHOLD;
        result.warnings.push_back("Error threshold was zero or negative (" + std::to_string(error_threshold) + 
                                 "), set to " + std::to_string(MIN_ERROR_THRESHOLD));
        result.has_warnings = true;
    }
    
    // Step 5: Check for extremely badly scaled coordinates (only extreme cases)
    float max_coord = std::max(static_cast<float>(width), static_cast<float>(height));
    float max_elevation_abs = std::max(std::abs(min_z), std::abs(max_z));
    
    // Only normalize for truly extreme cases that would cause numerical issues
    const float EXTREME_LARGE_THRESHOLD = 1e9f;
    const float EXTREME_SMALL_THRESHOLD = 1e-9f;
    
    if (max_coord > EXTREME_LARGE_THRESHOLD || max_elevation_abs > EXTREME_LARGE_THRESHOLD || 
        (max_elevation_abs > 0 && max_elevation_abs < EXTREME_SMALL_THRESHOLD)) {
        
        // Normalize elevations to reasonable range
        if (elevation_range > 0) {
            result.scale_factor = 1000.0f / elevation_range;  // Scale to ~1000 unit range
            result.z_offset = -min_z;  // Shift minimum to zero
            
            for (float& z : result.processed_elevations) {
                z = (z + result.z_offset) * result.scale_factor;
            }
            
            // Adjust error threshold proportionally
            result.adjusted_error_threshold *= result.scale_factor;
            
            result.warnings.push_back("Normalized extreme coordinate values (scale: " + 
                                     std::to_string(result.scale_factor) + 
                                     ", offset: " + std::to_string(result.z_offset) + ")");
            result.has_warnings = true;
        }
    }
    
    // Log warnings if any were generated
    if (result.has_warnings) {
        std::cout << "Input preprocessing warnings:\n";
        for (const auto& warning : result.warnings) {
            std::cout << "  - " << warning << "\n";
        }
    }
    
    return result;
}

// --- Main grid_to_mesh_detria implementation ---
template<typename T>
MeshResult grid_to_mesh_detria(
    int width, int height, const T* elevations,
    float error_threshold, int point_limit,
    MeshRefineStrategy strategy)
{
    // === PREPROCESSING FOR ROBUSTNESS ===
    // Preprocess input data to handle degenerate cases and prevent assertion failures
    float adjusted_error_threshold = error_threshold;
    PreprocessingResult preprocessing = preprocess_input_data(width, height, elevations, adjusted_error_threshold);
    
    // Use preprocessed data for all subsequent operations
    const float* processed_elevations = preprocessing.processed_elevations.data();
    error_threshold = preprocessing.adjusted_error_threshold;
    
    // If the input is severely degenerate (completely flat), return a minimal mesh
    if (preprocessing.is_degenerate && !preprocessing.needs_jitter) {
        MeshResult simple_result;
        
        // Create a simple quad mesh for flat grids
        simple_result.vertices = {
            {0.0f, 0.0f, processed_elevations[0]},                                    // bottom-left
            {static_cast<float>(width - 1), 0.0f, processed_elevations[width - 1]},  // bottom-right
            {static_cast<float>(width - 1), static_cast<float>(height - 1), 
             processed_elevations[(height - 1) * width + width - 1]},                // top-right
            {0.0f, static_cast<float>(height - 1), processed_elevations[(height - 1) * width]} // top-left
        };
        
        simple_result.triangles = {
            {0, 1, 2},  // First triangle
            {0, 2, 3}   // Second triangle
        };
        
        return simple_result;
    }
    
    // Create triangulation manager and initialize with boundary
    auto triangulation_manager = std::make_unique<DetriaTriangulationManager>();
    
    // Initialize boundary corners
    float minX = 0.0f, minY = 0.0f;
    float maxX = static_cast<float>(width - 1);
    float maxY = static_cast<float>(height - 1);
    
    triangulation_manager->initializeBoundary(
        minX, minY, maxX, maxY,
        processed_elevations[0],                                    // z00: bottom-left
        processed_elevations[width - 1],                           // z10: bottom-right
        processed_elevations[(height - 1) * width + width - 1],    // z11: top-right
        processed_elevations[(height - 1) * width]                 // z01: top-left
    );
    
    // Create greedy refiner for incremental insertion with batch processing
    // Use adaptive batch size: larger batches for larger point limits, but cap at reasonable size
    int batch_size = std::min(64, std::max(8, point_limit / 10));
    auto refiner = std::make_unique<GreedyMeshRefiner>(
        triangulation_manager.get(), error_threshold, point_limit, batch_size);
    
    if (strategy == MeshRefineStrategy::SPARSE) {
        // For sparse strategy, use regular sampling instead of error-driven
        int step = std::max(1, std::max(width, height) / 10);  // Denser sampling for better results
        std::cout << "Using SPARSE strategy with step=" << step << "\n";
        
        // Build list of sparse points first
        std::vector<std::tuple<float, float, float>> sparse_points;
        for (int y = step; y < height; y += step) {
            for (int x = step; x < width; x += step) {
                // Avoid points on boundary edges - they must be strictly interior
                bool on_boundary = (x == 0 || x == width - 1 || y == 0 || y == height - 1);
                if (!on_boundary && sparse_points.size() < static_cast<size_t>(point_limit - 4)) {
                    float px = static_cast<float>(x);
                    float py = static_cast<float>(y);
                    float pz = processed_elevations[y * width + x];
                    sparse_points.emplace_back(px, py, pz);
                }
            }
        }
        
        // Add points with progressive fallback if triangulation fails
        size_t points_to_try = sparse_points.size();
        bool success = false;
        
        while (!success && points_to_try > 0) {
            // Reset and add boundary + subset of sparse points
            triangulation_manager = std::make_unique<DetriaTriangulationManager>();
            triangulation_manager->initializeBoundary(minX, minY, maxX, maxY,
                processed_elevations[0],
                processed_elevations[width - 1],
                processed_elevations[(height - 1) * width + width - 1],
                processed_elevations[(height - 1) * width]);
            
            // Add subset of sparse points
            for (size_t i = 0; i < points_to_try; ++i) {
                const auto& pt = sparse_points[i];
                triangulation_manager->addPoint(std::get<0>(pt), std::get<1>(pt), std::get<2>(pt));
            }
            
            success = triangulation_manager->retriangulate();
            
            if (!success) {
                points_to_try = points_to_try * 3 / 4; // Reduce by 25%
                std::cout << "SPARSE triangulation failed, trying with " << points_to_try << " points\n";
            }
        }
        
        std::cout << "SPARSE strategy completed with " << triangulation_manager->getPointCount() 
                  << " total points\n";
    } else {
        // HEAP and HYBRID: Use optimized incremental insertion with error-driven selection
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Initialize all candidates from grid
        refiner->initializeCandidatesFromGrid(width, height, processed_elevations);
        
        // Perform incremental refinement
        int points_added = refiner->refineIncrementally(width, height, processed_elevations);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        // Get performance statistics
        size_t updates, calculations;
        refiner->getPerformanceStats(updates, calculations);
        
        std::cout << "Optimized incremental insertion completed in " << duration.count() << "ms" << std::endl;
        std::cout << "Added " << points_added << " points with " << calculations 
                  << " error calculations and " << updates << " candidate updates" << std::endl;
        std::cout << "Performance: " << (calculations > 0 ? static_cast<double>(points_added) / calculations * 100 : 0) 
                  << "% efficiency (points/calculations)" << std::endl;
    }
    
    // Convert to final mesh result
    return triangulation_manager->toMeshResult();
}

// Explicit instantiations for common types
template MeshResult grid_to_mesh_detria<float>(int width, int height, const float* elevations, float error_threshold, int point_limit, MeshRefineStrategy strategy);
template MeshResult grid_to_mesh_detria<double>(int width, int height, const double* elevations, float error_threshold, int point_limit, MeshRefineStrategy strategy);
template MeshResult grid_to_mesh_detria<int>(int width, int height, const int* elevations, float error_threshold, int point_limit, MeshRefineStrategy strategy);

// Explicit instantiations for preprocessing function
template PreprocessingResult preprocess_input_data<float>(int width, int height, const float* elevations, float& error_threshold, bool enable_jitter);
template PreprocessingResult preprocess_input_data<double>(int width, int height, const double* elevations, float& error_threshold, bool enable_jitter);
template PreprocessingResult preprocess_input_data<int>(int width, int height, const int* elevations, float& error_threshold, bool enable_jitter);

// === Volumetric Mesh Generation Implementation ===

std::vector<Edge> find_boundary_edges(const std::vector<Triangle>& triangles) {
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

MeshResult make_volumetric_mesh(const MeshResult& surface_mesh, float z_base) {
    MeshResult volumetric_result;
    volumetric_result.is_volumetric = true;
    
    // Copy surface vertices
    volumetric_result.vertices = surface_mesh.vertices;
    
    // Create base vertices (same x,y but z = z_base)
    std::vector<int> base_vertex_mapping(surface_mesh.vertices.size());
    for (size_t i = 0; i < surface_mesh.vertices.size(); ++i) {
        const Vertex& surface_vertex = surface_mesh.vertices[i];
        Vertex base_vertex = {surface_vertex.x, surface_vertex.y, z_base};
        base_vertex_mapping[i] = static_cast<int>(volumetric_result.vertices.size());
        volumetric_result.vertices.push_back(base_vertex);
    }
    
    // Copy surface triangles (maintain orientation)
    volumetric_result.triangles = surface_mesh.triangles;
    
    // Find boundary edges
    std::vector<Edge> boundary_edges = find_boundary_edges(surface_mesh.triangles);
    
    // Create side faces connecting surface boundary to base boundary
    for (const Edge& edge : boundary_edges) {
        int surface_v0 = edge.v0;
        int surface_v1 = edge.v1;
        int base_v0 = base_vertex_mapping[surface_v0];
        int base_v1 = base_vertex_mapping[surface_v1];
        
        // Create two triangles to form a quad face
        // Triangle 1: surface_v0 -> surface_v1 -> base_v0 (CCW when viewed from outside)
        volumetric_result.triangles.push_back({surface_v0, surface_v1, base_v0});
        
        // Triangle 2: surface_v1 -> base_v1 -> base_v0 (CCW when viewed from outside)
        volumetric_result.triangles.push_back({surface_v1, base_v1, base_v0});
    }
    
    // Create base triangles (reverse orientation compared to surface for inward-facing normals)
    for (const Triangle& surface_tri : surface_mesh.triangles) {
        int base_v0 = base_vertex_mapping[surface_tri.v0];
        int base_v1 = base_vertex_mapping[surface_tri.v1];
        int base_v2 = base_vertex_mapping[surface_tri.v2];
        
        // Reverse winding order for base (so normals point inward)
        volumetric_result.triangles.push_back({base_v0, base_v2, base_v1});
    }
    
    return volumetric_result;
}

VolumetricMeshResult make_volumetric_mesh_separated(const MeshResult& surface_mesh, float z_base) {
    VolumetricMeshResult result;
    
    const float epsilon = 1e-6f;  // Tolerance for height comparison
    
    // Separate triangles based on their centroid height relative to z_base
    std::vector<Triangle> positive_triangles, negative_triangles;
    
    for (const Triangle& tri : surface_mesh.triangles) {
        // Calculate triangle centroid
        const Vertex& v0 = surface_mesh.vertices[tri.v0];
        const Vertex& v1 = surface_mesh.vertices[tri.v1];
        const Vertex& v2 = surface_mesh.vertices[tri.v2];
        
        float centroid_z = (v0.z + v1.z + v2.z) / 3.0f;
        float height_diff = centroid_z - z_base;
        
        if (height_diff > epsilon) {
            positive_triangles.push_back(tri);
        } else if (height_diff < -epsilon) {
            negative_triangles.push_back(tri);
        }
        // Skip triangles at exactly the base level (height_diff within epsilon) to avoid degeneracies
    }
    
    // Create positive volume mesh if we have positive triangles
    if (!positive_triangles.empty()) {
        // Create a surface mesh with only positive triangles and their vertices
        MeshResult positive_surface;
        
        // Find all vertices used by positive triangles
        std::set<int> positive_vertex_indices;
        for (const Triangle& tri : positive_triangles) {
            positive_vertex_indices.insert(tri.v0);
            positive_vertex_indices.insert(tri.v1);
            positive_vertex_indices.insert(tri.v2);
        }
        
        // Create vertex mapping from old indices to new indices
        std::map<int, int> vertex_mapping;
        for (int old_idx : positive_vertex_indices) {
            int new_idx = static_cast<int>(positive_surface.vertices.size());
            vertex_mapping[old_idx] = new_idx;
            positive_surface.vertices.push_back(surface_mesh.vertices[old_idx]);
        }
        
        // Remap triangles to use new vertex indices
        for (const Triangle& tri : positive_triangles) {
            positive_surface.triangles.push_back({
                vertex_mapping[tri.v0],
                vertex_mapping[tri.v1],
                vertex_mapping[tri.v2]
            });
        }
        
        // Generate positive volumetric mesh using existing logic
        result.positive_volume = make_volumetric_mesh(positive_surface, z_base);
        result.has_positive_volume = true;
    }
    
    // Create negative volume mesh if we have negative triangles
    if (!negative_triangles.empty()) {
        // Create a surface mesh with only negative triangles and their vertices
        MeshResult negative_surface;
        
        // Find all vertices used by negative triangles
        std::set<int> negative_vertex_indices;
        for (const Triangle& tri : negative_triangles) {
            negative_vertex_indices.insert(tri.v0);
            negative_vertex_indices.insert(tri.v1);
            negative_vertex_indices.insert(tri.v2);
        }
        
        // Create vertex mapping from old indices to new indices
        std::map<int, int> vertex_mapping;
        for (int old_idx : negative_vertex_indices) {
            int new_idx = static_cast<int>(negative_surface.vertices.size());
            vertex_mapping[old_idx] = new_idx;
            negative_surface.vertices.push_back(surface_mesh.vertices[old_idx]);
        }
        
        // Remap triangles to use new vertex indices
        for (const Triangle& tri : negative_triangles) {
            negative_surface.triangles.push_back({
                vertex_mapping[tri.v0],
                vertex_mapping[tri.v1],
                vertex_mapping[tri.v2]
            });
        }
        
        // Generate negative volumetric mesh with reversed normals
        result.negative_volume.is_volumetric = true;
        
        // Copy surface vertices
        result.negative_volume.vertices = negative_surface.vertices;
        
        // Create base vertices at z_base
        std::vector<int> base_vertex_mapping(negative_surface.vertices.size());
        for (size_t i = 0; i < negative_surface.vertices.size(); ++i) {
            const Vertex& surface_vertex = negative_surface.vertices[i];
            Vertex base_vertex = {surface_vertex.x, surface_vertex.y, z_base};
            base_vertex_mapping[i] = static_cast<int>(result.negative_volume.vertices.size());
            result.negative_volume.vertices.push_back(base_vertex);
        }
        
        // Add surface triangles with reversed winding (since we want inverted normals for negative volume)
        for (const Triangle& surface_tri : negative_surface.triangles) {
            result.negative_volume.triangles.push_back({surface_tri.v0, surface_tri.v2, surface_tri.v1});
        }
        
        // Find boundary edges
        std::vector<Edge> boundary_edges = find_boundary_edges(negative_surface.triangles);
        
        // Create side faces connecting surface boundary to base boundary (reversed winding)
        for (const Edge& edge : boundary_edges) {
            int surface_v0 = edge.v0;
            int surface_v1 = edge.v1;
            int base_v0 = base_vertex_mapping[surface_v0];
            int base_v1 = base_vertex_mapping[surface_v1];
            
            // Reverse winding for negative volume
            result.negative_volume.triangles.push_back({surface_v0, base_v0, surface_v1});
            result.negative_volume.triangles.push_back({surface_v1, base_v0, base_v1});
        }
        
        // Create base triangles with same orientation as surface (since surface is already reversed)
        for (const Triangle& surface_tri : negative_surface.triangles) {
            int base_v0 = base_vertex_mapping[surface_tri.v0];
            int base_v1 = base_vertex_mapping[surface_tri.v1];
            int base_v2 = base_vertex_mapping[surface_tri.v2];
            
            result.negative_volume.triangles.push_back({base_v0, base_v1, base_v2});
        }
        
        result.has_negative_volume = true;
    }
    
    return result;
}

template<typename T>
MeshResult grid_to_mesh_volumetric(
    int width, int height, const T* elevations,
    float z_base,
    float error_threshold, int point_limit,
    MeshRefineStrategy strategy)
{
    // First generate the surface mesh
    MeshResult surface_mesh = grid_to_mesh_detria(width, height, elevations, 
                                                  error_threshold, point_limit, strategy);
    
    // Convert to volumetric mesh
    return make_volumetric_mesh(surface_mesh, z_base);
}

template<typename T>
VolumetricMeshResult grid_to_mesh_volumetric_separated(
    int width, int height, const T* elevations,
    float z_base,
    float error_threshold, int point_limit,
    MeshRefineStrategy strategy)
{
    // First generate the surface mesh
    MeshResult surface_mesh = grid_to_mesh_detria(width, height, elevations, 
                                                  error_threshold, point_limit, strategy);
    
    // Convert to separated volumetric meshes
    return make_volumetric_mesh_separated(surface_mesh, z_base);
}

// Explicit instantiations for volumetric mesh generation
template MeshResult grid_to_mesh_volumetric<float>(int width, int height, const float* elevations, float z_base, float error_threshold, int point_limit, MeshRefineStrategy strategy);
template MeshResult grid_to_mesh_volumetric<double>(int width, int height, const double* elevations, float z_base, float error_threshold, int point_limit, MeshRefineStrategy strategy);
template MeshResult grid_to_mesh_volumetric<int>(int width, int height, const int* elevations, float z_base, float error_threshold, int point_limit, MeshRefineStrategy strategy);

// Explicit instantiations for separated volumetric mesh generation
template VolumetricMeshResult grid_to_mesh_volumetric_separated<float>(int width, int height, const float* elevations, float z_base, float error_threshold, int point_limit, MeshRefineStrategy strategy);
template VolumetricMeshResult grid_to_mesh_volumetric_separated<double>(int width, int height, const double* elevations, float z_base, float error_threshold, int point_limit, MeshRefineStrategy strategy);
template VolumetricMeshResult grid_to_mesh_volumetric_separated<int>(int width, int height, const int* elevations, float z_base, float error_threshold, int point_limit, MeshRefineStrategy strategy);

} // namespace TerraScape

#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic pop /* end ignoring warnings */
#elif defined(__clang__)
#  pragma clang diagnostic pop /* end ignoring warnings */
#endif

