#include "TerraScapeImpl.h"
#include "TerraScape.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <unordered_map>

namespace TerraScape {

// Helper function for point-in-triangle test (barycentric coordinates)
static bool pointInTriangle(float px, float py, 
                           const detria::PointF& v0, const detria::PointF& v1, const detria::PointF& v2) {
    float dX = px - v2.x;
    float dY = py - v2.y;
    float dX21 = v2.x - v1.x;
    float dY12 = v1.y - v2.y;
    float D = (v0.x - v2.x) * dY12 + (v0.y - v2.y) * dX21;
    
    if (std::abs(D) < 1e-10f) return false; // Degenerate triangle
    
    float s = ((v0.x - v2.x) * dY + (v0.y - v2.y) * dX) / D;
    float t = (dY12 * dX + dX21 * dY) / D;
    return (s >= -1e-6f) && (t >= -1e-6f) && (s + t <= 1.0f + 1e-6f);
}

static float barycentricInterp(float px, float py,
                              const detria::PointF& v0, const detria::PointF& v1, const detria::PointF& v2,
                              float z0, float z1, float z2) {
    float denom = ((v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y));
    if (std::abs(denom) < 1e-10f) return z0; // Fallback for degenerate triangles
    
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
    
    // Convert triangles
    auto triangles = getAllTriangles();
    for (const auto& tri : triangles) {
        if (tri.size() == 3) {
            result.triangles.push_back({static_cast<int>(tri[0]), 
                                       static_cast<int>(tri[1]), 
                                       static_cast<int>(tri[2])});
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
    , grid_height_(0) {
}

template<typename T>
void GreedyMeshRefiner::initializeCandidatesFromGrid(int width, int height, const T* elevations) {
    grid_width_ = width;
    grid_height_ = height;
    
    // Clear existing data
    grid_candidates_.clear();
    candidate_heap_ = std::priority_queue<CandidatePoint>();
    
    // Ensure triangulation is up to date
    if (!triangulation_manager_->retriangulate()) {
        std::cerr << "Warning: Triangulation failed, cannot initialize candidates\n";
        return;
    }
    
    // Initialize all grid candidates but only add viable ones to heap
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Skip corner points (already in triangulation)
            bool is_corner = (x == 0 || x == width - 1) && (y == 0 || y == height - 1);
            if (is_corner) continue;
            
            CandidatePoint candidate = createCandidateFromGrid(x, y, width, height, elevations);
            int key = y * width + x;
            grid_candidates_[key] = candidate;
            
            // Add to heap if error is significant
            if (candidate.error > error_threshold_) {
                candidate_heap_.push(candidate);
            }
        }
    }
    
    std::cout << "Initialized " << grid_candidates_.size() << " grid candidates, " 
              << candidate_heap_.size() << " in active heap\n";
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
            uint32_t new_index = triangulation_manager_->addPoint(candidate.world_x, candidate.world_y, actual_z);
            
            // Mark this point as inserted to prevent duplicates
            int key = candidate.y * width + candidate.x;
            inserted_keys_.insert(key);
            
            // Remove this candidate from our tracking
            grid_candidates_.erase(key);
        }
        
        // Retriangulate once for the entire batch
        if (!triangulation_manager_->retriangulate()) {
            std::cerr << "Warning: Retriangulation failed after adding batch of " << batch_candidates.size() << " points\n";
            break;
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
    // Find grid points that might be affected by this insertion
    // Use a reasonable search radius based on typical triangle sizes
    float search_radius = std::max(width, height) / 20.0f;
    auto affected_points = findAffectedGridPoints(inserted_x, inserted_y, search_radius);
    
    int updated_count = 0;
    for (const auto& grid_point : affected_points) {
        int x = grid_point.first;
        int y = grid_point.second;
        int key = y * width + x;
        
        auto it = grid_candidates_.find(key);
        if (it != grid_candidates_.end()) {
            // Recalculate this candidate's error
            CandidatePoint updated = createCandidateFromGrid(x, y, width, height, elevations);
            it->second = updated;
            it->second.needs_update = false;
            
            // Add to heap if error is still significant
            if (updated.error > error_threshold_) {
                candidate_heap_.push(updated);
            }
            updated_count++;
        }
    }
    
    std::cout << "Updated " << updated_count << " affected candidates\n";
}

template<typename T>
GreedyMeshRefiner::CandidatePoint GreedyMeshRefiner::createCandidateFromGrid(int x, int y, int width, int height, 
                                                         const T* elevations) {
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
float GreedyMeshRefiner::calculateError(int gx, int gy, const T* elevations, int width, int height,
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

// --- Main grid_to_mesh_detria implementation ---
template<typename T>
MeshResult grid_to_mesh_detria(
    int width, int height, const T* elevations,
    float error_threshold, int point_limit,
    MeshRefineStrategy strategy)
{
    // Create triangulation manager and initialize with boundary
    auto triangulation_manager = std::make_unique<DetriaTriangulationManager>();
    
    // Initialize boundary corners
    float minX = 0.0f, minY = 0.0f;
    float maxX = static_cast<float>(width - 1);
    float maxY = static_cast<float>(height - 1);
    
    triangulation_manager->initializeBoundary(
        minX, minY, maxX, maxY,
        static_cast<float>(elevations[0]),                                    // z00: bottom-left
        static_cast<float>(elevations[width - 1]),                           // z10: bottom-right
        static_cast<float>(elevations[(height - 1) * width + width - 1]),    // z11: top-right
        static_cast<float>(elevations[(height - 1) * width])                 // z01: top-left
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
                bool is_corner = (x == 0 || x == width - 1) && (y == 0 || y == height - 1);
                if (!is_corner && sparse_points.size() < static_cast<size_t>(point_limit - 4)) {
                    float px = static_cast<float>(x);
                    float py = static_cast<float>(y);
                    float pz = static_cast<float>(elevations[y * width + x]);
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
                static_cast<float>(elevations[0]),
                static_cast<float>(elevations[width - 1]),
                static_cast<float>(elevations[(height - 1) * width + width - 1]),
                static_cast<float>(elevations[(height - 1) * width]));
            
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
        // HEAP and HYBRID: Use TRUE incremental insertion with error-driven selection
        std::cout << "Starting TRUE incremental point insertion for error-driven strategy\n";
        
        // Initialize all candidates from grid
        refiner->initializeCandidatesFromGrid(width, height, elevations);
        
        // Perform incremental refinement
        int points_added = refiner->refineIncrementally(width, height, elevations);
        
        std::cout << "TRUE incremental insertion completed. Added " << points_added 
                  << " points incrementally.\n";
    }
    
    // Convert to final mesh result
    return triangulation_manager->toMeshResult();
}

// Explicit instantiations for common types
template MeshResult grid_to_mesh_detria<float>(int width, int height, const float* elevations, float error_threshold, int point_limit, MeshRefineStrategy strategy);
template MeshResult grid_to_mesh_detria<double>(int width, int height, const double* elevations, float error_threshold, int point_limit, MeshRefineStrategy strategy);
template MeshResult grid_to_mesh_detria<int>(int width, int height, const int* elevations, float error_threshold, int point_limit, MeshRefineStrategy strategy);

} // namespace TerraScape