#include "bg_detria_mesh.h"
#include "bg_grid_mesh.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <unordered_map>

namespace bg {

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
    , is_triangulated_(false) {
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
    
    // Reset triangulation with all points
    triangulation_->setPoints(points_);
    triangulation_->addOutline(boundary_indices_);
    
    is_triangulated_ = triangulation_->triangulate(true); // Delaunay
    
    if (!is_triangulated_) {
        std::cerr << "Triangulation failed with " << points_.size() << " points. Error: " 
                  << triangulation_->getErrorMessage() << "\n";
    }
    
    return is_triangulated_;
}

std::vector<uint32_t> DetriaTriangulationManager::findContainingTriangle(float x, float y) const {
    if (!is_triangulated_) return {};
    
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
                                   float error_threshold, int point_limit)
    : triangulation_manager_(manager)
    , error_threshold_(error_threshold)
    , point_limit_(point_limit)
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
    
    while (hasViableCandidates() && 
           triangulation_manager_->getPointCount() < static_cast<size_t>(point_limit_)) {
        
        // Get the best candidate
        CandidatePoint best = candidate_heap_.top();
        candidate_heap_.pop();
        
        if (best.error < error_threshold_) {
            break; // All remaining candidates have acceptable error
        }
        
        // Verify this candidate is still valid (hasn't been invalidated by previous insertions)
        int key = best.y * width + best.x;
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
        
        // Add the point with actual elevation data
        float actual_z = static_cast<float>(elevations[best.y * width + best.x]);
        uint32_t new_index = triangulation_manager_->addPoint(best.world_x, best.world_y, actual_z);
        
        // Retriangulate
        if (!triangulation_manager_->retriangulate()) {
            std::cerr << "Warning: Retriangulation failed after adding point\n";
            break;
        }
        
        points_added++;
        
        // Remove this candidate from our tracking
        grid_candidates_.erase(key);
        
        // Update affected candidates (TRUE incremental approach)
        updateAffectedCandidates(width, height, elevations, best.world_x, best.world_y);
        
        if (points_added % 10 == 0) {
            std::cout << "Added " << points_added << " points incrementally\n";
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

} // namespace bg