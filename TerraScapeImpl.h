#pragma once

#include "bg_detria.hpp"
#include <vector>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace TerraScape {

// Forward declarations for compatibility with bg_grid_mesh.h
struct Vertex;
struct Triangle;
struct MeshResult;

/**
 * Triangulation manager that wraps bg_detria.hpp for mesh generation
 * Provides proper Delaunay triangulation with topology management
 */
class DetriaTriangulationManager {
public:
    using TriangulationT = detria::Triangulation<detria::PointF, uint32_t>;
    using PointT = detria::PointF;
    
private:
    std::unique_ptr<TriangulationT> triangulation_;
    std::vector<PointT> points_;
    std::vector<uint32_t> boundary_indices_;
    bool is_triangulated_;
    uint32_t triangulation_version_;
    
    // Spatial acceleration for fast triangle lookup
    struct SpatialAccel {
        float minX, minY, maxX, maxY;
        int gridWidth, gridHeight;
        std::vector<std::vector<std::vector<uint32_t>>> grid; // grid[y][x] = list of triangle indices
        bool is_built;
        
        SpatialAccel() : is_built(false) {}
    } spatial_accel_;
    
public:
    DetriaTriangulationManager();
    ~DetriaTriangulationManager();
    
    /**
     * Initialize with boundary rectangle (typically grid corners)
     */
    void initializeBoundary(float minX, float minY, float maxX, float maxY, 
                           float z00, float z10, float z11, float z01);
    
    /**
     * Add a new point to the triangulation
     * Returns the index of the added point
     */
    uint32_t addPoint(float x, float y, float z);
    
    /**
     * Get current triangulation version (incremented after each successful retriangulation)
     */
    uint32_t getVersion() const { return triangulation_version_; }
    size_t getPointCount() const { return points_.size(); }
    
    /**
     * Get point by index
     */
    const PointT& getPoint(uint32_t index) const { return points_[index]; }
    
    /**
     * Get Z coordinate for a point (stored separately from detria points)
     */
    float getPointZ(uint32_t index) const;
    
    /**
     * Retriangulate after adding points
     */
    bool retriangulate();
    
    /**
     * Find triangle containing a point (x, y)
     * Returns triangle vertex indices, or empty vector if not found
     */
    std::vector<uint32_t> findContainingTriangle(float x, float y) const;
    
    /**
     * Build spatial acceleration structure for fast triangle lookup
     */
    void buildSpatialAcceleration();
    
    /**
     * Get all triangles as vertex index triplets
     */
    std::vector<std::vector<uint32_t>> getAllTriangles() const;
    
    /**
     * Convert to bg::MeshResult format
     */
    MeshResult toMeshResult() const;
    
private:
    std::vector<float> point_z_coords_; // Z coordinates stored separately
};

/**
 * Greedy refinement layer that handles error-driven candidate selection
 * and TRUE incremental insertion on top of the triangulation manager
 */
class GreedyMeshRefiner {
public:
    struct CandidatePoint {
        int x, y;              // Grid coordinates
        float world_x, world_y; // World coordinates  
        float error;           // Approximation error
        std::vector<uint32_t> containing_triangle; // Triangle containing this point
        bool needs_update;     // Whether error needs recalculation
        uint32_t tri_version;  // Triangulation version when this candidate was computed
        
        bool operator<(const CandidatePoint& other) const {
            return error < other.error; // For max-heap
        }
    };
    
private:
    DetriaTriangulationManager* triangulation_manager_;
    std::priority_queue<CandidatePoint> candidate_heap_;
    float error_threshold_;
    int point_limit_;
    int batch_size_;  // Number of points to add before retriangulation
    
    // For true incremental insertion: track candidates by grid position
    std::unordered_map<int, CandidatePoint> grid_candidates_; // key = y*width + x
    std::unordered_set<int> inserted_keys_; // Guard against duplicate point insertion
    int grid_width_, grid_height_;
    
public:
    GreedyMeshRefiner(DetriaTriangulationManager* manager, 
                     float error_threshold, int point_limit, int batch_size = 32);
    
    /**
     * Initialize candidates from grid (but don't add all to heap immediately)
     */
    template<typename T>
    void initializeCandidatesFromGrid(int width, int height, const T* elevations);
    
    /**
     * Perform TRUE incremental greedy refinement:
     * - Find best candidate
     * - Insert single point
     * - Update only affected candidates
     * - Repeat until done
     */
    template<typename T>
    int refineIncrementally(int width, int height, const T* elevations);
    
    /**
     * Update candidate errors for triangles affected by the last point insertion
     */
    template<typename T>
    void updateAffectedCandidates(int width, int height, const T* elevations, 
                                 float inserted_x, float inserted_y);
    
    /**
     * Get the current best candidate without removing it
     */
    CandidatePoint getBestCandidate() const;
    
    /**
     * Check if there are viable candidates remaining
     */
    bool hasViableCandidates() const;
    
private:
    template<typename T>
    float calculateError(int gx, int gy, const T* elevations, int width, int height,
                        const std::vector<uint32_t>& triangle) const;
    
    template<typename T>
    CandidatePoint createCandidateFromGrid(int x, int y, int width, int height, 
                                          const T* elevations);
    
    float barycentricInterpolation(float px, float py, 
                                  const std::vector<uint32_t>& triangle) const;
    
    /**
     * Find all grid candidates that need error updates due to triangulation changes
     */
    std::vector<std::pair<int, int>> findAffectedGridPoints(float inserted_x, float inserted_y, 
                                                           float search_radius) const;
};

} // namespace TerraScape