#pragma once

#include "bg_detria.hpp"
#include <vector>
#include <memory>
#include <queue>

namespace bg {

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
     * Get current point count
     */
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
 * and insertion on top of the triangulation manager
 */
class GreedyMeshRefiner {
public:
    struct CandidatePoint {
        int x, y;              // Grid coordinates
        float world_x, world_y; // World coordinates  
        float error;           // Approximation error
        std::vector<uint32_t> containing_triangle; // Triangle containing this point
        
        bool operator<(const CandidatePoint& other) const {
            return error < other.error; // For max-heap
        }
    };
    
private:
    DetriaTriangulationManager* triangulation_manager_;
    std::priority_queue<CandidatePoint> candidate_heap_;
    float error_threshold_;
    int point_limit_;
    
public:
    GreedyMeshRefiner(DetriaTriangulationManager* manager, 
                     float error_threshold, int point_limit);
    
    /**
     * Add candidate points from grid
     */
    template<typename T>
    void addCandidatesFromGrid(int width, int height, const T* elevations);
    
    /**
     * Perform greedy refinement until error threshold or point limit reached
     */
    template<typename T>
    int refineGreedy(int width, int height, const T* elevations);
    
    /**
     * Update candidate errors after triangulation changes
     */
    void updateCandidateErrors();
    
private:
    template<typename T>
    float calculateError(int gx, int gy, const T* elevations, int width, int height,
                        const std::vector<uint32_t>& triangle) const;
    
    template<typename T>
    float bilinearInterpolation(float x, float y, const T* elevations, int width, int height) const;
    
    float barycentricInterpolation(float px, float py, 
                                  const std::vector<uint32_t>& triangle) const;
};

} // namespace bg