#pragma once

#include <vector>
#include <memory>
#include <unordered_set>
#include <queue>
#include <map>

namespace TerraScape {

/**
 * Vertex structure for grid-aware triangulation
 */
struct GridVertex {
    float x, y, z;
    int grid_x, grid_y;  // Original grid coordinates
    uint32_t index;      // Vertex index in final mesh
    
    GridVertex(float x, float y, float z, int gx = -1, int gy = -1, uint32_t idx = 0)
        : x(x), y(y), z(z), grid_x(gx), grid_y(gy), index(idx) {}
};

/**
 * Edge structure for advancing front algorithm
 */
struct AdvancingEdge {
    uint32_t v0, v1;           // Vertex indices
    bool is_boundary;          // Whether this is a boundary edge
    float quality_metric;      // Used for prioritizing edge advancement
    
    AdvancingEdge(uint32_t v0, uint32_t v1, bool boundary = false)
        : v0(v0), v1(v1), is_boundary(boundary), quality_metric(0.0f) {}
        
    bool operator<(const AdvancingEdge& other) const {
        return quality_metric < other.quality_metric; // Min-heap (advance best edges first)
    }
};

/**
 * Triangle structure
 */
struct GridTriangle {
    uint32_t v0, v1, v2;
    float quality;  // Aspect ratio or other quality metric
    
    GridTriangle(uint32_t v0, uint32_t v1, uint32_t v2) : v0(v0), v1(v1), v2(v2), quality(0.0f) {}
};

/**
 * Forward declaration for MeshResult
 */
struct MeshResult;

/**
 * Grid-aware triangulation manager that replaces detria for gridded terrain data.
 * Uses advancing front algorithm that naturally handles collinear points and 
 * takes advantage of the structured nature of grid data.
 */
class GridTriangulator {
public:
    GridTriangulator();
    ~GridTriangulator();
    
    /**
     * Initialize triangulator with grid dimensions and boundary
     */
    void initializeGrid(int width, int height, float min_x, float min_y, float max_x, float max_y);
    
    /**
     * Add a grid point at specific grid coordinates
     */
    uint32_t addGridPoint(int grid_x, int grid_y, float z);
    
    /**
     * Add arbitrary point (for sparse sampling)
     */
    uint32_t addPoint(float x, float y, float z);
    
    /**
     * Execute advancing front triangulation
     */
    bool triangulate();
    
    /**
     * Optimize triangle quality via edge swapping and other techniques
     */
    void optimizeTriangleQuality();
    
    /**
     * Convert to standard MeshResult format
     */
    MeshResult toMeshResult() const;
    
    /**
     * Get statistics
     */
    size_t getVertexCount() const { return vertices_.size(); }
    size_t getTriangleCount() const { return triangles_.size(); }
    
    /**
     * Clear all data
     */
    void clear();

private:
    // Grid parameters
    int grid_width_, grid_height_;
    float min_x_, min_y_, max_x_, max_y_;
    
    // Mesh data
    std::vector<GridVertex> vertices_;
    std::vector<GridTriangle> triangles_;
    
    // Grid mapping for efficient lookups
    std::vector<uint32_t> grid_to_vertex_;  // grid_y * width + grid_x -> vertex index
    
    // Advancing front data structures
    std::priority_queue<AdvancingEdge> advancing_front_;
    std::unordered_set<uint64_t> processed_edges_;  // Track processed edges to avoid duplicates
    
    // Helper methods
    void initializeBoundaryTriangulation();
    bool advanceFrontStep();
    uint32_t findBestInteriorPoint(const AdvancingEdge& edge);
    float calculateTriangleQuality(uint32_t v0, uint32_t v1, uint32_t v2) const;
    float calculateEdgeQuality(const AdvancingEdge& edge) const;
    void addTriangle(uint32_t v0, uint32_t v1, uint32_t v2);
    void updateAdvancingFront(uint32_t v0, uint32_t v1, uint32_t v2);
    uint64_t encodeEdge(uint32_t v0, uint32_t v1) const;
    bool isEdgeProcessed(uint32_t v0, uint32_t v1) const;
    void markEdgeProcessed(uint32_t v0, uint32_t v1);
    
    // Grid utilities
    bool isValidGridCoordinate(int x, int y) const;
    float gridToWorldX(int grid_x) const;
    float gridToWorldY(int grid_y) const;
    std::pair<int, int> worldToGrid(float x, float y) const;
    
    // Geometry utilities
    float distanceSquared(const GridVertex& a, const GridVertex& b) const;
    float triangleArea(const GridVertex& a, const GridVertex& b, const GridVertex& c) const;
    bool isLeftTurn(const GridVertex& a, const GridVertex& b, const GridVertex& c) const;
    
    // Triangle quality optimization
    bool canSwapEdge(uint32_t tri1, uint32_t tri2, uint32_t shared_v0, uint32_t shared_v1);
    void swapEdge(uint32_t tri1, uint32_t tri2, uint32_t shared_v0, uint32_t shared_v1);
    std::vector<uint32_t> findTrianglesUsingEdge(uint32_t v0, uint32_t v1);
};

} // namespace TerraScape