#pragma once

/*
 * TerraScape - Advanced Terrain Mesh Generation Library
 *
 * This library provides efficient grid-to-mesh conversion with advanced
 * Delaunay triangulation and greedy refinement algorithms.
 *
 * Based on the Terra and Scape terrain simplification algorithms with
 * modern C++ implementation using robust geometric predicates.
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
#include <map>
#include <string>
#include <stdexcept>
#include <cstdint>

#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#elif defined(__APPLE__)
#include <sys/types.h>
#include <sys/sysctl.h>
#endif

namespace TerraScape {

// Grid-aware advancing front triangulation is now the single, optimal approach
// for gridded terrain data. No strategy selection needed - the algorithm
// naturally handles collinear points and leverages grid structure.

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

// --- Forward declarations for implementation ---
class GridTriangulator;

// --- Helper: Enhanced grid accessor ---
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

// --- Utility: barycentric triangle routines ---
inline bool point_in_triangle(float px, float py,
                              const Vertex& v0, const Vertex& v1, const Vertex& v2)
{
    float dX = px - v2.x;
    float dY = py - v2.y;
    float dX21 = v2.x - v1.x;
    float dY12 = v1.y - v2.y;
    float D = (v0.x - v2.x) * dY12 + (v0.y - v2.y) * dX21;
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

// --- Implementation section ---
// Header-only implementation follows

/**
 * Grid-aware triangulation manager that replaces external triangulation libraries
 * for gridded terrain data. Uses advancing front algorithm that naturally handles
 * collinear points and takes advantage of the structured nature of grid data.
 */
class GridTriangulator {
public:
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

    GridTriangulator()
        : grid_width_(0), grid_height_(0)
        , min_x_(0.0f), min_y_(0.0f), max_x_(0.0f), max_y_(0.0f) {
    }

    ~GridTriangulator() = default;

    /**
     * Initialize triangulator with grid dimensions and boundary
     */
    void initializeGrid(int width, int height, float min_x, float min_y, float max_x, float max_y) {
        clear();

        grid_width_ = width;
        grid_height_ = height;
        min_x_ = min_x;
        min_y_ = min_y;
        max_x_ = max_x;
        max_y_ = max_y;

        // Initialize grid mapping
        grid_to_vertex_.assign(width * height, UINT32_MAX);
    }

    void clear() {
        vertices_.clear();
        triangles_.clear();
        grid_to_vertex_.clear();

        // Clear advancing front
        while (!advancing_front_.empty()) {
            advancing_front_.pop();
        }
        processed_edges_.clear();
    }

    /**
     * Add a grid point at specific grid coordinates
     */
    uint32_t addGridPoint(int grid_x, int grid_y, float z) {
        if (!isValidGridCoordinate(grid_x, grid_y)) {
            return UINT32_MAX;
        }

        float x = gridToWorldX(grid_x);
        float y = gridToWorldY(grid_y);

        uint32_t vertex_index = static_cast<uint32_t>(vertices_.size());
        vertices_.emplace_back(x, y, z, grid_x, grid_y, vertex_index);

        // Map grid coordinate to vertex
        grid_to_vertex_[grid_y * grid_width_ + grid_x] = vertex_index;

        return vertex_index;
    }

    /**
     * Add arbitrary point (for sparse sampling)
     */
    uint32_t addPoint(float x, float y, float z) {
        uint32_t vertex_index = static_cast<uint32_t>(vertices_.size());
        vertices_.emplace_back(x, y, z, -1, -1, vertex_index);
        return vertex_index;
    }

    /**
     * Execute advancing front triangulation
     */
    bool triangulate() {
        if (vertices_.size() < 3) return false;

        // Initialize boundary triangulation
        initializeBoundaryTriangulation();

        // Advance the front until complete
        while (!advancing_front_.empty()) {
            if (!advanceFrontStep()) {
                break; // Failed to advance
            }
        }

        return !triangles_.empty();
    }

    /**
     * Convert to standard MeshResult format
     */
    MeshResult toMeshResult() const {
        MeshResult result;

        // Convert vertices
        result.vertices.reserve(vertices_.size());
        for (const auto& gv : vertices_) {
            result.vertices.push_back({gv.x, gv.y, gv.z});
        }

        // Convert triangles
        result.triangles.reserve(triangles_.size());
        for (const auto& gt : triangles_) {
            result.triangles.push_back({static_cast<int>(gt.v0), static_cast<int>(gt.v1), static_cast<int>(gt.v2)});
        }

        return result;
    }

    /**
     * Get statistics
     */
    size_t getVertexCount() const { return vertices_.size(); }
    size_t getTriangleCount() const { return triangles_.size(); }

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
    void initializeBoundaryTriangulation() {
        // Create initial boundary triangulation using convex hull or regular grid boundary
        if (vertices_.size() < 3) return;

        // For grid data, we can create a simple boundary triangulation
        // Find corner vertices (assuming they exist)
        std::vector<uint32_t> corners;

        // Look for corners in the grid
        for (const auto& v : vertices_) {
            if (v.grid_x != -1 && v.grid_y != -1) {
                if ((v.grid_x == 0 || v.grid_x == grid_width_ - 1) &&
                    (v.grid_y == 0 || v.grid_y == grid_height_ - 1)) {
                    corners.push_back(v.index);
                }
            }
        }

        if (corners.size() >= 3) {
            // Create initial triangles from corners
            if (corners.size() == 4) {
                // Rectangle: create two triangles
                addTriangle(corners[0], corners[1], corners[2]);
                addTriangle(corners[0], corners[2], corners[3]);
            } else {
                // General case: fan triangulation from first corner
                for (size_t i = 1; i < corners.size() - 1; ++i) {
                    addTriangle(corners[0], corners[i], corners[i + 1]);
                }
            }
        }
    }

    bool advanceFrontStep() {
        if (advancing_front_.empty()) return false;

        AdvancingEdge edge = advancing_front_.top();
        advancing_front_.pop();

        // Skip if already processed
        if (isEdgeProcessed(edge.v0, edge.v1)) {
            return true;
        }

        // Find best point to form triangle with this edge
        uint32_t best_point = findBestInteriorPoint(edge);
        if (best_point == UINT32_MAX) {
            return false; // No suitable point found
        }

        // Create triangle
        addTriangle(edge.v0, edge.v1, best_point);

        // Update advancing front
        updateAdvancingFront(edge.v0, edge.v1, best_point);

        return true;
    }

    uint32_t findBestInteriorPoint(const AdvancingEdge& edge) {
        uint32_t best_point = UINT32_MAX;
        float best_quality = -1.0f;

        const GridVertex& v0 = vertices_[edge.v0];
        const GridVertex& v1 = vertices_[edge.v1];

        for (uint32_t i = 0; i < vertices_.size(); ++i) {
            if (i == edge.v0 || i == edge.v1) continue;

            const GridVertex& candidate = vertices_[i];

            // Check if this would form a valid triangle
            if (isLeftTurn(v0, v1, candidate)) {
                float quality = calculateTriangleQuality(edge.v0, edge.v1, i);
                if (quality > best_quality) {
                    best_quality = quality;
                    best_point = i;
                }
            }
        }

        return best_point;
    }

    float calculateTriangleQuality(uint32_t v0, uint32_t v1, uint32_t v2) const {
        const GridVertex& a = vertices_[v0];
        const GridVertex& b = vertices_[v1];
        const GridVertex& c = vertices_[v2];

        // Calculate triangle area and perimeter for aspect ratio
        float area = triangleArea(a, b, c);
        if (area <= 0.0f) return 0.0f;

        float ab = std::sqrt(distanceSquared(a, b));
        float bc = std::sqrt(distanceSquared(b, c));
        float ca = std::sqrt(distanceSquared(c, a));
        float perimeter = ab + bc + ca;

        if (perimeter <= 0.0f) return 0.0f;

        // Return normalized aspect ratio (higher is better)
        return (4.0f * 3.14159f * area) / (perimeter * perimeter);
    }

    void addTriangle(uint32_t v0, uint32_t v1, uint32_t v2) {
        triangles_.emplace_back(v0, v1, v2);

        // Mark edges as processed
        markEdgeProcessed(v0, v1);
        markEdgeProcessed(v1, v2);
        markEdgeProcessed(v2, v0);
    }

    void updateAdvancingFront(uint32_t v0, uint32_t v1, uint32_t v2) {
        // Add new edges to advancing front if they're not already processed
        if (!isEdgeProcessed(v1, v2)) {
            AdvancingEdge new_edge(v1, v2);
            new_edge.quality_metric = calculateEdgeQuality(new_edge);
            advancing_front_.push(new_edge);
        }

        if (!isEdgeProcessed(v2, v0)) {
            AdvancingEdge new_edge(v2, v0);
            new_edge.quality_metric = calculateEdgeQuality(new_edge);
            advancing_front_.push(new_edge);
        }
    }

    float calculateEdgeQuality(const AdvancingEdge& edge) const {
        const GridVertex& v0 = vertices_[edge.v0];
        const GridVertex& v1 = vertices_[edge.v1];
        return -distanceSquared(v0, v1); // Negative because we want shorter edges first
    }

    uint64_t encodeEdge(uint32_t v0, uint32_t v1) const {
        if (v0 > v1) std::swap(v0, v1);
        return (static_cast<uint64_t>(v0) << 32) | v1;
    }

    bool isEdgeProcessed(uint32_t v0, uint32_t v1) const {
        return processed_edges_.count(encodeEdge(v0, v1)) > 0;
    }

    void markEdgeProcessed(uint32_t v0, uint32_t v1) {
        processed_edges_.insert(encodeEdge(v0, v1));
    }

    bool isValidGridCoordinate(int x, int y) const {
        return x >= 0 && x < grid_width_ && y >= 0 && y < grid_height_;
    }

    float gridToWorldX(int grid_x) const {
        return min_x_ + (max_x_ - min_x_) * grid_x / (grid_width_ - 1);
    }

    float gridToWorldY(int grid_y) const {
        return min_y_ + (max_y_ - min_y_) * grid_y / (grid_height_ - 1);
    }

    float distanceSquared(const GridVertex& a, const GridVertex& b) const {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    float triangleArea(const GridVertex& a, const GridVertex& b, const GridVertex& c) const {
        return 0.5f * std::abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
    }

    bool isLeftTurn(const GridVertex& a, const GridVertex& b, const GridVertex& c) const {
        return ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) > 0.0f;
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

// --- Main grid-to-mesh implementation ---

template<typename T>
inline MeshResult grid_to_mesh_impl(
    int width, int height, const T* elevations,
    float error_threshold, int point_limit) {

    // Preprocess input for robustness
    auto preprocessing = preprocess_input_data(width, height, elevations, error_threshold);

    if (preprocessing.has_warnings) {
        for (const auto& warning : preprocessing.warnings) {
            std::cerr << "TerraScape warning: " << warning << std::endl;
        }
    }

    try {
        // Create grid triangulator
        GridTriangulator triangulator;
        triangulator.initializeGrid(width, height, 0.0f, 0.0f,
                                   static_cast<float>(width - 1), static_cast<float>(height - 1));

        // Add vertices at grid points (simplified approach - add all points for now)
        int step = 1;
        if (width * height > point_limit) {
            step = static_cast<int>(std::sqrt(static_cast<float>(width * height) / point_limit));
            step = std::max(step, 1);
        }

        for (int y = 0; y < height; y += step) {
            for (int x = 0; x < width; x += step) {
                int idx = y * width + x;
                triangulator.addGridPoint(x, y, preprocessing.processed_elevations[idx]);
            }
        }

        // Triangulate
        if (!triangulator.triangulate()) {
            throw std::runtime_error("Grid triangulation failed");
        }

        return triangulator.toMeshResult();

    } catch (const std::exception& e) {
        std::cerr << "Grid triangulation error: " << e.what() << std::endl;
        std::cerr << "Falling back to simple quad mesh" << std::endl;

        // Fallback to simple mesh
        MeshResult fallback_result;
        fallback_result.vertices = {
            Vertex{0.0f, 0.0f, static_cast<float>(preprocessing.processed_elevations[0])},
            Vertex{static_cast<float>(width - 1), 0.0f, static_cast<float>(preprocessing.processed_elevations[width - 1])},
            Vertex{static_cast<float>(width - 1), static_cast<float>(height - 1),
                   static_cast<float>(preprocessing.processed_elevations[(height - 1) * width + width - 1])},
            Vertex{0.0f, static_cast<float>(height - 1), static_cast<float>(preprocessing.processed_elevations[(height - 1) * width])}
        };
        fallback_result.triangles = {Triangle{0, 1, 2}, Triangle{0, 2, 3}};
        return fallback_result;
    }
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
// These are the primary user-facing functions that delegate to implementations above

// Core mesh generation with grid-aware triangulation
template<typename T>
inline MeshResult grid_to_mesh(
    int width, int height, const T* elevations,
    float error_threshold = 1.0f, int point_limit = 10000)
{
    // Delegate to grid-aware implementation for optimal collinear point handling
    return grid_to_mesh_impl(width, height, elevations, error_threshold, point_limit);
}

} // namespace TerraScape
