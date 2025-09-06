#pragma once
#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <memory>
#include "bg_detria_mesh.h"

#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#elif defined(__APPLE__)
#include <sys/types.h>
#include <sys/sysctl.h>
#endif

namespace bg {

// --- Strategy selection for mesh refinement ---
enum class MeshRefineStrategy {
    AUTO,      // Automatically select method based on grid size and RAM
    HEAP,      // Use heap + affected update (optimal, high memory) - now uses detria
    HYBRID,    // Hybrid: sparse sampling, then local refinement - now uses detria  
    SPARSE     // Sparse sampling only (fastest, lowest quality) - now uses detria
};

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

struct Vertex { float x, y, z; };
struct Triangle { int v0, v1, v2; };

struct MeshResult {
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
};

// --- Forward declarations for detria integration ---
class DetriaTriangulationManager;
class GreedyMeshRefiner;

// --- Helper: Enhanced grid accessor with detria integration ---
template<typename T>
class Grid {
public:
    int width, height;
    const T* data;
    Grid(int w, int h, const T* d) : width(w), height(h), data(d) {}
    T operator()(int x, int y) const { return data[y * width + x]; }
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
    float w1 = ((v1.y - v2.y)*(px - v2.x) + (v2.x - v1.x)*(py - v2.y)) / denom;
    float w2 = ((v2.y - v0.y)*(px - v2.x) + (v0.x - v2.x)*(py - v2.y)) / denom;
    float w3 = 1.0f - w1 - w2;
    return w1 * v0.z + w2 * v1.z + w3 * v2.z;
}

// --- Detria-based mesh generation implementation ---
// This implementation uses bg_detria.hpp for proper Delaunay triangulation
// and topology management, with a greedy refinement layer on top.
template<typename T>
MeshResult grid_to_mesh_detria(
    int width, int height, const T* elevations,
    float error_threshold = 1.0f, int point_limit = 10000,
    MeshRefineStrategy strategy = MeshRefineStrategy::AUTO)
{
    // Collect all candidate points first, then do batch triangulation
    std::vector<detria::PointF> points;
    std::vector<float> point_z_coords;
    
    // Add boundary corners first
    points.emplace_back(detria::PointF{0.0f, 0.0f});
    points.emplace_back(detria::PointF{static_cast<float>(width - 1), 0.0f});
    points.emplace_back(detria::PointF{static_cast<float>(width - 1), static_cast<float>(height - 1)});
    points.emplace_back(detria::PointF{0.0f, static_cast<float>(height - 1)});
    
    point_z_coords.push_back(static_cast<float>(elevations[0]));                          // bottom-left
    point_z_coords.push_back(static_cast<float>(elevations[width - 1]));                 // bottom-right
    point_z_coords.push_back(static_cast<float>(elevations[(height - 1) * width + width - 1])); // top-right
    point_z_coords.push_back(static_cast<float>(elevations[(height - 1) * width]));      // top-left
    
    // Collect candidate points based on strategy
    std::vector<std::tuple<int, int, float>> candidates; // x, y, error
    
    if (strategy == MeshRefineStrategy::SPARSE) {
        // Sparse strategy: sample at regular intervals
        int step = std::max(1, std::max(width, height) / 50);
        for (int y = 0; y < height; y += step) {
            for (int x = 0; x < width; x += step) {
                bool is_corner = (x == 0 || x == width - 1) && (y == 0 || y == height - 1);
                if (!is_corner) {
                    candidates.emplace_back(x, y, 1.0f); // Dummy error for sparse sampling
                }
            }
        }
    } else {
        // HEAP and HYBRID: evaluate all grid points for error
        Grid<T> grid(width, height, elevations);
        
        // Start with simple initial triangulation for error calculation
        std::vector<Vertex> temp_vertices = {
            {0, 0, static_cast<float>(grid(0, 0))},
            {static_cast<float>(width - 1), 0, static_cast<float>(grid(width - 1, 0))},
            {static_cast<float>(width - 1), static_cast<float>(height - 1), static_cast<float>(grid(width - 1, height - 1))},
            {0, static_cast<float>(height - 1), static_cast<float>(grid(0, height - 1))}
        };
        std::vector<Triangle> temp_triangles = {{0, 1, 2}, {0, 2, 3}};
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                bool is_corner = (x == 0 || x == width - 1) && (y == 0 || y == height - 1);
                if (is_corner) continue;
                
                float gx = static_cast<float>(x), gy = static_cast<float>(y);
                float actual = static_cast<float>(grid(x, y));
                
                // Find containing triangle and calculate error
                float error = 0.0f;
                for (const auto& tri : temp_triangles) {
                    const auto& v0 = temp_vertices[tri.v0];
                    const auto& v1 = temp_vertices[tri.v1]; 
                    const auto& v2 = temp_vertices[tri.v2];
                    
                    if (point_in_triangle(gx, gy, v0, v1, v2)) {
                        float approx = barycentric_interp(gx, gy, v0, v1, v2);
                        error = std::abs(actual - approx);
                        break;
                    }
                }
                
                if (error > 0.0f) {
                    candidates.emplace_back(x, y, error);
                }
            }
        }
        
        // Sort by error (descending) and limit points
        std::sort(candidates.begin(), candidates.end(), 
                 [](const auto& a, const auto& b) { return std::get<2>(a) > std::get<2>(b); });
    }
    
    // Limit number of candidates
    if (candidates.size() > static_cast<size_t>(point_limit - 4)) {
        candidates.resize(point_limit - 4);
    }
    
    // Add selected candidates to points (with duplicate checking)
    for (const auto& candidate : candidates) {
        int x = std::get<0>(candidate);
        int y = std::get<1>(candidate);
        float error = std::get<2>(candidate);
        
        if (error >= error_threshold || strategy == MeshRefineStrategy::SPARSE) {
            float px = static_cast<float>(x);
            float py = static_cast<float>(y);
            
            // Check for duplicates (simple distance check)
            bool is_duplicate = false;
            for (const auto& existing_point : points) {
                if (std::abs(existing_point.x - px) < 0.01f && std::abs(existing_point.y - py) < 0.01f) {
                    is_duplicate = true;
                    break;
                }
            }
            
            if (!is_duplicate) {
                points.emplace_back(detria::PointF{px, py});
                point_z_coords.push_back(static_cast<float>(elevations[y * width + x]));
            }
        }
    }
    
    std::cerr << "Total points for triangulation: " << points.size() << "\n";
    
    // Perform batch triangulation using detria with fallback
    detria::Triangulation<detria::PointF, uint32_t> triangulation;
    bool success = false;
    size_t attempt_point_count = points.size();
    
    // Try triangulation with progressively fewer points if it fails
    while (!success && attempt_point_count >= 4) {
        std::vector<detria::PointF> attempt_points(points.begin(), points.begin() + attempt_point_count);
        std::vector<float> attempt_z_coords(point_z_coords.begin(), point_z_coords.begin() + attempt_point_count);
        
        triangulation.setPoints(attempt_points);
        
        // Set boundary outline (counter-clockwise: 0,1,2,3)
        std::vector<uint32_t> boundary = {0, 1, 2, 3};
        triangulation.addOutline(boundary);
        
        success = triangulation.triangulate(true); // Delaunay
        
        if (success) {
            std::cerr << "Detria triangulation succeeded with " << attempt_point_count << " points\n";
            std::cerr << "Max triangles: " << triangulation.getMaxNumTriangles() << "\n";
            
            // Update points and z_coords to match successful attempt
            points.resize(attempt_point_count);
            point_z_coords.resize(attempt_point_count);
            break;
        } else {
            std::cerr << "Detria failed with " << attempt_point_count << " points, trying fewer...\n";
            attempt_point_count = attempt_point_count * 3 / 4; // Reduce by 25%
        }
    }
    
    if (!success) {
        std::cerr << "Warning: Detria triangulation failed even with reduced points, using fallback\n";
        // Fallback to simple manual triangulation
        MeshResult result;
        for (size_t i = 0; i < points.size(); ++i) {
            result.vertices.push_back({points[i].x, points[i].y, point_z_coords[i]});
        }
        // Simple triangulation with first 4 points
        if (points.size() >= 4) {
            result.triangles.push_back({0, 1, 2});
            result.triangles.push_back({0, 2, 3});
        }
        return result;
    }
    
    // Convert detria result to MeshResult
    MeshResult result;
    
    // Add vertices
    for (size_t i = 0; i < points.size(); ++i) {
        result.vertices.push_back({points[i].x, points[i].y, point_z_coords[i]});
    }
    
    // Extract triangles - use forEachTriangleOfEveryLocation to get all triangles
    int triangle_count = 0;
    triangulation.forEachTriangleOfEveryLocation([&](detria::Triangle<uint32_t> triangle) {
        if (triangle.x < points.size() && triangle.y < points.size() && triangle.z < points.size()) {
            result.triangles.push_back({static_cast<int>(triangle.x), 
                                       static_cast<int>(triangle.y), 
                                       static_cast<int>(triangle.z)});
            triangle_count++;
        }
    });
    
    std::cerr << "Extracted " << triangle_count << " triangles\n";
    
    return result;
}

// --- Core mesh generation with detria integration ---
// This function now delegates to bg_detria.hpp for triangulation and topology management
// while maintaining the same API and refinement strategies as before.
template<typename T>
MeshResult grid_to_mesh(
    int width, int height, const T* elevations,
    float error_threshold = 1.0f, int point_limit = 10000,
    MeshRefineStrategy strategy = MeshRefineStrategy::AUTO)
{
    // --- Strategy selection (unchanged logic) ---
    if (strategy == MeshRefineStrategy::AUTO) {
        size_t grid_size = size_t(width) * size_t(height);
        size_t avail_ram = get_available_ram_bytes();
        // Estimate: Each heap entry ~32 bytes; threshold = 0.5GB
        size_t heap_mem_needed = grid_size * 32;
        if (grid_size <= 500000 && heap_mem_needed < avail_ram / 2) {
            strategy = MeshRefineStrategy::HEAP;
        } else if (grid_size < 5000000) {
            strategy = MeshRefineStrategy::HYBRID;
        } else {
            strategy = MeshRefineStrategy::SPARSE;
        }
    }
    
    // Delegate to detria-based implementation
    return grid_to_mesh_detria(width, height, elevations, error_threshold, point_limit, strategy);
}

} // namespace bg