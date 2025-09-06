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

// --- Detria-based mesh generation implementation with TRUE incremental insertion ---
// This implementation uses bg_detria.hpp for proper Delaunay triangulation
// and includes proper incremental point insertion with localized error updates.
template<typename T>
MeshResult grid_to_mesh_detria(
    int width, int height, const T* elevations,
    float error_threshold = 1.0f, int point_limit = 10000,
    MeshRefineStrategy strategy = MeshRefineStrategy::AUTO)
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
    
    // Create greedy refiner for incremental insertion
    auto refiner = std::make_unique<GreedyMeshRefiner>(
        triangulation_manager.get(), error_threshold, point_limit);
    
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