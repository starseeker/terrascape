#pragma once
#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <memory>

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
    HEAP,      // Use heap + affected update (optimal, high memory)
    HYBRID,    // Hybrid: sparse sampling, then local refinement
    SPARSE     // Sparse sampling only (fastest, lowest quality)
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

// --- Helper: 2D grid accessor ---
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

// --- Core Hybrid/Heap Mesh Generation ---
template<typename T>
MeshResult grid_to_mesh(
    int width, int height, const T* elevations,
    float error_threshold = 1.0f, int point_limit = 10000,
    MeshRefineStrategy strategy = MeshRefineStrategy::AUTO)
{
    MeshResult result;
    Grid<T> grid(width, height, elevations);
    size_t grid_size = size_t(width) * size_t(height);

    // --- Strategy selection ---
    if (strategy == MeshRefineStrategy::AUTO) {
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

    // --- Initial mesh: corners ---
    result.vertices.push_back({0, 0, float(grid(0, 0))});
    result.vertices.push_back({float(width - 1), 0, float(grid(width - 1, 0))});
    result.vertices.push_back({float(width - 1), float(height - 1), float(grid(width - 1, height - 1))});
    result.vertices.push_back({0, float(height - 1), float(grid(0, height - 1))});
    result.triangles.push_back({0, 1, 2});
    result.triangles.push_back({0, 2, 3});

    // --- SPARSE strategy ---
    if (strategy == MeshRefineStrategy::SPARSE) {
        int step = std::max(1, std::max(width, height) / 50);
        std::vector<std::pair<int, int>> candidates;
        for (int y = 0; y < height; y += step) {
            for (int x = 0; x < width; x += step) {
                bool is_corner = (x == 0 || x == width - 1) && (y == 0 || y == height - 1);
                if (!is_corner) candidates.emplace_back(x, y);
            }
        }
        int inserted = 4, max_iterations = std::min(point_limit * 2, int(candidates.size()));
        int iterations = 0;
        while (inserted < point_limit && iterations < max_iterations) {
            iterations++;
            float max_err = -1.0f; int max_x = -1, max_y = -1, tri_idx = -1;
            for (size_t i = 0; i < candidates.size(); ++i) {
                int x = candidates[i].first, y = candidates[i].second;
                float gx = float(x), gy = float(y), actual = grid(x, y);
                float approx = 0.0f; bool found = false; int containing = -1;
                for (size_t t = 0; t < result.triangles.size(); ++t) {
                    const auto& tri = result.triangles[t];
                    const auto& v0 = result.vertices[tri.v0], v1 = result.vertices[tri.v1], v2 = result.vertices[tri.v2];
                    if (point_in_triangle(gx, gy, v0, v1, v2)) {
                        approx = barycentric_interp(gx, gy, v0, v1, v2);
                        found = true; containing = int(t); break;
                    }
                }
                if (!found) continue;
                float err = std::abs(actual - approx);
                if (err > max_err) { max_err = err; max_x = x; max_y = y; tri_idx = containing; }
            }
            if (max_err < error_threshold || max_x < 0 || max_y < 0 || tri_idx < 0) break;
            result.vertices.push_back({float(max_x), float(max_y), float(grid(max_x, max_y))});
            int new_idx = int(result.vertices.size()) - 1;
            const auto tri = result.triangles[tri_idx];
            result.triangles.erase(result.triangles.begin() + tri_idx);
            result.triangles.push_back({tri.v0, tri.v1, new_idx});
            result.triangles.push_back({tri.v1, tri.v2, new_idx});
            result.triangles.push_back({tri.v2, tri.v0, new_idx});
            inserted++;
            candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                [max_x, max_y](const std::pair<int, int>& p) { return p.first == max_x && p.second == max_y; }),
                candidates.end());
        }
        return result;
    }

    // --- HEAP strategy (full greedy) ---
    if (strategy == MeshRefineStrategy::HEAP) {
        struct HeapEntry {
            int x, y, tri_idx;
            float err;
            bool operator<(const HeapEntry& h) const { return err < h.err; }
        };
        std::vector<HeapEntry> heap_entries;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                bool is_corner = (x == 0 || x == width - 1) && (y == 0 || y == height - 1);
                if (is_corner) continue;
                float gx = float(x), gy = float(y), actual = grid(x, y);
                for (size_t t = 0; t < result.triangles.size(); ++t) {
                    const auto& tri = result.triangles[t];
                    const auto& v0 = result.vertices[tri.v0], v1 = result.vertices[tri.v1], v2 = result.vertices[tri.v2];
                    if (point_in_triangle(gx, gy, v0, v1, v2)) {
                        float approx = barycentric_interp(gx, gy, v0, v1, v2);
                        float err = std::abs(actual - approx);
                        heap_entries.push_back({x, y, int(t), err});
                        break;
                    }
                }
            }
        }
        auto cmp = [](const HeapEntry& a, const HeapEntry& b) { return a.err < b.err; };
        std::make_heap(heap_entries.begin(), heap_entries.end(), cmp);
        int inserted = 4;
        while (inserted < point_limit && !heap_entries.empty()) {
            std::pop_heap(heap_entries.begin(), heap_entries.end(), cmp);
            HeapEntry top = heap_entries.back(); heap_entries.pop_back();
            if (top.err < error_threshold) break;
            result.vertices.push_back({float(top.x), float(top.y), float(grid(top.x, top.y))});
            int new_idx = int(result.vertices.size()) - 1;
            const auto tri = result.triangles[top.tri_idx];
            result.triangles.erase(result.triangles.begin() + top.tri_idx);
            result.triangles.push_back({tri.v0, tri.v1, new_idx});
            result.triangles.push_back({tri.v1, tri.v2, new_idx});
            result.triangles.push_back({tri.v2, tri.v0, new_idx});
            inserted++;
            // Update heap: only entries inside affected triangles need errors recalculated.
            for (auto& entry : heap_entries) {
                float gx = float(entry.x), gy = float(entry.y);
                bool found = false;
                for (size_t t = 0; t < result.triangles.size(); ++t) {
                    const auto& tri2 = result.triangles[t];
                    const auto& v0 = result.vertices[tri2.v0], v1 = result.vertices[tri2.v1], v2 = result.vertices[tri2.v2];
                    if (point_in_triangle(gx, gy, v0, v1, v2)) {
                        float approx = barycentric_interp(gx, gy, v0, v1, v2);
                        entry.tri_idx = int(t);
                        entry.err = std::abs(grid(entry.x, entry.y) - approx);
                        found = true; break;
                    }
                }
                if (!found) entry.err = 0; // No longer in mesh
            }
            // Remove any with zero error or not in mesh (optional for efficiency)
            heap_entries.erase(std::remove_if(heap_entries.begin(), heap_entries.end(),
                [](const HeapEntry& e) { return e.err == 0; }), heap_entries.end());
            std::make_heap(heap_entries.begin(), heap_entries.end(), cmp);
        }
        return result;
    }

    // --- HYBRID strategy ---
    if (strategy == MeshRefineStrategy::HYBRID) {
        int step = std::max(1, std::max(width, height) / 50);
        std::vector<std::pair<int, int>> candidates;
        for (int y = 0; y < height; y += step) {
            for (int x = 0; x < width; x += step) {
                bool is_corner = (x == 0 || x == width - 1) && (y == 0 || y == height - 1);
                if (!is_corner) candidates.emplace_back(x, y);
            }
        }
        int inserted = 4, max_iterations = std::min(point_limit * 2, int(candidates.size()));
        int iterations = 0;
        while (inserted < point_limit && iterations < max_iterations) {
            iterations++;
            float max_err = -1.0f; int max_x = -1, max_y = -1, tri_idx = -1;
            for (size_t i = 0; i < candidates.size(); ++i) {
                int x = candidates[i].first, y = candidates[i].second;
                float gx = float(x), gy = float(y), actual = grid(x, y);
                float approx = 0.0f; bool found = false; int containing = -1;
                for (size_t t = 0; t < result.triangles.size(); ++t) {
                    const auto& tri = result.triangles[t];
                    const auto& v0 = result.vertices[tri.v0], v1 = result.vertices[tri.v1], v2 = result.vertices[tri.v2];
                    if (point_in_triangle(gx, gy, v0, v1, v2)) {
                        approx = barycentric_interp(gx, gy, v0, v1, v2);
                        found = true; containing = int(t); break;
                    }
                }
                if (!found) continue;
                float err = std::abs(actual - approx);
                if (err > max_err) { max_err = err; max_x = x; max_y = y; tri_idx = containing; }
            }
            if (max_err < error_threshold || max_x < 0 || max_y < 0 || tri_idx < 0) break;
            // If highest error is very high, increase density and locally refine
            if (max_err > error_threshold * 10 && step > 1) {
                step = std::max(1, step / 2); // Increase density
                for (int y = 0; y < height; y += step) {
                    for (int x = 0; x < width; x += step) {
                        bool is_corner = (x == 0 || x == width - 1) && (y == 0 || y == height - 1);
                        auto it = std::find(candidates.begin(), candidates.end(), std::make_pair(x, y));
                        if (!is_corner && it == candidates.end()) candidates.emplace_back(x, y);
                    }
                }
                continue; // Next iteration uses denser candidates
            }
            result.vertices.push_back({float(max_x), float(max_y), float(grid(max_x, max_y))});
            int new_idx = int(result.vertices.size()) - 1;
            const auto tri = result.triangles[tri_idx];
            result.triangles.erase(result.triangles.begin() + tri_idx);
            result.triangles.push_back({tri.v0, tri.v1, new_idx});
            result.triangles.push_back({tri.v1, tri.v2, new_idx});
            result.triangles.push_back({tri.v2, tri.v0, new_idx});
            inserted++;
            candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                [max_x, max_y](const std::pair<int, int>& p) { return p.first == max_x && p.second == max_y; }),
                candidates.end());
        }
        return result;
    }

    // Fallback: SPARSE
    return grid_to_mesh(width, height, elevations, error_threshold, point_limit, MeshRefineStrategy::SPARSE);
}

} // namespace bg