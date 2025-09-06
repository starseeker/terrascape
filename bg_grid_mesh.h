#pragma once
#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <limits>

/*
   bg_grid_to_mesh API (C++ version)
   Inputs:
     - width: grid width
     - height: grid height
     - elevations: pointer to contiguous array of floats/doubles [height][width]
     - error_threshold: stop refinement when max error < threshold
     - point_limit: stop when mesh reaches this number of vertices

   Outputs:
     - vertices: std::vector<Vertex> {x, y, z}
     - triangles: std::vector<Triangle> {v0, v1, v2} (vertex indices)
*/

namespace bg {

struct Vertex { float x, y, z; };
struct Triangle { int v0, v1, v2; };

// Helper: 2D grid accessor
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

// A simple triangle mesh for the grid
struct MeshResult {
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
};

// Utility: check if a point is in a triangle using barycentric coordinates
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

// Utility: interpolate z at a point inside a triangle using barycentric coordinates
inline float barycentric_interp(float px, float py,
                                const Vertex& v0, const Vertex& v1, const Vertex& v2)
{
    float denom = ((v1.y - v2.y)*(v0.x - v2.x) + (v2.x - v1.x)*(v0.y - v2.y));
    float w1 = ((v1.y - v2.y)*(px - v2.x) + (v2.x - v1.x)*(py - v2.y)) / denom;
    float w2 = ((v2.y - v0.y)*(px - v2.x) + (v0.x - v2.x)*(py - v2.y)) / denom;
    float w3 = 1.0f - w1 - w2;
    return w1 * v0.z + w2 * v1.z + w3 * v2.z;
}

/*
    A simple greedy grid-to-mesh implementation:
    - Starts with grid corners as two triangles
    - Iteratively inserts the grid point with highest error
    - Updates mesh triangles (stub: in this version, splits triangles containing the new point)
    - Stops when error threshold or point limit is reached
*/
template<typename T>
MeshResult grid_to_mesh(
    int width, int height, const T* elevations,
    float error_threshold = 1.0f, int point_limit = 10000)
{
    MeshResult result;
    Grid<T> grid(width, height, elevations);

    // Seed mesh with grid corners
    result.vertices.push_back({0, 0, float(grid(0, 0))});
    result.vertices.push_back({float(width - 1), 0, float(grid(width - 1, 0))});
    result.vertices.push_back({float(width - 1), float(height - 1), float(grid(width - 1, height - 1))});
    result.vertices.push_back({0, float(height - 1), float(grid(0, height - 1))});
    result.triangles.push_back({0, 1, 2});
    result.triangles.push_back({0, 2, 3});

    // Candidate points: sample the grid more sparsely for large grids
    std::vector<std::pair<int, int>> candidates;
    int step = std::max(1, std::max(width, height) / 50); // Sample roughly 50x50 points max
    for (int y = 0; y < height; y += step) {
        for (int x = 0; x < width; x += step) {
            bool is_corner = (x == 0 || x == width - 1) && (y == 0 || y == height - 1);
            if (!is_corner) {
                candidates.emplace_back(x, y);
            }
        }
    }

    int inserted = 4;
    int max_iterations = std::min(point_limit * 2, int(candidates.size())); // Prevent infinite loops
    int iterations = 0;
    
    while (inserted < point_limit && iterations < max_iterations) {
        iterations++;
        float max_err = -1.0f;
        int max_x = -1, max_y = -1;
        int tri_idx = -1;

        // For each candidate, compute error (difference between grid and mesh)
        for (size_t i = 0; i < candidates.size(); ++i) {
            int x = candidates[i].first, y = candidates[i].second;
            float gx = float(x), gy = float(y);
            float actual = grid(x, y);

            // Find triangle containing (x, y)
            float approx = 0.0f;
            bool found = false;
            int containing = -1;
            for (size_t t = 0; t < result.triangles.size(); ++t) {
                const auto& tri = result.triangles[t];
                const auto& v0 = result.vertices[tri.v0];
                const auto& v1 = result.vertices[tri.v1];
                const auto& v2 = result.vertices[tri.v2];

                if (point_in_triangle(gx, gy, v0, v1, v2)) {
                    approx = barycentric_interp(gx, gy, v0, v1, v2);
                    found = true;
                    containing = int(t);
                    break;
                }
            }
            // If not found, skip (outside current mesh)
            if (!found) continue;

            float err = std::abs(actual - approx);
            if (err > max_err) {
                max_err = err;
                max_x = x;
                max_y = y;
                tri_idx = containing;
            }
        }

        if (max_err < error_threshold || max_x < 0 || max_y < 0 || tri_idx < 0) break;

        // Insert this point as a new vertex
        result.vertices.push_back({float(max_x), float(max_y), float(grid(max_x, max_y))});
        int new_idx = int(result.vertices.size()) - 1;

        // Update mesh: split the triangle containing the point into 3 triangles
        const auto tri = result.triangles[tri_idx];
        result.triangles.erase(result.triangles.begin() + tri_idx);
        result.triangles.push_back({tri.v0, tri.v1, new_idx});
        result.triangles.push_back({tri.v1, tri.v2, new_idx});
        result.triangles.push_back({tri.v2, tri.v0, new_idx});

        inserted++;

        // Remove from candidates
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
            [max_x, max_y](const std::pair<int, int>& p) { return p.first == max_x && p.second == max_y; }),
            candidates.end());
    }

    return result;
}

} // namespace bg