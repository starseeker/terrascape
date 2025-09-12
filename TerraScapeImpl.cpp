#include "TerraScapeImpl.h"
#include "TerraScape.hpp"
#include "GridTriangulator.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <chrono>
#include <map>
#include <set>

namespace TerraScape {

// --- Simulation of Simplicity functions for degenerate case handling ---
static float simulation_of_simplicity_perturbation(int x, int y, int width, int height, float magnitude) {
    // Create a deterministic pseudo-random perturbation based on position
    // This ensures consistent results across runs while breaking degeneracies
    uint32_t hash = static_cast<uint32_t>(x * 73856093 ^ y * 19349663 ^ width * 83492791 ^ height * 50331653);
    hash = ((hash >> 16) ^ hash) * 0x45d9f3b;
    hash = ((hash >> 16) ^ hash) * 0x45d9f3b;
    hash = (hash >> 16) ^ hash;
    
    // Convert hash to [-1, 1] range and scale by magnitude
    float normalized = (static_cast<float>(hash) / static_cast<float>(0xFFFFFFFF)) * 2.0f - 1.0f;
    return normalized * magnitude;
}

static int simulation_of_simplicity_orientation(float ax, float ay, int ai, float bx, float by, int bi, float cx, float cy, int ci) {
    // Standard 2D orientation test
    float det = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    
    const float EPSILON = 1e-10f;
    
    if (std::abs(det) > EPSILON) {
        return (det > 0) ? 1 : -1;
    }
    
    // Degenerate case - use Simulation of Simplicity ordering
    // Break ties using lexicographic ordering of indices
    if (ai != bi) return (ai < bi) ? -1 : 1;
    if (bi != ci) return (bi < ci) ? -1 : 1;
    if (ai != ci) return (ai < ci) ? -1 : 1;
    
    // All indices equal - should not happen in practice
    return 0;
}

static bool detect_regular_pattern(const float* elevations, int width, int height) {
    if (width < 3 || height < 3) return false;
    
    // Check for regular grid patterns by examining elevation differences
    bool has_regular_x_pattern = true;
    bool has_regular_y_pattern = true;
    
    // Check X direction patterns (along rows)
    for (int y = 0; y < height && has_regular_x_pattern; ++y) {
        if (width < 3) break;
        float diff1 = elevations[y * width + 1] - elevations[y * width + 0];
        for (int x = 2; x < width; ++x) {
            float diff2 = elevations[y * width + x] - elevations[y * width + x - 1];
            if (std::abs(diff2 - diff1) > 1e-6f) {
                has_regular_x_pattern = false;
                break;
            }
        }
    }
    
    // Check Y direction patterns (along columns)
    for (int x = 0; x < width && has_regular_y_pattern; ++x) {
        if (height < 3) break;
        float diff1 = elevations[1 * width + x] - elevations[0 * width + x];
        for (int y = 2; y < height; ++y) {
            float diff2 = elevations[y * width + x] - elevations[(y - 1) * width + x];
            if (std::abs(diff2 - diff1) > 1e-6f) {
                has_regular_y_pattern = false;
                break;
            }
        }
    }
    
    return has_regular_x_pattern || has_regular_y_pattern;
}

static bool detect_collinear_rows(const float* elevations, int width, int height) {
    if (width < 3) return false;
    
    // Check for rows/columns where all points are collinear
    int collinear_rows = 0;
    int collinear_cols = 0;
    
    // Check rows for collinearity
    for (int y = 0; y < height; ++y) {
        bool row_collinear = true;
        for (int x = 2; x < width && row_collinear; ++x) {
            // Check if three consecutive points in this row are collinear
            float x1 = static_cast<float>(x - 2), y1 = static_cast<float>(y), z1 = elevations[y * width + (x - 2)];
            float x2 = static_cast<float>(x - 1), y2 = static_cast<float>(y), z2 = elevations[y * width + (x - 1)];
            float x3 = static_cast<float>(x), y3 = static_cast<float>(y), z3 = elevations[y * width + x];
            
            // Cross product to check collinearity in 2D projection
            float cross = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
            if (std::abs(cross) > 1e-6f) {
                row_collinear = false;
            }
        }
        if (row_collinear) collinear_rows++;
    }
    
    // Check columns for collinearity
    for (int x = 0; x < width; ++x) {
        bool col_collinear = true;
        for (int y = 2; y < height && col_collinear; ++y) {
            // Check if three consecutive points in this column are collinear
            float x1 = static_cast<float>(x), y1 = static_cast<float>(y - 2), z1 = elevations[(y - 2) * width + x];
            float x2 = static_cast<float>(x), y2 = static_cast<float>(y - 1), z2 = elevations[(y - 1) * width + x];
            float x3 = static_cast<float>(x), y3 = static_cast<float>(y), z3 = elevations[y * width + x];
            
            // Cross product to check collinearity in 2D projection
            float cross = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
            if (std::abs(cross) > 1e-6f) {
                col_collinear = false;
            }
        }
        if (col_collinear) collinear_cols++;
    }
    
    // Consider problematic if more than half the rows/columns are collinear
    return (collinear_rows > height / 2) || (collinear_cols > width / 2);
}

// --- Input Preprocessing Implementation ---
template<typename T>
PreprocessingResult preprocess_input_data(
    int width, int height, const T* elevations,
    float& error_threshold, bool enable_jitter)
{
    PreprocessingResult result;
    result.processed_elevations.reserve(static_cast<size_t>(width) * height);
    
    // === STEP 1: Convert to float and find elevation range ===
    float min_elev = std::numeric_limits<float>::max();
    float max_elev = std::numeric_limits<float>::lowest();
    
    for (int i = 0; i < width * height; ++i) {
        float elev = static_cast<float>(elevations[i]);
        result.processed_elevations.push_back(elev);
        min_elev = std::min(min_elev, elev);
        max_elev = std::max(max_elev, elev);
    }
    
    float elevation_range = max_elev - min_elev;
    
    // === STEP 2: Detect problematic patterns ===
    bool is_flat = (elevation_range < 1e-6f);
    bool has_regular_pattern = detect_regular_pattern(result.processed_elevations.data(), width, height);
    bool has_collinear_rows = detect_collinear_rows(result.processed_elevations.data(), width, height);
    
    // === STEP 3: Apply fixes and generate warnings ===
    if (is_flat) {
        result.warnings.push_back("Grid is perfectly flat (elevation range: " + std::to_string(elevation_range) + ")");
        result.has_warnings = true;
        result.is_degenerate = true;
        
        // For completely flat grids, skip jitter to preserve the flatness
        result.needs_jitter = false;
    }
    
    if (has_regular_pattern) {
        result.warnings.push_back("Detected regular pattern that may cause triangulation degeneracy");
        result.has_warnings = true;
        
        if (enable_jitter && !is_flat) {
            result.needs_jitter = true;
        }
    }
    
    if (has_collinear_rows) {
        result.warnings.push_back("Detected collinear rows/columns that may cause triangulation issues");
        result.has_warnings = true;
        
        if (enable_jitter && !is_flat) {
            result.needs_jitter = true;
        }
    }
    
    // === STEP 4: Apply Simulation of Simplicity perturbations ===
    if (result.needs_jitter || has_regular_pattern || has_collinear_rows) {
        result.warnings.push_back("Applied Simulation of Simplicity perturbation for deterministic degeneracy handling");
        
        // Apply very small deterministic perturbations
        float perturbation_magnitude = std::max(1e-8f, elevation_range * 1e-10f);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                float perturbation = simulation_of_simplicity_perturbation(x, y, width, height, perturbation_magnitude);
                result.processed_elevations[idx] += perturbation;
            }
        }
    }
    
    // === STEP 5: Adjust error threshold ===
    result.adjusted_error_threshold = error_threshold;
    
    // Clamp error threshold to reasonable range based on elevation range
    float min_threshold = std::max(1e-6f, elevation_range * 1e-8f);
    float max_threshold = elevation_range * 0.1f;
    
    if (result.adjusted_error_threshold < min_threshold) {
        result.adjusted_error_threshold = min_threshold;
        result.warnings.push_back("Error threshold clamped to minimum: " + std::to_string(min_threshold));
        result.has_warnings = true;
    }
    
    if (result.adjusted_error_threshold > max_threshold) {
        result.adjusted_error_threshold = max_threshold;
        result.warnings.push_back("Error threshold clamped to maximum: " + std::to_string(max_threshold));
        result.has_warnings = true;
    }
    
    // === STEP 6: Report preprocessing results ===
    if (result.has_warnings) {
        std::cout << "Input preprocessing warnings:\n";
        for (const auto& warning : result.warnings) {
            std::cout << "  - " << warning << "\n";
        }
    }
    
    return result;
}

// --- Grid-aware triangulation implementation ---
template<typename T>
MeshResult grid_to_mesh_impl(
    int width, int height, const T* elevations,
    float error_threshold, int point_limit)
{
    std::cout << "Using grid-aware triangulation for " << width << "x" << height << " grid\n";
    
    // === INPUT VALIDATION ===
    if (width <= 0 || height <= 0 || elevations == nullptr) {
        std::cerr << "Error: Invalid input parameters for grid-aware triangulation\n";
        return MeshResult{};
    }
    
    // Handle degenerate cases
    if (width == 1 || height == 1) {
        std::cerr << "Warning: Grid-aware triangulation requires at least 2x2 grid, falling back to simple mesh\n";
        MeshResult simple_result;
        
        if (width == 1 && height == 1) {
            // Single point - create minimal triangle
            simple_result.vertices.emplace_back(Vertex{0.0f, 0.0f, static_cast<float>(elevations[0])});
            simple_result.vertices.emplace_back(Vertex{1.0f, 0.0f, static_cast<float>(elevations[0])});
            simple_result.vertices.emplace_back(Vertex{0.5f, 1.0f, static_cast<float>(elevations[0])});
            simple_result.triangles = {Triangle{0, 1, 2}};
        } else if (width == 1) {
            // Single column
            for (int y = 0; y < height; ++y) {
                simple_result.vertices.emplace_back(Vertex{0.0f, static_cast<float>(y), static_cast<float>(elevations[y])});
            }
            // Create triangles along the line (degenerate, but valid)
            for (int y = 0; y < height - 1; ++y) {
                simple_result.vertices.emplace_back(Vertex{1.0f, static_cast<float>(y), static_cast<float>(elevations[y])});
                simple_result.vertices.emplace_back(Vertex{1.0f, static_cast<float>(y + 1), static_cast<float>(elevations[y + 1])});
                
                uint32_t base = static_cast<uint32_t>(height + y * 2);
                simple_result.triangles.push_back(Triangle{static_cast<int>(y), static_cast<int>(base), static_cast<int>(y + 1)});
                simple_result.triangles.push_back(Triangle{static_cast<int>(y + 1), static_cast<int>(base), static_cast<int>(base + 1)});
            }
        } else {
            // Single row
            for (int x = 0; x < width; ++x) {
                simple_result.vertices.emplace_back(Vertex{static_cast<float>(x), 0.0f, static_cast<float>(elevations[x])});
            }
            // Create triangles along the line (degenerate, but valid)
            for (int x = 0; x < width - 1; ++x) {
                simple_result.vertices.emplace_back(Vertex{static_cast<float>(x), 1.0f, static_cast<float>(elevations[x])});
                simple_result.vertices.emplace_back(Vertex{static_cast<float>(x + 1), 1.0f, static_cast<float>(elevations[x + 1])});
                
                uint32_t base = static_cast<uint32_t>(width + x * 2);
                simple_result.triangles.push_back(Triangle{static_cast<int>(x), static_cast<int>(x + 1), static_cast<int>(base)});
                simple_result.triangles.push_back(Triangle{static_cast<int>(x + 1), static_cast<int>(base + 1), static_cast<int>(base)});
            }
        }
        return simple_result;
    }
    
    // === PREPROCESSING FOR ROBUSTNESS ===
    float adjusted_error_threshold = error_threshold;
    PreprocessingResult preprocessing = preprocess_input_data(width, height, elevations, adjusted_error_threshold);
    
    // Use preprocessed data for all subsequent operations
    const float* processed_elevations = preprocessing.processed_elevations.data();
    error_threshold = preprocessing.adjusted_error_threshold;
    
    // If the input is severely degenerate (completely flat), return a minimal mesh
    if (preprocessing.is_degenerate && !preprocessing.needs_jitter) {
        MeshResult simple_result;
        
        // Create a simple quad mesh for flat grids
        simple_result.vertices = {
            {0.0f, 0.0f, processed_elevations[0]},                                    // bottom-left
            {static_cast<float>(width - 1), 0.0f, processed_elevations[width - 1]},  // bottom-right
            {static_cast<float>(width - 1), static_cast<float>(height - 1), 
             processed_elevations[(height - 1) * width + width - 1]},                // top-right
            {0.0f, static_cast<float>(height - 1), processed_elevations[(height - 1) * width]} // top-left
        };
        
        simple_result.triangles = {
            {0, 1, 2},  // First triangle
            {0, 2, 3}   // Second triangle
        };
        
        return simple_result;
    }
    
    try {
        // Create grid triangulator
        auto grid_triangulator = std::make_unique<GridTriangulator>();
        
        // Initialize with grid bounds
        float minX = 0.0f, minY = 0.0f;
        float maxX = static_cast<float>(width - 1);
        float maxY = static_cast<float>(height - 1);
        
        grid_triangulator->initializeGrid(width, height, minX, minY, maxX, maxY);
        
        // Add grid points strategically
        // For grid-aware triangulation, we can be much more systematic
        
        // Always add corners first
        grid_triangulator->addGridPoint(0, 0, static_cast<float>(processed_elevations[0]));
        grid_triangulator->addGridPoint(width - 1, 0, static_cast<float>(processed_elevations[width - 1]));
        grid_triangulator->addGridPoint(width - 1, height - 1, static_cast<float>(processed_elevations[(height - 1) * width + width - 1]));
        grid_triangulator->addGridPoint(0, height - 1, static_cast<float>(processed_elevations[(height - 1) * width]));
        
        // Add edge points for better boundary representation
        int edge_step = std::max(1, std::max(width, height) / 10);
        
        // Bottom and top edges
        for (int x = edge_step; x < width - 1; x += edge_step) {
            grid_triangulator->addGridPoint(x, 0, static_cast<float>(processed_elevations[x]));
            grid_triangulator->addGridPoint(x, height - 1, static_cast<float>(processed_elevations[(height - 1) * width + x]));
        }
        
        // Left and right edges
        for (int y = edge_step; y < height - 1; y += edge_step) {
            grid_triangulator->addGridPoint(0, y, static_cast<float>(processed_elevations[y * width]));
            grid_triangulator->addGridPoint(width - 1, y, static_cast<float>(processed_elevations[y * width + width - 1]));
        }
        
        // Add interior points based on error threshold and point limit
        int interior_step = std::max(1, std::min(width, height) / 8);
        
        // Adaptive step size based on point limit
        int estimated_points = 4 + (width + height) / edge_step * 2 + (width / interior_step) * (height / interior_step);
        if (estimated_points > point_limit) {
            interior_step = std::max(1, static_cast<int>(interior_step * std::sqrt(static_cast<float>(estimated_points) / point_limit)));
        }
        
        std::cout << "Grid-aware: Adding interior points with step=" << interior_step << "\n";
        
        // Add interior points systematically
        for (int y = interior_step; y < height; y += interior_step) {
            for (int x = interior_step; x < width; x += interior_step) {
                // Skip if too close to boundary
                if (x == 0 || x == width - 1 || y == 0 || y == height - 1) continue;
                
                grid_triangulator->addGridPoint(x, y, static_cast<float>(processed_elevations[y * width + x]));
            }
        }
        
        std::cout << "Grid-aware: Added " << grid_triangulator->getVertexCount() << " vertices\n";
        
        // Execute triangulation
        bool success = grid_triangulator->triangulate();
        
        if (!success) {
            std::cerr << "Grid-aware triangulation failed, creating fallback mesh\n";
            // Create simple quad mesh as fallback
            MeshResult fallback_result;
            fallback_result.vertices = {
                Vertex{0.0f, 0.0f, static_cast<float>(processed_elevations[0])},
                Vertex{maxX, 0.0f, static_cast<float>(processed_elevations[width - 1])},
                Vertex{maxX, maxY, static_cast<float>(processed_elevations[(height - 1) * width + width - 1])},
                Vertex{0.0f, maxY, static_cast<float>(processed_elevations[(height - 1) * width])}
            };
            fallback_result.triangles = {Triangle{0, 1, 2}, Triangle{0, 2, 3}};
            return fallback_result;
        }
        
        // Optimize triangle quality
        grid_triangulator->optimizeTriangleQuality();
        
        std::cout << "Grid-aware: Generated " << grid_triangulator->getTriangleCount() << " triangles\n";
        
        // Convert to standard mesh result
        return grid_triangulator->toMeshResult();
        
    } catch (const std::exception& e) {
        std::cerr << "Grid-aware triangulation error: " << e.what() << "\n";
        std::cerr << "Falling back to simple quad mesh\n";
        
        // Fallback to simple mesh
        MeshResult fallback_result;
        fallback_result.vertices = {
            Vertex{0.0f, 0.0f, static_cast<float>(processed_elevations[0])},
            Vertex{static_cast<float>(width - 1), 0.0f, static_cast<float>(processed_elevations[width - 1])},
            Vertex{static_cast<float>(width - 1), static_cast<float>(height - 1), 
                   static_cast<float>(processed_elevations[(height - 1) * width + width - 1])},
            Vertex{0.0f, static_cast<float>(height - 1), static_cast<float>(processed_elevations[(height - 1) * width])}
        };
        fallback_result.triangles = {Triangle{0, 1, 2}, Triangle{0, 2, 3}};
        return fallback_result;
    }
}

// --- Volumetric mesh implementations ---

std::vector<Edge> find_boundary_edges(const std::vector<Triangle>& triangles) {
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

MeshResult make_volumetric_mesh(const MeshResult& surface_mesh, float z_base) {
    MeshResult volumetric_result;
    volumetric_result.is_volumetric = true;
    
    // Copy surface vertices
    volumetric_result.vertices = surface_mesh.vertices;
    
    // Create base vertices (same x,y but z = z_base)
    std::vector<int> base_vertex_mapping(surface_mesh.vertices.size());
    for (size_t i = 0; i < surface_mesh.vertices.size(); ++i) {
        const Vertex& surface_vertex = surface_mesh.vertices[i];
        Vertex base_vertex = {surface_vertex.x, surface_vertex.y, z_base};
        base_vertex_mapping[i] = static_cast<int>(volumetric_result.vertices.size());
        volumetric_result.vertices.push_back(base_vertex);
    }
    
    // Copy surface triangles (maintain orientation)
    volumetric_result.triangles = surface_mesh.triangles;
    
    // Find boundary edges
    std::vector<Edge> boundary_edges = find_boundary_edges(surface_mesh.triangles);
    
    // Create side faces connecting surface boundary to base boundary
    for (const Edge& edge : boundary_edges) {
        int surface_v0 = edge.v0;
        int surface_v1 = edge.v1;
        int base_v0 = base_vertex_mapping[surface_v0];
        int base_v1 = base_vertex_mapping[surface_v1];
        
        // Create two triangles to form a quad face
        // Triangle 1: surface_v0 -> surface_v1 -> base_v0 (CCW when viewed from outside)
        volumetric_result.triangles.push_back({surface_v0, surface_v1, base_v0});
        
        // Triangle 2: surface_v1 -> base_v1 -> base_v0 (CCW when viewed from outside)
        volumetric_result.triangles.push_back({surface_v1, base_v1, base_v0});
    }
    
    // Create base triangles (reverse orientation compared to surface for inward-facing normals)
    for (const Triangle& surface_tri : surface_mesh.triangles) {
        int base_v0 = base_vertex_mapping[surface_tri.v0];
        int base_v1 = base_vertex_mapping[surface_tri.v1];
        int base_v2 = base_vertex_mapping[surface_tri.v2];
        
        // Reverse winding order for base (so normals point inward)
        volumetric_result.triangles.push_back({base_v0, base_v2, base_v1});
    }
    
    return volumetric_result;
}

VolumetricMeshResult make_volumetric_mesh_separated(const MeshResult& surface_mesh, float z_base) {
    VolumetricMeshResult result;
    
    const float epsilon = 1e-6f;  // Tolerance for height comparison
    
    // Separate triangles based on their centroid height relative to z_base
    std::vector<Triangle> positive_triangles, negative_triangles;
    
    for (const Triangle& tri : surface_mesh.triangles) {
        // Calculate triangle centroid
        const Vertex& v0 = surface_mesh.vertices[tri.v0];
        const Vertex& v1 = surface_mesh.vertices[tri.v1];
        const Vertex& v2 = surface_mesh.vertices[tri.v2];
        
        float centroid_z = (v0.z + v1.z + v2.z) / 3.0f;
        float height_diff = centroid_z - z_base;
        
        if (height_diff > epsilon) {
            positive_triangles.push_back(tri);
        } else if (height_diff < -epsilon) {
            negative_triangles.push_back(tri);
        }
        // Skip triangles at exactly the base level (height_diff within epsilon) to avoid degeneracies
    }
    
    // Create positive volume mesh if we have positive triangles
    if (!positive_triangles.empty()) {
        // Create a surface mesh with only positive triangles and their vertices
        MeshResult positive_surface;
        
        // Find all vertices used by positive triangles
        std::set<int> positive_vertex_indices;
        for (const Triangle& tri : positive_triangles) {
            positive_vertex_indices.insert(tri.v0);
            positive_vertex_indices.insert(tri.v1);
            positive_vertex_indices.insert(tri.v2);
        }
        
        // Create vertex mapping from old indices to new indices
        std::map<int, int> vertex_mapping;
        for (int old_idx : positive_vertex_indices) {
            int new_idx = static_cast<int>(positive_surface.vertices.size());
            vertex_mapping[old_idx] = new_idx;
            positive_surface.vertices.push_back(surface_mesh.vertices[old_idx]);
        }
        
        // Remap triangles to use new vertex indices
        for (const Triangle& tri : positive_triangles) {
            positive_surface.triangles.push_back({
                vertex_mapping[tri.v0],
                vertex_mapping[tri.v1],
                vertex_mapping[tri.v2]
            });
        }
        
        // Generate positive volumetric mesh using existing logic
        result.positive_volume = make_volumetric_mesh(positive_surface, z_base);
        result.has_positive_volume = true;
    }
    
    // Create negative volume mesh if we have negative triangles
    if (!negative_triangles.empty()) {
        // Create a surface mesh with only negative triangles and their vertices
        MeshResult negative_surface;
        
        // Find all vertices used by negative triangles
        std::set<int> negative_vertex_indices;
        for (const Triangle& tri : negative_triangles) {
            negative_vertex_indices.insert(tri.v0);
            negative_vertex_indices.insert(tri.v1);
            negative_vertex_indices.insert(tri.v2);
        }
        
        // Create vertex mapping from old indices to new indices
        std::map<int, int> vertex_mapping;
        for (int old_idx : negative_vertex_indices) {
            int new_idx = static_cast<int>(negative_surface.vertices.size());
            vertex_mapping[old_idx] = new_idx;
            negative_surface.vertices.push_back(surface_mesh.vertices[old_idx]);
        }
        
        // Remap triangles to use new vertex indices
        for (const Triangle& tri : negative_triangles) {
            negative_surface.triangles.push_back({
                vertex_mapping[tri.v0],
                vertex_mapping[tri.v1],
                vertex_mapping[tri.v2]
            });
        }
        
        // Generate negative volumetric mesh with reversed normals
        result.negative_volume.is_volumetric = true;
        
        // Copy surface vertices
        result.negative_volume.vertices = negative_surface.vertices;
        
        // Create base vertices at z_base
        std::vector<int> base_vertex_mapping(negative_surface.vertices.size());
        for (size_t i = 0; i < negative_surface.vertices.size(); ++i) {
            const Vertex& surface_vertex = negative_surface.vertices[i];
            Vertex base_vertex = {surface_vertex.x, surface_vertex.y, z_base};
            base_vertex_mapping[i] = static_cast<int>(result.negative_volume.vertices.size());
            result.negative_volume.vertices.push_back(base_vertex);
        }
        
        // Add surface triangles with reversed winding (since we want inverted normals for negative volume)
        for (const Triangle& surface_tri : negative_surface.triangles) {
            result.negative_volume.triangles.push_back({surface_tri.v0, surface_tri.v2, surface_tri.v1});
        }
        
        // Find boundary edges
        std::vector<Edge> boundary_edges = find_boundary_edges(negative_surface.triangles);
        
        // Create side faces connecting surface boundary to base boundary (reversed winding)
        for (const Edge& edge : boundary_edges) {
            int surface_v0 = edge.v0;
            int surface_v1 = edge.v1;
            int base_v0 = base_vertex_mapping[surface_v0];
            int base_v1 = base_vertex_mapping[surface_v1];
            
            // Reverse winding for negative volume
            result.negative_volume.triangles.push_back({surface_v0, base_v0, surface_v1});
            result.negative_volume.triangles.push_back({surface_v1, base_v0, base_v1});
        }
        
        // Create base triangles with same orientation as surface (since surface is already reversed)
        for (const Triangle& surface_tri : negative_surface.triangles) {
            int base_v0 = base_vertex_mapping[surface_tri.v0];
            int base_v1 = base_vertex_mapping[surface_tri.v1];
            int base_v2 = base_vertex_mapping[surface_tri.v2];
            
            result.negative_volume.triangles.push_back({base_v0, base_v1, base_v2});
        }
        
        result.has_negative_volume = true;
    }
    
    return result;
}

template<typename T>
MeshResult grid_to_mesh_volumetric(
    int width, int height, const T* elevations,
    float z_base,
    float error_threshold, int point_limit)
{
    // First generate the surface mesh
    MeshResult surface_mesh = grid_to_mesh_impl(width, height, elevations, 
                                               error_threshold, point_limit);
    
    // Convert to volumetric mesh
    return make_volumetric_mesh(surface_mesh, z_base);
}

template<typename T>
VolumetricMeshResult grid_to_mesh_volumetric_separated(
    int width, int height, const T* elevations,
    float z_base,
    float error_threshold, int point_limit)
{
    // First generate the surface mesh
    MeshResult surface_mesh = grid_to_mesh_impl(width, height, elevations, 
                                               error_threshold, point_limit);
    
    // Convert to separated volumetric meshes
    return make_volumetric_mesh_separated(surface_mesh, z_base);
}

// Explicit instantiations for grid_to_mesh_impl
template MeshResult grid_to_mesh_impl<float>(int width, int height, const float* elevations, float error_threshold, int point_limit);
template MeshResult grid_to_mesh_impl<double>(int width, int height, const double* elevations, float error_threshold, int point_limit);
template MeshResult grid_to_mesh_impl<int>(int width, int height, const int* elevations, float error_threshold, int point_limit);
template MeshResult grid_to_mesh_impl<unsigned short>(int width, int height, const unsigned short* elevations, float error_threshold, int point_limit);

// Explicit instantiations for preprocessing
template PreprocessingResult preprocess_input_data<float>(int width, int height, const float* elevations, float& error_threshold, bool enable_jitter);
template PreprocessingResult preprocess_input_data<double>(int width, int height, const double* elevations, float& error_threshold, bool enable_jitter);
template PreprocessingResult preprocess_input_data<int>(int width, int height, const int* elevations, float& error_threshold, bool enable_jitter);
template PreprocessingResult preprocess_input_data<unsigned short>(int width, int height, const unsigned short* elevations, float& error_threshold, bool enable_jitter);

// Explicit instantiations for volumetric mesh generation
template MeshResult grid_to_mesh_volumetric<float>(int width, int height, const float* elevations, float z_base, float error_threshold, int point_limit);
template MeshResult grid_to_mesh_volumetric<double>(int width, int height, const double* elevations, float z_base, float error_threshold, int point_limit);
template MeshResult grid_to_mesh_volumetric<int>(int width, int height, const int* elevations, float z_base, float error_threshold, int point_limit);

// Explicit instantiations for separated volumetric mesh generation
template VolumetricMeshResult grid_to_mesh_volumetric_separated<float>(int width, int height, const float* elevations, float z_base, float error_threshold, int point_limit);
template VolumetricMeshResult grid_to_mesh_volumetric_separated<double>(int width, int height, const double* elevations, float z_base, float error_threshold, int point_limit);
template VolumetricMeshResult grid_to_mesh_volumetric_separated<int>(int width, int height, const int* elevations, float z_base, float error_threshold, int point_limit);

} // namespace TerraScape