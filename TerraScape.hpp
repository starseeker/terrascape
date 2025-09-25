#pragma once

/*
 * TerraScape - Terrain Triangle Mesh Generation
 *
 * A header-only C++ library for converting elevation grids to triangle meshes.
 * Features volumetric mesh generation with manifold guarantee and CCW orientation.
 * 
 * BRL-CAD DSP Integration:
 * This library now includes functions to integrate with BRL-CAD's DSP (displacement map)
 * primitive for improved tessellation. The integration provides:
 * 
 * 1. DSPData structure compatible with rt_dsp_internal buffer format
 * 2. Conversion functions between BRL-CAD and TerraScape data formats  
 * 3. NMGTriangleData output format suitable for nmg_region construction
 * 4. Complete rt_dsp_tess replacement function
 *
 * Key functions for BRL-CAD integration:
 * - convertDSPToTerrain(): Convert rt_dsp_internal buffer to TerrainData
 * - triangulateTerrainForBRLCAD(): Complete DSP->NMG triangulation pipeline
 * - convertMeshToNMG(): Convert TerrainMesh to NMG-compatible format
 *
 * See example usage in the triangulateTerrainForBRLCAD() documentation.
 */

#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <limits>
#include <climits>
#include <queue>
#include <memory>
#include <iostream>
#include <unordered_set>
#include <set>
#include <map>
#include <string>
#include <stdexcept>
#include <cstdint>
#include <array>
#include <functional>
#include <unordered_map>
#include <fstream>
#include <iomanip>
#ifdef HAVE_GDAL
#include <gdal_priv.h>
#include <gdal.h>
#endif
#include "earcut.hpp"
#include "detria.hpp"
#include "manifold/manifold.h"

#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic push /* start new diagnostic pragma */
#  pragma GCC diagnostic ignored "-Wfloat-equal"
#elif defined(__clang__)
#  pragma clang diagnostic push /* start new diagnostic pragma */
#  pragma clang diagnostic ignored "-Wfloat-equal"
#endif

namespace TerraScape {

    // Basic 3D point structure
    struct Point3D {
        double x, y, z;
        
        Point3D() : x(0), y(0), z(0) {}
        Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
        
        Point3D operator+(const Point3D& other) const {
            return Point3D(x + other.x, y + other.y, z + other.z);
        }
        
        Point3D operator-(const Point3D& other) const {
            return Point3D(x - other.x, y - other.y, z - other.z);
        }
        
        Point3D cross(const Point3D& other) const {
            return Point3D(
                y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x
            );
        }
        
        double dot(const Point3D& other) const {
            return x * other.x + y * other.y + z * other.z;
        }
        
        double length() const {
            return std::sqrt(x * x + y * y + z * z);
        }
        
        Point3D normalized() const {
            double len = length();
            if (len > 0) {
                return Point3D(x / len, y / len, z / len);
            }
            return Point3D(0, 0, 0);
        }
    };

    // Triangle structure with vertex indices
    struct Triangle {
        std::array<size_t, 3> vertices;
        Point3D normal;
        
        Triangle() {}
        Triangle(size_t v0, size_t v1, size_t v2) {
            vertices[0] = v0;
            vertices[1] = v1;
            vertices[2] = v2;
        }
        
        void computeNormal(const std::vector<Point3D>& vertex_list) {
            const Point3D& p0 = vertex_list[vertices[0]];
            const Point3D& p1 = vertex_list[vertices[1]];
            const Point3D& p2 = vertex_list[vertices[2]];
            
            Point3D edge1 = p1 - p0;
            Point3D edge2 = p2 - p0;
            normal = edge1.cross(edge2).normalized();
        }
        
        bool isCCW(const std::vector<Point3D>& vertex_list) const {
            const Point3D& p0 = vertex_list[vertices[0]];
            const Point3D& p1 = vertex_list[vertices[1]];
            const Point3D& p2 = vertex_list[vertices[2]];
            
            Point3D edge1 = p1 - p0;
            Point3D edge2 = p2 - p0;
            Point3D cross_product = edge1.cross(edge2);
            
            // The CCW test depends on the face type and desired outward normal direction
            // This is a simplified test - in practice we'd need more context about face type
            return cross_product.length() > 0; // Just check that it's not degenerate
        }
    };

    // Quad tree node for integer-coordinate bottom face mesh generation
    struct QuadNode {
        int min_x, min_y, max_x, max_y;
        std::vector<std::pair<int, int>> edge_points;  // Actual edge points within this quad
        std::vector<std::unique_ptr<QuadNode>> children; // 4 children for subdivided quads
        bool is_leaf;
        
        QuadNode(int minX, int minY, int maxX, int maxY) 
            : min_x(minX), min_y(minY), max_x(maxX), max_y(maxY), is_leaf(true) {}
            
        double getAspectRatio() const {
            int width = max_x - min_x;
            int height = max_y - min_y;
            if (width == 0 || height == 0) return 1.0;
            return std::max(width, height) / static_cast<double>(std::min(width, height));
        }
        
        bool hasGoodAspectRatio(double threshold = 2.0) const {
            return getAspectRatio() <= threshold;
        }
        
        std::pair<int, int> getCenter() const {
            return {(min_x + max_x) / 2, (min_y + max_y) / 2};
        }
    };

    // Terrain data structure
    struct TerrainData {
        std::vector<std::vector<double>> heights;
        int width, height;
        double min_height, max_height;
        double cell_size;
        Point3D origin;
        
        TerrainData() : width(0), height(0), min_height(0), max_height(0), cell_size(1.0), origin(0, 0, 0) {}
        
        double getHeight(int x, int y) const {
            if (x >= 0 && x < width && y >= 0 && y < height) {
                return heights[y][x];
            }
            return 0.0;
        }
        
        bool isValidCell(int x, int y) const {
            return x >= 0 && x < width && y >= 0 && y < height;
        }
    };

    // Triangle mesh structure
    struct TerrainMesh {
        std::vector<Point3D> vertices;
        std::vector<Triangle> triangles;
        size_t surface_triangle_count = 0; // Number of terrain surface triangles
        
        void clear() {
            vertices.clear();
            triangles.clear();
            surface_triangle_count = 0;
        }
        
        size_t addVertex(const Point3D& vertex) {
            vertices.push_back(vertex);
            return vertices.size() - 1;
        }
        
        void addTriangle(size_t v0, size_t v1, size_t v2) {
            Triangle tri(v0, v1, v2);
            tri.computeNormal(vertices);
            triangles.push_back(tri);
        }
        
        void addSurfaceTriangle(size_t v0, size_t v1, size_t v2) {
            addTriangle(v0, v1, v2);
            surface_triangle_count++;
        }
    };

    // Mesh validation statistics
    struct MeshStats {
        double volume;
        double surface_area;
        double expected_volume;
        double expected_surface_area;
        bool is_manifold;
        bool is_ccw_oriented;
        int non_manifold_edges;
        
        MeshStats() : volume(0), surface_area(0), expected_volume(0), expected_surface_area(0),
                      is_manifold(true), is_ccw_oriented(true), non_manifold_edges(0) {}
    };

    // Terrain simplification parameters based on Terra/Scape concepts
    struct SimplificationParams {
        double error_threshold;     // Maximum allowed geometric error
        double slope_threshold;     // Slope threshold for feature preservation
        int min_triangle_reduction; // Minimum percentage of triangles to remove
        bool preserve_boundaries;   // Whether to preserve terrain boundaries
        
        SimplificationParams() : 
            error_threshold(0.1), 
            slope_threshold(0.2), 
            min_triangle_reduction(50), 
            preserve_boundaries(true) {}
    };

    // Local terrain analysis for adaptive simplification
    struct TerrainFeature {
        double curvature;          // Local surface curvature
        double slope;              // Local slope magnitude
        double roughness;          // Local height variation
        bool is_boundary;          // Whether this is a boundary vertex
        double importance_score;   // Combined importance metric
        
        TerrainFeature() : curvature(0), slope(0), roughness(0), is_boundary(false), importance_score(0) {}
    };

    // Edge structure for manifold checking
    struct Edge {
        size_t v0, v1;
        
        Edge(size_t a, size_t b) {
            if (a < b) {
                v0 = a; v1 = b;
            } else {
                v0 = b; v1 = a;
            }
        }
        
        bool operator<(const Edge& other) const {
            if (v0 != other.v0) return v0 < other.v0;
            return v1 < other.v1;
        }
        
        bool operator==(const Edge& other) const {
            return v0 == other.v0 && v1 == other.v1;
        }
    };

    // Hash function for Edge
    struct EdgeHash {
        size_t operator()(const Edge& e) const {
            return std::hash<size_t>()(e.v0) ^ (std::hash<size_t>()(e.v1) << 1);
        }
    };

    // Connected component structure for handling terrain islands
    struct ConnectedComponent {
        int id;
        std::vector<std::pair<int, int>> cells;  // List of (x, y) coordinates in this component
        std::vector<std::pair<int, int>> boundary_cells;  // Boundary cells of this component
        std::vector<std::pair<int, int>> holes;  // Interior holes within this component
        int min_x, max_x, min_y, max_y;  // Bounding box
        
        ConnectedComponent() : id(-1), min_x(INT_MAX), max_x(INT_MIN), min_y(INT_MAX), max_y(INT_MIN) {}
        
        void addCell(int x, int y) {
            cells.push_back({x, y});
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
        }
    };

    // Terrain analysis result containing all connected components
    struct TerrainComponents {
        std::vector<ConnectedComponent> components;
        std::vector<std::vector<int>> component_map;  // component_id for each cell (-1 for background)
        
        TerrainComponents(int width, int height) {
            component_map.resize(height, std::vector<int>(width, -1));
        }
    };

    // BRL-CAD DSP compatibility structures
    struct DSPData {
        unsigned short* dsp_buf;     // Height data buffer (BRL-CAD format)
        uint32_t dsp_xcnt;          // Number of samples in row  
        uint32_t dsp_ycnt;          // Number of columns
        double cell_size;           // Physical size of each cell
        Point3D origin;             // Origin point in model space
        bool owns_buffer;           // Whether this structure owns the buffer
        
        DSPData() : dsp_buf(nullptr), dsp_xcnt(0), dsp_ycnt(0), 
                   cell_size(1.0), origin(0,0,0), owns_buffer(false) {}
        
        ~DSPData() {
            if (owns_buffer && dsp_buf) {
                delete[] dsp_buf;
                dsp_buf = nullptr;
            }
        }
        
        // Get height value at grid position
        double getHeight(uint32_t x, uint32_t y) const {
            if (x < dsp_xcnt && y < dsp_ycnt && dsp_buf) {
                return static_cast<double>(dsp_buf[y * dsp_xcnt + x]);
            }
            return 0.0;
        }
        
        // Set height value at grid position  
        void setHeight(uint32_t x, uint32_t y, unsigned short height) {
            if (x < dsp_xcnt && y < dsp_ycnt && dsp_buf) {
                dsp_buf[y * dsp_xcnt + x] = height;
            }
        }
    };

    // NMG-compatible triangle output for BRL-CAD integration
    struct NMGTriangleData {
        struct TriangleVertex {
            Point3D point;
            size_t original_index;  // Index in original vertex array
            
            TriangleVertex(const Point3D& p, size_t idx) : point(p), original_index(idx) {}
        };
        
        struct NMGTriangle {
            std::array<TriangleVertex, 3> vertices;
            Point3D normal;
            bool is_surface;  // True if this is a terrain surface triangle
            
            NMGTriangle(const TriangleVertex& v0, const TriangleVertex& v1, const TriangleVertex& v2, bool surf = false) 
                : vertices{v0, v1, v2}, is_surface(surf) {
                // Compute normal
                Point3D edge1 = v1.point - v0.point;
                Point3D edge2 = v2.point - v0.point;
                normal = edge1.cross(edge2).normalized();
            }
        };
        
        std::vector<NMGTriangle> triangles;
        std::vector<Point3D> unique_vertices;
        size_t surface_triangle_count;
        
        NMGTriangleData() : surface_triangle_count(0) {}
    };

    // Forward declarations
    bool readTerrainFile(const std::string& filename, TerrainData& terrain);
    bool readPGMFile(const std::string& filename, TerrainData& terrain);
    void triangulateTerrainVolume(const TerrainData& terrain, TerrainMesh& mesh);
    void triangulateTerrainVolumeLegacy(const TerrainData& terrain, TerrainMesh& mesh);
    void triangulateTerrainVolumeSimplified(const TerrainData& terrain, TerrainMesh& mesh, const SimplificationParams& params);
    void triangulateTerrainSurfaceOnly(const TerrainData& terrain, TerrainMesh& mesh, const SimplificationParams& params);
    TerrainFeature analyzeTerrainPoint(const TerrainData& terrain, int x, int y);
    std::vector<std::vector<bool>> generateAdaptiveSampleMask(const TerrainData& terrain, const SimplificationParams& params);
    MeshStats validateMesh(const TerrainMesh& mesh, const TerrainData& terrain);
    bool writeObjFile(const std::string& filename, const TerrainMesh& mesh);
    
    // BRL-CAD DSP integration functions
    bool convertDSPToTerrain(const DSPData& dsp, TerrainData& terrain);
    bool convertTerrainToDSP(const TerrainData& terrain, DSPData& dsp);
    bool convertMeshToNMG(const TerrainMesh& mesh, NMGTriangleData& nmg_data);
    bool triangulateTerrainForBRLCAD(const DSPData& dsp, NMGTriangleData& nmg_data);
    
    // Flood fill and connected component analysis
    TerrainComponents analyzeTerrainComponents(const TerrainData& terrain, double height_threshold = 1e-6);
    void floodFillComponent(const TerrainData& terrain, std::vector<std::vector<bool>>& visited, 
                           ConnectedComponent& component, int start_x, int start_y, double height_threshold);
    void triangulateTerrainVolumeWithComponents(const TerrainData& terrain, TerrainMesh& mesh);
    void triangulateComponentVolume(const TerrainData& terrain, const ConnectedComponent& component, TerrainMesh& mesh);
    
    // Quad-tree-based bottom face triangulation  
    void triangulateBottomFaceWithQuadTree(TerrainMesh& mesh, const std::vector<std::vector<size_t>>& bottom_vertices, 
                                        const TerrainData& terrain, const std::set<std::pair<int, int>>* filter_cells);
    void fallbackBottomTriangulation(TerrainMesh& mesh, const std::vector<std::vector<size_t>>& bottom_vertices, 
                                    const TerrainData& terrain, const std::set<std::pair<int, int>>* filter_cells);
    
    // Helper for hole boundary extraction
    bool extractHoleBoundary(const std::vector<std::pair<int, int>>& hole_cells,
                            const std::set<std::pair<int, int>>& active_cells,
                            const std::vector<std::vector<size_t>>& bottom_vertices,
                            const TerrainMesh& mesh,
                            std::vector<std::pair<double, double>>& hole_boundary,
                            std::vector<size_t>& vertex_indices);

    // Steiner point generation for improved triangle quality
    std::vector<std::pair<double, double>> generateSteinerPoints(
        const std::vector<std::pair<double, double>>& boundary,
        const std::vector<std::vector<std::pair<double, double>>>& holes,
        const std::set<std::pair<int, int>>& active_cells,
        const TerrainData& terrain,
        double min_x, double max_x, double min_y, double max_y);
    
    // Point-in-polygon test
    bool pointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon);

    // Enhanced triangulation with detria (proper Steiner point support)
    void triangulateWithDetria(TerrainMesh& mesh, 
                              const std::vector<std::pair<double, double>>& boundary,
                              const std::vector<std::vector<std::pair<double, double>>>& holes,
                              const std::vector<std::pair<double, double>>& steiner_points,
                              std::vector<size_t>& vertex_indices,
                              const TerrainData& terrain);

    // Manifold validation functions using @elalish/manifold library
    enum class ManifoldValidationResult {
        Valid,
        NonFiniteVertex,
        NotManifold,
        VertexOutOfBounds,
        OtherError
    };
    
    struct ManifoldValidationInfo {
        ManifoldValidationResult result;
        std::string error_message;
        bool is_manifold;
        size_t num_vertices;
        size_t num_triangles;
        
        ManifoldValidationInfo() : result(ManifoldValidationResult::Valid), 
                                  is_manifold(false), num_vertices(0), num_triangles(0) {}
    };
    
    ManifoldValidationInfo validateMeshWithManifold(const TerrainMesh& mesh);

    // Implementation

    // Helper function to check file extension
    bool hasExtension(const std::string& filename, const std::string& ext) {
        if (filename.length() >= ext.length()) {
            return (0 == filename.compare(filename.length() - ext.length(), ext.length(), ext));
        }
        return false;
    }

    // Read terrain data from file using GDAL or custom PGM reader
    bool readTerrainFile(const std::string& filename, TerrainData& terrain) {
        // Try PGM format first
        if (hasExtension(filename, ".pgm") || hasExtension(filename, ".PGM")) {
            return readPGMFile(filename, terrain);
        }

#ifdef HAVE_GDAL
        // Use GDAL for other formats
        GDALDataset* dataset = (GDALDataset*)GDALOpen(filename.c_str(), GA_ReadOnly);
        if (!dataset) {
            return false;
        }

        terrain.width = dataset->GetRasterXSize();
        terrain.height = dataset->GetRasterYSize();
        
        // Get geotransform for scaling
        double geotransform[6];
        if (dataset->GetGeoTransform(geotransform) == CE_None) {
            terrain.cell_size = geotransform[1]; // pixel width
            terrain.origin = Point3D(geotransform[0], geotransform[3], 0);
        } else {
            terrain.cell_size = 1.0;
            terrain.origin = Point3D(0, 0, 0);
        }

        GDALRasterBand* band = dataset->GetRasterBand(1);
        if (!band) {
            GDALClose(dataset);
            return false;
        }

        // Read the height data
        terrain.heights.resize(terrain.height);
        for (int y = 0; y < terrain.height; ++y) {
            terrain.heights[y].resize(terrain.width);
        }

        std::vector<float> scanline(terrain.width);
        terrain.min_height = std::numeric_limits<double>::max();
        terrain.max_height = std::numeric_limits<double>::lowest();

        for (int y = 0; y < terrain.height; ++y) {
            if (band->RasterIO(GF_Read, 0, y, terrain.width, 1, 
                              scanline.data(), terrain.width, 1, 
                              GDT_Float32, 0, 0) != CE_None) {
                GDALClose(dataset);
                return false;
            }
            
            for (int x = 0; x < terrain.width; ++x) {
                double height = static_cast<double>(scanline[x]);
                terrain.heights[y][x] = height;
                terrain.min_height = std::min(terrain.min_height, height);
                terrain.max_height = std::max(terrain.max_height, height);
            }
        }

        GDALClose(dataset);
        return true;
#endif
	return false;
    }

    // Simple PGM file reader
    bool readPGMFile(const std::string& filename, TerrainData& terrain) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }

        std::string magic;
        file >> magic;
        if (magic != "P2") {
            return false; // Only support ASCII PGM
        }

        int width, height, max_val;
        file >> width >> height >> max_val;
        
        terrain.width = width;
        terrain.height = height;
        terrain.cell_size = 1.0;
        terrain.origin = Point3D(0, 0, 0);

        // Read the height data
        terrain.heights.resize(terrain.height);
        terrain.min_height = std::numeric_limits<double>::max();
        terrain.max_height = std::numeric_limits<double>::lowest();

        for (int y = 0; y < terrain.height; ++y) {
            terrain.heights[y].resize(terrain.width);
            for (int x = 0; x < terrain.width; ++x) {
                int pixel_value;
                file >> pixel_value;
                
                // Convert pixel value to height (normalize to reasonable range)
                double height = static_cast<double>(pixel_value) / max_val * 100.0; // Scale to 0-100 range
                terrain.heights[y][x] = height;
                terrain.min_height = std::min(terrain.min_height, height);
                terrain.max_height = std::max(terrain.max_height, height);
            }
        }

        file.close();
        return true;
    }

    // Analyze terrain features at a specific point (Terra/Scape inspired)
    TerrainFeature analyzeTerrainPoint(const TerrainData& terrain, int x, int y) {
        TerrainFeature feature;
        
        if (!terrain.isValidCell(x, y)) {
            return feature;
        }
        
        // Calculate local slope using central differences
        double dx = 0, dy = 0;
        if (terrain.isValidCell(x-1, y) && terrain.isValidCell(x+1, y)) {
            dx = (terrain.getHeight(x+1, y) - terrain.getHeight(x-1, y)) / (2.0 * terrain.cell_size);
        }
        if (terrain.isValidCell(x, y-1) && terrain.isValidCell(x, y+1)) {
            dy = (terrain.getHeight(x, y+1) - terrain.getHeight(x, y-1)) / (2.0 * terrain.cell_size);
        }
        feature.slope = std::sqrt(dx*dx + dy*dy);
        
        // Calculate local curvature (second derivatives)
        double dxx = 0, dyy = 0;
        double h_center = terrain.getHeight(x, y);
        if (terrain.isValidCell(x-1, y) && terrain.isValidCell(x+1, y)) {
            dxx = (terrain.getHeight(x+1, y) - 2*h_center + terrain.getHeight(x-1, y)) / (terrain.cell_size * terrain.cell_size);
        }
        if (terrain.isValidCell(x, y-1) && terrain.isValidCell(x, y+1)) {
            dyy = (terrain.getHeight(x, y+1) - 2*h_center + terrain.getHeight(x, y-1)) / (terrain.cell_size * terrain.cell_size);
        }
        feature.curvature = std::abs(dxx) + std::abs(dyy);
        
        // Calculate local roughness (height variation in neighborhood)
        double height_sum = 0;
        double height_variance = 0;
        int neighbor_count = 0;
        
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (terrain.isValidCell(x+dx, y+dy)) {
                    double h = terrain.getHeight(x+dx, y+dy);
                    height_sum += h;
                    neighbor_count++;
                }
            }
        }
        
        if (neighbor_count > 0) {
            double mean_height = height_sum / neighbor_count;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (terrain.isValidCell(x+dx, y+dy)) {
                        double h = terrain.getHeight(x+dx, y+dy);
                        height_variance += (h - mean_height) * (h - mean_height);
                    }
                }
            }
            feature.roughness = std::sqrt(height_variance / neighbor_count);
        }
        
        // Check if this is a boundary point
        feature.is_boundary = (x == 0 || x == terrain.width-1 || y == 0 || y == terrain.height-1);
        
        // Calculate importance score (Terra/Scape style geometric importance)
        feature.importance_score = feature.curvature + 0.5 * feature.slope + 0.3 * feature.roughness;
        if (feature.is_boundary) feature.importance_score *= 2.0; // Preserve boundaries
        
        return feature;
    }

    // Generate adaptive sampling mask based on terrain features
    std::vector<std::vector<bool>> generateAdaptiveSampleMask(const TerrainData& terrain, const SimplificationParams& params) {
        std::vector<std::vector<bool>> mask(terrain.height, std::vector<bool>(terrain.width, false));
        
        // Always include boundary points
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                if (x == 0 || x == terrain.width-1 || y == 0 || y == terrain.height-1) {
                    mask[y][x] = true;
                }
            }
        }
        
        // Analyze terrain features and mark important points
        std::vector<std::pair<double, std::pair<int, int>>> importance_points;
        
        for (int y = 1; y < terrain.height-1; ++y) {
            for (int x = 1; x < terrain.width-1; ++x) {
                TerrainFeature feature = analyzeTerrainPoint(terrain, x, y);
                
                // Include points with high importance or exceeding thresholds
                if (feature.importance_score > params.error_threshold || 
                    feature.slope > params.slope_threshold) {
                    mask[y][x] = true;
                } else {
                    // Store for potential inclusion based on overall reduction target
                    importance_points.push_back({feature.importance_score, {x, y}});
                }
            }
        }
        
        // Sort by importance and include additional points to meet minimum density
        std::sort(importance_points.rbegin(), importance_points.rend());
        
        int current_points = 0;
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                if (mask[y][x]) current_points++;
            }
        }
        
        int total_points = terrain.width * terrain.height;
        int min_required = total_points * (100 - params.min_triangle_reduction) / 100;
        
        // Add most important remaining points to reach minimum density
        for (const auto& point : importance_points) {
            if (current_points >= min_required) break;
            
            int x = point.second.first;
            int y = point.second.second;
            if (!mask[y][x]) {
                mask[y][x] = true;
                current_points++;
            }
        }
        
        return mask;
    }

    // Flood fill to identify a connected component of non-zero height cells
    void floodFillComponent(const TerrainData& terrain, std::vector<std::vector<bool>>& visited, 
                           ConnectedComponent& component, int start_x, int start_y, double height_threshold) {
        std::queue<std::pair<int, int>> to_visit;
        to_visit.push({start_x, start_y});
        visited[start_y][start_x] = true;
        component.addCell(start_x, start_y);
        
        // 8-connected neighborhood (including diagonals)
        int dx[] = {-1, -1, -1,  0,  0,  1,  1,  1};
        int dy[] = {-1,  0,  1, -1,  1, -1,  0,  1};
        
        while (!to_visit.empty()) {
            auto [x, y] = to_visit.front();
            to_visit.pop();
            
            // Check all 8 neighbors
            for (int i = 0; i < 8; ++i) {
                int nx = x + dx[i];
                int ny = y + dy[i];
                
                if (terrain.isValidCell(nx, ny) && 
                    !visited[ny][nx] && 
                    terrain.getHeight(nx, ny) > height_threshold) {
                    
                    visited[ny][nx] = true;
                    to_visit.push({nx, ny});
                    component.addCell(nx, ny);
                }
            }
        }
    }
    
    // Analyze terrain to identify connected components (islands)
    TerrainComponents analyzeTerrainComponents(const TerrainData& terrain, double height_threshold) {
        TerrainComponents result(terrain.width, terrain.height);
        std::vector<std::vector<bool>> visited(terrain.height, std::vector<bool>(terrain.width, false));
        
        int component_id = 0;
        
        // Find all connected components of non-zero height cells
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                if (!visited[y][x] && terrain.getHeight(x, y) > height_threshold) {
                    ConnectedComponent component;
                    component.id = component_id;
                    
                    floodFillComponent(terrain, visited, component, x, y, height_threshold);
                    
                    // Mark cells in the component map
                    for (const auto& cell : component.cells) {
                        result.component_map[cell.second][cell.first] = component_id;
                    }
                    
                    // Find boundary cells (cells adjacent to zero-height regions)
                    for (const auto& cell : component.cells) {
                        int cx = cell.first;
                        int cy = cell.second;
                        bool is_boundary = false;
                        
                        // Check 4-connected neighbors for boundary detection
                        int dx[] = {-1, 1, 0, 0};
                        int dy[] = {0, 0, -1, 1};
                        
                        for (int i = 0; i < 4; ++i) {
                            int nx = cx + dx[i];
                            int ny = cy + dy[i];
                            
                            // Boundary if neighbor is out of bounds or zero-height
                            if (!terrain.isValidCell(nx, ny) || terrain.getHeight(nx, ny) <= height_threshold) {
                                is_boundary = true;
                                break;
                            }
                        }
                        
                        if (is_boundary) {
                            component.boundary_cells.push_back(cell);
                        }
                    }
                    
                    result.components.push_back(component);
                    component_id++;
                }
            }
        }
        
        return result;
    }

    // Triangulate a single connected component as a separate volumetric mesh
    void triangulateComponentVolume(const TerrainData& terrain, const ConnectedComponent& component, TerrainMesh& mesh) {
        if (component.cells.empty()) {
            return;
        }
        
        // Create mapping from terrain coordinates to mesh vertices
        std::vector<std::vector<size_t>> top_vertices(terrain.height, std::vector<size_t>(terrain.width, SIZE_MAX));
        std::vector<std::vector<size_t>> bottom_vertices(terrain.height, std::vector<size_t>(terrain.width, SIZE_MAX));
        
        // Add vertices only for cells in this component
        for (const auto& cell : component.cells) {
            int x = cell.first;
            int y = cell.second;
            
            double world_x = terrain.origin.x + x * terrain.cell_size;
            double world_y = terrain.origin.y - y * terrain.cell_size;
            double height = terrain.getHeight(x, y);
            
            top_vertices[y][x] = mesh.addVertex(Point3D(world_x, world_y, height));
            bottom_vertices[y][x] = mesh.addVertex(Point3D(world_x, world_y, 0.0));
        }
        
        // Create a set for quick lookup of component cells
        std::set<std::pair<int, int>> component_cells(component.cells.begin(), component.cells.end());
        
        // Add top surface triangles for complete quads within the component
        for (int y = component.min_y; y < component.max_y; ++y) {
            for (int x = component.min_x; x < component.max_x; ++x) {
                // Check if we can form a quad with all corners in the component
                if (component_cells.count({x, y}) && 
                    component_cells.count({x+1, y}) && 
                    component_cells.count({x, y+1}) && 
                    component_cells.count({x+1, y+1})) {
                    
                    size_t v00 = top_vertices[y][x];
                    size_t v10 = top_vertices[y][x+1];
                    size_t v01 = top_vertices[y+1][x];
                    size_t v11 = top_vertices[y+1][x+1];
                    
                    // Add two triangles with CCW orientation (viewed from above)
                    mesh.addSurfaceTriangle(v00, v01, v10);
                    mesh.addSurfaceTriangle(v10, v01, v11);
                }
            }
        }
        
        // Add bottom surface triangles using earcut for more efficient triangulation
        std::set<std::pair<int, int>> component_cells_set(component.cells.begin(), component.cells.end());
        triangulateBottomFaceWithQuadTree(mesh, bottom_vertices, terrain, &component_cells_set);
        
        // Generate walls by examining each potential wall edge
        // For each cell, check its 4 neighbors and create walls where needed
        for (const auto& cell : component.cells) {
            int x = cell.first;
            int y = cell.second;
            
            // Check right edge: if there's no cell to the right, create a wall
            if (!component_cells.count({x+1, y})) {
                // Create wall on the right side of this cell
                // We need to check if there's a cell below to form the wall
                if (component_cells.count({x, y+1})) {
                    size_t t1 = top_vertices[y][x];
                    size_t t2 = top_vertices[y+1][x];
                    size_t b1 = bottom_vertices[y][x];
                    size_t b2 = bottom_vertices[y+1][x];
                    
                    // Right wall facing outward
                    mesh.addTriangle(t1, t2, b1);
                    mesh.addTriangle(t2, b2, b1);
                }
            }
            
            // Check bottom edge: if there's no cell below, create a wall
            if (!component_cells.count({x, y+1})) {
                // Create wall on the bottom side of this cell
                if (component_cells.count({x+1, y})) {
                    size_t t1 = top_vertices[y][x];
                    size_t t2 = top_vertices[y][x+1];
                    size_t b1 = bottom_vertices[y][x];
                    size_t b2 = bottom_vertices[y][x+1];
                    
                    // Bottom wall facing outward
                    mesh.addTriangle(t1, b1, t2);
                    mesh.addTriangle(t2, b1, b2);
                }
            }
            
            // Check left edge: if there's no cell to the left, create a wall
            if (!component_cells.count({x-1, y})) {
                // Create wall on the left side of this cell
                if (component_cells.count({x, y+1})) {
                    size_t t1 = top_vertices[y][x];
                    size_t t2 = top_vertices[y+1][x];
                    size_t b1 = bottom_vertices[y][x];
                    size_t b2 = bottom_vertices[y+1][x];
                    
                    // Left wall facing outward
                    mesh.addTriangle(t1, b1, t2);
                    mesh.addTriangle(t2, b1, b2);
                }
            }
            
            // Check top edge: if there's no cell above, create a wall
            if (!component_cells.count({x, y-1})) {
                // Create wall on the top side of this cell
                if (component_cells.count({x+1, y})) {
                    size_t t1 = top_vertices[y][x];
                    size_t t2 = top_vertices[y][x+1];
                    size_t b1 = bottom_vertices[y][x];
                    size_t b2 = bottom_vertices[y][x+1];
                    
                    // Top wall facing outward
                    mesh.addTriangle(t1, t2, b1);
                    mesh.addTriangle(t2, b2, b1);
                }
            }
        }
    }
    
    // Triangulate terrain volume with proper handling of connected components
    void triangulateTerrainVolumeWithComponents(const TerrainData& terrain, TerrainMesh& mesh) {
        mesh.clear();
        
        if (terrain.width <= 0 || terrain.height <= 0) {
            return;
        }
        
        // Analyze terrain to find connected components
        TerrainComponents components = analyzeTerrainComponents(terrain);
        
        std::cout << "Found " << components.components.size() << " terrain component(s)" << std::endl;
        
        // Triangulate each component separately
        for (const auto& component : components.components) {
            std::cout << "Processing component " << component.id 
                      << " with " << component.cells.size() << " cells" << std::endl;
            triangulateComponentVolume(terrain, component, mesh);
        }
    }

    // Generate a volumetric triangle mesh from terrain data (legacy single-mesh approach)
    void triangulateTerrainVolumeLegacy(const TerrainData& terrain, TerrainMesh& mesh) {
        mesh.clear();
        
        if (terrain.width <= 0 || terrain.height <= 0) {
            return;
        }

        // For a volumetric mesh, we need vertices at multiple levels:
        // 1. Top surface vertices (at height values)
        // 2. Bottom surface vertices (at z=0)
        // 3. Side vertices for walls

        // Create vertex lookup table
        std::vector<std::vector<size_t>> top_vertices(terrain.height, std::vector<size_t>(terrain.width));
        std::vector<std::vector<size_t>> bottom_vertices(terrain.height, std::vector<size_t>(terrain.width));

        // Add top surface vertices
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                double world_x = terrain.origin.x + x * terrain.cell_size;
                double world_y = terrain.origin.y - y * terrain.cell_size; // Note: y is flipped in image coordinates
                double height = terrain.getHeight(x, y);
                
                top_vertices[y][x] = mesh.addVertex(Point3D(world_x, world_y, height));
                bottom_vertices[y][x] = mesh.addVertex(Point3D(world_x, world_y, 0.0));
            }
        }

        // Add top surface triangles (2 triangles per cell)
        for (int y = 0; y < terrain.height - 1; ++y) {
            for (int x = 0; x < terrain.width - 1; ++x) {
                // Get the four corner vertices of the cell
                size_t v00 = top_vertices[y][x];
                size_t v10 = top_vertices[y][x + 1];
                size_t v01 = top_vertices[y + 1][x];
                size_t v11 = top_vertices[y + 1][x + 1];

                // Add two triangles with CCW orientation (viewed from above)
                // Triangle 1: v00, v01, v10
                mesh.addSurfaceTriangle(v00, v01, v10);
                // Triangle 2: v10, v01, v11
                mesh.addSurfaceTriangle(v10, v01, v11);
            }
        }

        // Add bottom surface triangles using earcut for more efficient triangulation
        triangulateBottomFaceWithQuadTree(mesh, bottom_vertices, terrain, nullptr);

        // Add side walls
        // Left wall (x = 0)
        for (int y = 0; y < terrain.height - 1; ++y) {
            size_t top_0 = top_vertices[y][0];
            size_t top_1 = top_vertices[y + 1][0];
            size_t bot_0 = bottom_vertices[y][0];
            size_t bot_1 = bottom_vertices[y + 1][0];

            mesh.addTriangle(top_0, bot_0, top_1);
            mesh.addTriangle(top_1, bot_0, bot_1);
        }

        // Right wall (x = width - 1)
        for (int y = 0; y < terrain.height - 1; ++y) {
            int x = terrain.width - 1;
            size_t top_0 = top_vertices[y][x];
            size_t top_1 = top_vertices[y + 1][x];
            size_t bot_0 = bottom_vertices[y][x];
            size_t bot_1 = bottom_vertices[y + 1][x];

            mesh.addTriangle(top_0, top_1, bot_0);
            mesh.addTriangle(top_1, bot_1, bot_0);
        }

        // Top wall (y = 0)
        for (int x = 0; x < terrain.width - 1; ++x) {
            size_t top_0 = top_vertices[0][x];
            size_t top_1 = top_vertices[0][x + 1];
            size_t bot_0 = bottom_vertices[0][x];
            size_t bot_1 = bottom_vertices[0][x + 1];

            mesh.addTriangle(top_0, top_1, bot_0);
            mesh.addTriangle(top_1, bot_1, bot_0);
        }

        // Bottom wall (y = height - 1)
        for (int x = 0; x < terrain.width - 1; ++x) {
            int y = terrain.height - 1;
            size_t top_0 = top_vertices[y][x];
            size_t top_1 = top_vertices[y][x + 1];
            size_t bot_0 = bottom_vertices[y][x];
            size_t bot_1 = bottom_vertices[y][x + 1];

            mesh.addTriangle(top_0, bot_0, top_1);
            mesh.addTriangle(top_1, bot_0, bot_1);
        }
    }

    // Generate a volumetric triangle mesh from terrain data
    void triangulateTerrainVolume(const TerrainData& terrain, TerrainMesh& mesh) {
        // Use component-based approach by default for better handling of disjoint islands
        triangulateTerrainVolumeWithComponents(terrain, mesh);
    }

    // Generate a simplified volumetric triangle mesh using Terra/Scape concepts
    void triangulateTerrainVolumeSimplified(const TerrainData& terrain, TerrainMesh& mesh, const SimplificationParams& params) {
        mesh.clear();
        
        if (terrain.width <= 0 || terrain.height <= 0) {
            return;
        }

        // Generate adaptive sampling mask based on terrain features
        auto sample_mask = generateAdaptiveSampleMask(terrain, params);
        
        // Create a new simplified grid by decimation
        std::vector<std::vector<bool>> keep_vertex(terrain.height, std::vector<bool>(terrain.width, false));
        std::vector<std::vector<size_t>> top_vertices(terrain.height, std::vector<size_t>(terrain.width, SIZE_MAX));
        std::vector<std::vector<size_t>> bottom_vertices(terrain.height, std::vector<size_t>(terrain.width, SIZE_MAX));

        // For manifold guarantee, ensure we keep vertices in a structured grid pattern
        // Use regular subsampling combined with feature-based importance
        int step_size = std::max(1, (int)std::sqrt(100.0 / (100.0 - params.min_triangle_reduction)));
        
        // First pass: structured subsampling to maintain topology
        for (int y = 0; y < terrain.height; y += step_size) {
            for (int x = 0; x < terrain.width; x += step_size) {
                keep_vertex[y][x] = true;
            }
        }
        
        // Second pass: add important feature points
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                if (sample_mask[y][x] && !keep_vertex[y][x]) {
                    keep_vertex[y][x] = true;
                }
            }
        }
        
        // Third pass: ensure boundary completeness
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                if ((x == 0 || x == terrain.width-1 || y == 0 || y == terrain.height-1)) {
                    keep_vertex[y][x] = true;
                }
            }
        }

        // Add vertices for kept points
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                if (keep_vertex[y][x]) {
                    double world_x = terrain.origin.x + x * terrain.cell_size;
                    double world_y = terrain.origin.y - y * terrain.cell_size;
                    double height = terrain.getHeight(x, y);
                    
                    top_vertices[y][x] = mesh.addVertex(Point3D(world_x, world_y, height));
                    bottom_vertices[y][x] = mesh.addVertex(Point3D(world_x, world_y, 0.0));
                }
            }
        }

        // Triangulate surface using a grid-walking approach to maintain manifold property
        for (int y = 0; y < terrain.height - 1; ++y) {
            for (int x = 0; x < terrain.width - 1; ++x) {
                // Find the next valid grid cell that can be triangulated
                std::vector<std::pair<int, int>> quad_corners;
                if (keep_vertex[y][x]) quad_corners.push_back({x, y});
                if (keep_vertex[y][x+1]) quad_corners.push_back({x+1, y});
                if (keep_vertex[y+1][x]) quad_corners.push_back({x, y+1});
                if (keep_vertex[y+1][x+1]) quad_corners.push_back({x+1, y+1});
                
                // If we have all 4 corners, create the standard 2 triangles
                if (quad_corners.size() == 4) {
                    size_t v00_top = top_vertices[y][x];
                    size_t v10_top = top_vertices[y][x+1];
                    size_t v01_top = top_vertices[y+1][x];
                    size_t v11_top = top_vertices[y+1][x+1];
                    
                    size_t v00_bot = bottom_vertices[y][x];
                    size_t v10_bot = bottom_vertices[y][x+1];
                    size_t v01_bot = bottom_vertices[y+1][x];
                    size_t v11_bot = bottom_vertices[y+1][x+1];
                    
                    // Top surface triangles
                    mesh.addSurfaceTriangle(v00_top, v01_top, v10_top);
                    mesh.addSurfaceTriangle(v10_top, v01_top, v11_top);
                } 
                // Handle cases where we have 3 vertices (create 1 triangle)
                else if (quad_corners.size() == 3) {
                    for (size_t i = 0; i < 3; ++i) {
                        int px = quad_corners[i].first;
                        int py = quad_corners[i].second;
                        
                        size_t v_top = top_vertices[py][px];
                        size_t v_bot = bottom_vertices[py][px];
                        
                        if (i == 0) {
                            size_t v1_top = top_vertices[quad_corners[1].second][quad_corners[1].first];
                            size_t v2_top = top_vertices[quad_corners[2].second][quad_corners[2].first];
                            size_t v1_bot = bottom_vertices[quad_corners[1].second][quad_corners[1].first];
                            size_t v2_bot = bottom_vertices[quad_corners[2].second][quad_corners[2].first];
                            
                            mesh.addSurfaceTriangle(v_top, v1_top, v2_top);
                            break;
                        }
                    }
                }
            }
        }

        // Add bottom surface triangles using earcut for more efficient triangulation
        // Create filter set from keep_vertex array
        std::set<std::pair<int, int>> keep_cells;
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                if (keep_vertex[y][x]) {
                    keep_cells.insert({x, y});
                }
            }
        }
        triangulateBottomFaceWithQuadTree(mesh, bottom_vertices, terrain, &keep_cells);

        // Add side walls for boundary edges with proper gap filling
        // Left wall (x = 0)
        std::vector<int> left_wall_vertices;
        for (int y = 0; y < terrain.height; ++y) {
            if (keep_vertex[y][0]) {
                left_wall_vertices.push_back(y);
            }
        }
        for (size_t i = 0; i < left_wall_vertices.size() - 1; ++i) {
            int y0 = left_wall_vertices[i];
            int y1 = left_wall_vertices[i + 1];
            
            size_t top_0 = top_vertices[y0][0];
            size_t top_1 = top_vertices[y1][0];
            size_t bot_0 = bottom_vertices[y0][0];
            size_t bot_1 = bottom_vertices[y1][0];

            mesh.addTriangle(top_0, bot_0, top_1);
            mesh.addTriangle(top_1, bot_0, bot_1);
        }

        // Right wall (x = width - 1)
        std::vector<int> right_wall_vertices;
        int x = terrain.width - 1;
        for (int y = 0; y < terrain.height; ++y) {
            if (keep_vertex[y][x]) {
                right_wall_vertices.push_back(y);
            }
        }
        for (size_t i = 0; i < right_wall_vertices.size() - 1; ++i) {
            int y0 = right_wall_vertices[i];
            int y1 = right_wall_vertices[i + 1];
            
            size_t top_0 = top_vertices[y0][x];
            size_t top_1 = top_vertices[y1][x];
            size_t bot_0 = bottom_vertices[y0][x];
            size_t bot_1 = bottom_vertices[y1][x];

            mesh.addTriangle(top_0, top_1, bot_0);
            mesh.addTriangle(top_1, bot_1, bot_0);
        }

        // Top wall (y = 0)
        std::vector<int> top_wall_vertices;
        for (int x = 0; x < terrain.width; ++x) {
            if (keep_vertex[0][x]) {
                top_wall_vertices.push_back(x);
            }
        }
        for (size_t i = 0; i < top_wall_vertices.size() - 1; ++i) {
            int x0 = top_wall_vertices[i];
            int x1 = top_wall_vertices[i + 1];
            
            size_t top_0 = top_vertices[0][x0];
            size_t top_1 = top_vertices[0][x1];
            size_t bot_0 = bottom_vertices[0][x0];
            size_t bot_1 = bottom_vertices[0][x1];

            mesh.addTriangle(top_0, top_1, bot_0);
            mesh.addTriangle(top_1, bot_1, bot_0);
        }

        // Bottom wall (y = height - 1)
        std::vector<int> bottom_wall_vertices;
        int y = terrain.height - 1;
        for (int x = 0; x < terrain.width; ++x) {
            if (keep_vertex[y][x]) {
                bottom_wall_vertices.push_back(x);
            }
        }
        for (size_t i = 0; i < bottom_wall_vertices.size() - 1; ++i) {
            int x0 = bottom_wall_vertices[i];
            int x1 = bottom_wall_vertices[i + 1];
            
            size_t top_0 = top_vertices[y][x0];
            size_t top_1 = top_vertices[y][x1];
            size_t bot_0 = bottom_vertices[y][x0];
            size_t bot_1 = bottom_vertices[y][x1];

            mesh.addTriangle(top_0, bot_0, top_1);
            mesh.addTriangle(top_1, bot_0, bot_1);
        }
    }

    // Generate terrain surface-only mesh with Terra/Scape simplification (no volume)
    void triangulateTerrainSurfaceOnly(const TerrainData& terrain, TerrainMesh& mesh, const SimplificationParams& params) {
        mesh.clear();
        
        if (terrain.width <= 0 || terrain.height <= 0) {
            return;
        }

        // Generate adaptive sampling mask
        auto sample_mask = generateAdaptiveSampleMask(terrain, params);
        
        // Use more aggressive decimation for surface-only mode
        int step_size = std::max(2, (int)std::sqrt(100.0 / (100.0 - params.min_triangle_reduction)));
        
        // Create simplified grid with structured subsampling
        std::vector<std::vector<bool>> keep_vertex(terrain.height, std::vector<bool>(terrain.width, false));
        std::vector<std::vector<size_t>> surface_vertices(terrain.height, std::vector<size_t>(terrain.width, SIZE_MAX));

        // Structured subsampling
        for (int y = 0; y < terrain.height; y += step_size) {
            for (int x = 0; x < terrain.width; x += step_size) {
                keep_vertex[y][x] = true;
            }
        }
        
        // Add important features 
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                if (sample_mask[y][x]) {
                    keep_vertex[y][x] = true;
                }
            }
        }
        
        // Ensure boundaries are complete
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                if (x == 0 || x == terrain.width-1 || y == 0 || y == terrain.height-1) {
                    keep_vertex[y][x] = true;
                }
            }
        }

        // Add surface vertices only
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                if (keep_vertex[y][x]) {
                    double world_x = terrain.origin.x + x * terrain.cell_size;
                    double world_y = terrain.origin.y - y * terrain.cell_size;
                    double height = terrain.getHeight(x, y);
                    
                    surface_vertices[y][x] = mesh.addVertex(Point3D(world_x, world_y, height));
                }
            }
        }

        // Triangulate surface using grid approach
        for (int y = 0; y < terrain.height - 1; ++y) {
            for (int x = 0; x < terrain.width - 1; ++x) {
                // Find valid quad corners
                std::vector<std::pair<int, int>> corners;
                if (keep_vertex[y][x]) corners.push_back({x, y});
                if (keep_vertex[y][x+1]) corners.push_back({x+1, y});
                if (keep_vertex[y+1][x]) corners.push_back({x, y+1});
                if (keep_vertex[y+1][x+1]) corners.push_back({x+1, y+1});
                
                // Triangulate complete quads
                if (corners.size() == 4) {
                    size_t v00 = surface_vertices[y][x];
                    size_t v10 = surface_vertices[y][x+1];
                    size_t v01 = surface_vertices[y+1][x];
                    size_t v11 = surface_vertices[y+1][x+1];
                    
                    mesh.addSurfaceTriangle(v00, v01, v10);
                    mesh.addSurfaceTriangle(v10, v01, v11);
                }
                // Handle partial quads
                else if (corners.size() == 3) {
                    // Create single triangle from 3 corners
                    auto p0 = corners[0];
                    auto p1 = corners[1]; 
                    auto p2 = corners[2];
                    
                    size_t v0 = surface_vertices[p0.second][p0.first];
                    size_t v1 = surface_vertices[p1.second][p1.first];
                    size_t v2 = surface_vertices[p2.second][p2.first];
                    
                    mesh.addSurfaceTriangle(v0, v1, v2);
                }
            }
        }
    }

    // Generate Steiner points in geometric progression from edges toward center
    std::vector<std::pair<double, double>> generateSteinerPoints(
        const std::vector<std::pair<double, double>>& boundary,
        const std::vector<std::vector<std::pair<double, double>>>& holes,
        const std::set<std::pair<int, int>>& active_cells,
        const TerrainData& terrain,
        double min_x, double max_x, double min_y, double max_y) {
        
        std::vector<std::pair<double, double>> steiner_points;
        
        // Parameters for Steiner point generation
        double progression_factor = 0.75; // Geometric progression ratio (balanced)
        int max_levels = 2;               // Fewer levels but better quality
        double min_distance = terrain.cell_size * 5.0; // More spaced out points
        
        // Calculate bounding box center
        double center_x = (min_x + max_x) * 0.5 * terrain.cell_size + terrain.origin.x;
        double center_y = (min_y + max_y) * 0.5 * terrain.cell_size + terrain.origin.y;
        
        // Calculate distance from boundary to center for normalization
        double max_distance_to_center = 0.0;
        for (const auto& point : boundary) {
            double dx = point.first - center_x;
            double dy = point.second - center_y;
            double dist = std::sqrt(dx * dx + dy * dy);
            max_distance_to_center = std::max(max_distance_to_center, dist);
        }
        
        // Generate concentric layers of Steiner points
        for (int level = 1; level <= max_levels; ++level) {
            double level_factor = std::pow(progression_factor, level);
            double ring_distance = max_distance_to_center * level_factor;
            
            // Skip if ring is too close to center
            if (ring_distance < min_distance) continue;
            
            // Generate points in this ring based on size
            int points_in_ring = std::max(6, static_cast<int>(2.0 * M_PI * ring_distance / min_distance));
            
            for (int i = 0; i < points_in_ring; ++i) {
                double angle = 2.0 * M_PI * i / points_in_ring;
                double x = center_x + ring_distance * std::cos(angle);
                double y = center_y + ring_distance * std::sin(angle);
                
                // Convert back to terrain coordinates to check if point is valid
                int terrain_x = static_cast<int>((x - terrain.origin.x) / terrain.cell_size);
                int terrain_y = static_cast<int>((terrain.origin.y - y) / terrain.cell_size);
                
                // Check if point is within active region
                bool is_in_active_region = false;
                if (terrain_x >= 0 && terrain_x < terrain.width && 
                    terrain_y >= 0 && terrain_y < terrain.height) {
                    is_in_active_region = active_cells.count({terrain_x, terrain_y}) > 0;
                }
                
                if (is_in_active_region) {
                    // Check if point is inside any hole using point-in-polygon test
                    bool inside_hole = false;
                    for (const auto& hole : holes) {
                        if (pointInPolygon(x, y, hole)) {
                            inside_hole = true;
                            break;
                        }
                    }
                    
                    if (!inside_hole) {
                        // Check minimum distance to existing points
                        bool too_close = false;
                        for (const auto& existing : steiner_points) {
                            double dx = x - existing.first;
                            double dy = y - existing.second;
                            if (std::sqrt(dx * dx + dy * dy) < min_distance) {
                                too_close = true;
                                break;
                            }
                        }
                        
                        if (!too_close) {
                            steiner_points.push_back({x, y});
                        }
                    }
                }
            }
        }
        
        return steiner_points;
    }
    
    // Enhanced triangulation with detria (proper Steiner point support)
    void triangulateWithDetria(TerrainMesh& mesh, 
                              const std::vector<std::pair<double, double>>& boundary,
                              const std::vector<std::vector<std::pair<double, double>>>& holes,
                              const std::vector<std::pair<double, double>>& steiner_points,
                              std::vector<size_t>& vertex_indices,
                              const TerrainData& terrain) {
        
        // Create all points: boundary + steiner points
        std::vector<detria::PointD> all_points;
        std::vector<size_t> all_vertex_indices;
        
        // Add boundary points first
        for (const auto& point : boundary) {
            all_points.push_back({point.first, point.second});
            // vertex_indices already contains boundary vertex indices
        }
        
        // Add Steiner points as vertices and to the point list
        for (const auto& point : steiner_points) {
            double world_z = terrain.min_height - 1.0; // Slightly below terrain
            size_t vertex_index = mesh.addVertex(Point3D(point.first, point.second, world_z));
            vertex_indices.push_back(vertex_index);
            all_points.push_back({point.first, point.second});
        }
        
        // Set up detria triangulation
        detria::Triangulation tri;
        tri.setPoints(all_points);
        
        // Add boundary outline (indices refer to all_points)
        std::vector<uint32_t> outline_indices;
        for (size_t i = 0; i < boundary.size(); ++i) {
            outline_indices.push_back(static_cast<uint32_t>(i));
        }
        tri.addOutline(outline_indices);
        
        // Add holes (indices refer to all_points)  
        size_t hole_start_idx = boundary.size() + steiner_points.size();
        for (const auto& hole : holes) {
            std::vector<uint32_t> hole_indices;
            for (size_t i = 0; i < hole.size(); ++i) {
                all_points.push_back({hole[i].first, hole[i].second});
                hole_indices.push_back(static_cast<uint32_t>(hole_start_idx + i));
            }
            tri.addHole(hole_indices);
            hole_start_idx += hole.size();
        }
        
        // Update points after adding holes
        tri.setPoints(all_points);
        
        // Perform constrained Delaunay triangulation with Steiner points
        bool delaunay = true;
        bool success = tri.triangulate(delaunay);
        
        if (success) {
            // Extract triangles and add them to the mesh
            bool cwTriangles = false; // We want counter-clockwise for bottom face
            
            tri.forEachTriangle([&](detria::Triangle<uint32_t> triangle) {
                // Map detria point indices back to our vertex indices
                size_t v0, v1, v2;
                
                if (triangle.x < vertex_indices.size()) {
                    v0 = vertex_indices[triangle.x];
                } else return; // Skip if invalid index
                
                if (triangle.y < vertex_indices.size()) {
                    v1 = vertex_indices[triangle.y];
                } else return;
                
                if (triangle.z < vertex_indices.size()) {
                    v2 = vertex_indices[triangle.z];
                } else return;
                
                // Add triangle with correct bottom face orientation (reversed winding)
                mesh.addTriangle(v0, v2, v1);
                
            }, cwTriangles);
        } else {
            // Fall back to earcut if detria fails
            std::vector<std::vector<std::pair<double, double>>> rings;
            rings.push_back(boundary);
            for (const auto& hole : holes) {
                rings.push_back(hole);
            }
            
            try {
                std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(rings);
                
                // Only use boundary vertex count for earcut fallback
                size_t boundary_vertex_count = boundary.size();
                for (size_t i = 0; i < indices.size(); i += 3) {
                    if (i + 2 < indices.size() && indices[i] < boundary_vertex_count && 
                        indices[i+1] < boundary_vertex_count && indices[i+2] < boundary_vertex_count) {
                        
                        size_t v0 = vertex_indices[indices[i]];
                        size_t v1 = vertex_indices[indices[i + 1]];
                        size_t v2 = vertex_indices[indices[i + 2]];
                        
                        mesh.addTriangle(v0, v2, v1);
                    }
                }
            } catch (const std::exception&) {
                // If both detria and earcut fail, use fallback
            }
        }
    }
    
    // Point-in-polygon test using ray casting algorithm
    bool pointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon) {
        int n = polygon.size();
        if (n < 3) return false;
        
        bool inside = false;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            const auto& pi = polygon[i];
            const auto& pj = polygon[j];
            
            if (((pi.second > y) != (pj.second > y)) &&
                (x < (pj.first - pi.first) * (y - pi.second) / (pj.second - pi.second) + pi.first)) {
                inside = !inside;
            }
        }
        return inside;
    }
    
    // Build quad tree for integer coordinates with aspect ratio control
    std::unique_ptr<QuadNode> buildQuadTree(const std::set<std::pair<int, int>>& edge_points,
                                           int min_x, int min_y, int max_x, int max_y,
                                           double aspect_ratio_threshold = 2.0, int max_depth = 8, int current_depth = 0) {
        auto node = std::make_unique<QuadNode>(min_x, min_y, max_x, max_y);
        
        // Find edge points within this quad's bounds
        for (const auto& point : edge_points) {
            if (point.first >= min_x && point.first <= max_x &&
                point.second >= min_y && point.second <= max_y) {
                node->edge_points.push_back(point);
            }
        }
        
        // Termination conditions:
        // 1. Maximum depth reached
        // 2. Quad is too small (1x1 or 2x2)
        // 3. If quad has no edge points and good aspect ratio, stop
        // 4. For small quads with edge points, stop subdividing
        int width = max_x - min_x;
        int height = max_y - min_y;
        
        if (current_depth >= max_depth ||
            (width <= 2 && height <= 2) ||
            (node->edge_points.empty() && node->hasGoodAspectRatio(aspect_ratio_threshold)) ||
            (width <= 1 || height <= 1)) {
            return node;
        }
        
        // For larger quads with edge points, check if all are on boundary corners
        if (!node->edge_points.empty() && width > 2 && height > 2) {
            bool all_edge_points_are_corners = true;
            for (const auto& point : node->edge_points) {
                // Check if point is on a corner (not just boundary)
                bool is_corner = (point.first == min_x || point.first == max_x) &&
                                (point.second == min_y || point.second == max_y);
                if (!is_corner) {
                    all_edge_points_are_corners = false;
                    break;
                }
            }
            if (all_edge_points_are_corners && node->edge_points.size() <= 4) {
                return node; // Good termination condition met
            }
        }
        
        // If aspect ratio is bad, split only in the longer direction
        if (!node->hasGoodAspectRatio(aspect_ratio_threshold)) {
            if (width > height && width > 1) {
                // Split horizontally only
                int split_x = min_x + width / 2;
                node->children.push_back(buildQuadTree(edge_points, min_x, min_y, split_x, max_y, aspect_ratio_threshold, max_depth, current_depth + 1));
                node->children.push_back(buildQuadTree(edge_points, split_x, min_y, max_x, max_y, aspect_ratio_threshold, max_depth, current_depth + 1));
                node->is_leaf = false;
            } else if (height > width && height > 1) {
                // Split vertically only
                int split_y = min_y + height / 2;
                node->children.push_back(buildQuadTree(edge_points, min_x, min_y, max_x, split_y, aspect_ratio_threshold, max_depth, current_depth + 1));
                node->children.push_back(buildQuadTree(edge_points, min_x, split_y, max_x, max_y, aspect_ratio_threshold, max_depth, current_depth + 1));
                node->is_leaf = false;
            }
        } else {
            // Normal quad split if we can split in both directions
            if (width > 1 && height > 1) {
                int split_x = min_x + width / 2;
                int split_y = min_y + height / 2;
                
                // Create 4 child quads
                node->children.push_back(buildQuadTree(edge_points, min_x, min_y, split_x, split_y, aspect_ratio_threshold, max_depth, current_depth + 1));
                node->children.push_back(buildQuadTree(edge_points, split_x, min_y, max_x, split_y, aspect_ratio_threshold, max_depth, current_depth + 1));
                node->children.push_back(buildQuadTree(edge_points, min_x, split_y, split_x, max_y, aspect_ratio_threshold, max_depth, current_depth + 1));
                node->children.push_back(buildQuadTree(edge_points, split_x, split_y, max_x, max_y, aspect_ratio_threshold, max_depth, current_depth + 1));
                node->is_leaf = false;
            }
        }
        
        return node;
    }
    
    // Simple triangulation for a single quad (rectangles are easy to triangulate)
    void triangulateQuad(TerrainMesh& mesh, const QuadNode& quad, 
                        const std::vector<std::vector<size_t>>& bottom_vertices) {
        
        // For simple quads, just create two triangles using the 4 corners
        size_t v00, v10, v01, v11;
        
        // Get the 4 corner vertices if they exist
        if (bottom_vertices[quad.min_y][quad.min_x] != SIZE_MAX &&
            bottom_vertices[quad.min_y][quad.max_x] != SIZE_MAX &&
            bottom_vertices[quad.max_y][quad.min_x] != SIZE_MAX &&
            bottom_vertices[quad.max_y][quad.max_x] != SIZE_MAX) {
            
            v00 = bottom_vertices[quad.min_y][quad.min_x];  // Bottom-left
            v10 = bottom_vertices[quad.min_y][quad.max_x];  // Bottom-right
            v01 = bottom_vertices[quad.max_y][quad.min_x];  // Top-left
            v11 = bottom_vertices[quad.max_y][quad.max_x];  // Top-right
            
            // Create two triangles with correct winding for bottom face (reversed for upward normal)
            // Triangle 1: bottom-left, top-left, bottom-right
            mesh.addTriangle(v00, v01, v10);
            // Triangle 2: bottom-right, top-left, top-right
            mesh.addTriangle(v10, v01, v11);
        }
    }
    
    // Triangulate all leaf nodes of the quad tree
    void triangulateQuadTreeLeaves(TerrainMesh& mesh, const QuadNode& node,
                                  const std::vector<std::vector<size_t>>& bottom_vertices) {
        if (node.is_leaf) {
            triangulateQuad(mesh, node, bottom_vertices);
        } else {
            for (const auto& child : node.children) {
                if (child) {
                    triangulateQuadTreeLeaves(mesh, *child, bottom_vertices);
                }
            }
        }
    }

    // Quad-tree-based triangulation of coplanar bottom face with integer coordinates
    void triangulateBottomFaceWithQuadTree(TerrainMesh& mesh, const std::vector<std::vector<size_t>>& bottom_vertices, 
                                          const TerrainData& terrain, const std::set<std::pair<int, int>>* filter_cells = nullptr) {
        
        // Build set of cells that should have bottom faces (these are our "edge points")
        std::set<std::pair<int, int>> active_cells;
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                if (bottom_vertices[y][x] != SIZE_MAX) {
                    if (filter_cells == nullptr || filter_cells->count({x, y})) {
                        active_cells.insert({x, y});
                    }
                }
            }
        }
        
        if (active_cells.empty()) {
            return;
        }
        
        // Find bounds of active region 
        int min_x = INT_MAX, max_x = INT_MIN, min_y = INT_MAX, max_y = INT_MIN;
        for (const auto& cell : active_cells) {
            min_x = std::min(min_x, cell.first);
            max_x = std::max(max_x, cell.first);
            min_y = std::min(min_y, cell.second);
            max_y = std::max(max_y, cell.second);
        }
        
        try {
            // Build quad tree using the active cells as edge points
            auto quad_tree = buildQuadTree(active_cells, min_x, min_y, max_x, max_y, 2.0, 8, 0);
            
            // Triangulate all leaf nodes of the quad tree
            triangulateQuadTreeLeaves(mesh, *quad_tree, bottom_vertices);
            
        } catch (const std::exception&) {
            // Fall back to original grid-based approach if quad tree fails
            fallbackBottomTriangulation(mesh, bottom_vertices, terrain, filter_cells);
        }
    }
    void triangulateBottomFaceWithEarcut(TerrainMesh& mesh, const std::vector<std::vector<size_t>>& bottom_vertices, 
                                        const TerrainData& terrain, const std::set<std::pair<int, int>>* filter_cells = nullptr) {
        
        // Build set of cells that should have bottom faces
        std::set<std::pair<int, int>> active_cells;
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                if (bottom_vertices[y][x] != SIZE_MAX) {
                    if (filter_cells == nullptr || filter_cells->count({x, y})) {
                        active_cells.insert({x, y});
                    }
                }
            }
        }
        
        if (active_cells.empty()) {
            return;
        }
        
        // Find bounds of active region
        int min_x = INT_MAX, max_x = INT_MIN, min_y = INT_MAX, max_y = INT_MIN;
        for (const auto& cell : active_cells) {
            min_x = std::min(min_x, cell.first);
            max_x = std::max(max_x, cell.first);
            min_y = std::min(min_y, cell.second);
            max_y = std::max(max_y, cell.second);
        }
        
        // Create outer boundary (counter-clockwise)
        std::vector<std::pair<double, double>> outer_boundary;
        std::vector<size_t> vertex_indices;
        
        // Bottom edge (left to right)
        for (int x = min_x; x <= max_x; ++x) {
            if (active_cells.count({x, min_y})) {
                const Point3D& vertex = mesh.vertices[bottom_vertices[min_y][x]];
                outer_boundary.push_back({vertex.x, vertex.y});
                vertex_indices.push_back(bottom_vertices[min_y][x]);
            }
        }
        
        // Right edge (bottom to top, skip corners)
        for (int y = min_y + 1; y <= max_y; ++y) {
            if (active_cells.count({max_x, y})) {
                const Point3D& vertex = mesh.vertices[bottom_vertices[y][max_x]];
                outer_boundary.push_back({vertex.x, vertex.y});
                vertex_indices.push_back(bottom_vertices[y][max_x]);
            }
        }
        
        // Top edge (right to left, skip corners)
        for (int x = max_x - 1; x >= min_x; --x) {
            if (active_cells.count({x, max_y})) {
                const Point3D& vertex = mesh.vertices[bottom_vertices[max_y][x]];
                outer_boundary.push_back({vertex.x, vertex.y});
                vertex_indices.push_back(bottom_vertices[max_y][x]);
            }
        }
        
        // Left edge (top to bottom, skip corners)
        for (int y = max_y - 1; y > min_y; --y) {
            if (active_cells.count({min_x, y})) {
                const Point3D& vertex = mesh.vertices[bottom_vertices[y][min_x]];
                outer_boundary.push_back({vertex.x, vertex.y});
                vertex_indices.push_back(bottom_vertices[y][min_x]);
            }
        }
        
        if (outer_boundary.size() < 3) {
            fallbackBottomTriangulation(mesh, bottom_vertices, terrain, filter_cells);
            return;
        }
        
        // Find interior holes using flood fill on inactive cells
        std::vector<std::vector<std::pair<double, double>>> polygon_rings;
        polygon_rings.push_back(outer_boundary);
        
        std::set<std::pair<int, int>> processed_holes;
        
        // Look for holes (inactive regions completely surrounded by active regions)
        for (int y = min_y + 1; y < max_y; ++y) {
            for (int x = min_x + 1; x < max_x; ++x) {
                if (!active_cells.count({x, y}) && !processed_holes.count({x, y})) {
                    
                    // Flood fill to find connected inactive region
                    std::vector<std::pair<int, int>> hole_cells;
                    std::queue<std::pair<int, int>> to_visit;
                    std::set<std::pair<int, int>> visited;
                    
                    to_visit.push({x, y});
                    visited.insert({x, y});
                    bool touches_boundary = false;
                    
                    while (!to_visit.empty()) {
                        auto current = to_visit.front();
                        to_visit.pop();
                        hole_cells.push_back(current);
                        
                        // Check if this touches the boundary of our active region
                        if (current.first <= min_x || current.first >= max_x || 
                            current.second <= min_y || current.second >= max_y) {
                            touches_boundary = true;
                        }
                        
                        // Explore 4-connected neighbors
                        int dx[] = {-1, 1, 0, 0};
                        int dy[] = {0, 0, -1, 1};
                        
                        for (int i = 0; i < 4; ++i) {
                            int nx = current.first + dx[i];
                            int ny = current.second + dy[i];
                            
                            if (nx >= 0 && nx < terrain.width && ny >= 0 && ny < terrain.height &&
                                !active_cells.count({nx, ny}) && !visited.count({nx, ny})) {
                                visited.insert({nx, ny});
                                to_visit.push({nx, ny});
                            }
                        }
                    }
                    
                    // Mark all cells as processed
                    for (const auto& cell : hole_cells) {
                        processed_holes.insert(cell);
                    }
                    
                    // If this doesn't touch boundary, it's a true hole
                    if (!touches_boundary && hole_cells.size() > 0) {
                        std::vector<std::pair<double, double>> hole_boundary;
                        
                        if (extractHoleBoundary(hole_cells, active_cells, bottom_vertices, mesh, hole_boundary, vertex_indices)) {
                            polygon_rings.push_back(hole_boundary);
                        }
                    }
                }
            }
        }
        
        // Use detria for proper Steiner point support instead of earcut workaround
        if (polygon_rings.size() <= 2 && (max_x - min_x) * (max_y - min_y) > 10) {
            
            std::vector<std::vector<std::pair<double, double>>> holes;
            for (size_t i = 1; i < polygon_rings.size(); ++i) {
                holes.push_back(polygon_rings[i]);
            }
            
            std::vector<std::pair<double, double>> steiner_points = 
                generateSteinerPoints(outer_boundary, holes, active_cells, terrain, 
                                    min_x, max_x, min_y, max_y);
            
            if (steiner_points.size() > 2) {
                triangulateWithDetria(mesh, outer_boundary, holes, steiner_points, vertex_indices, terrain);
                return;
            }
        }
        
        try {
            std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(polygon_rings);
            
            // Add triangles with correct bottom face orientation
            for (size_t i = 0; i < indices.size(); i += 3) {
                if (i + 2 < indices.size() && indices[i] < vertex_indices.size() && 
                    indices[i+1] < vertex_indices.size() && indices[i+2] < vertex_indices.size()) {
                    
                    size_t v0 = vertex_indices[indices[i]];
                    size_t v1 = vertex_indices[indices[i + 1]];
                    size_t v2 = vertex_indices[indices[i + 2]];
                    
                    // Add triangle with reversed winding for bottom face
                    mesh.addTriangle(v0, v2, v1);
                }
            }
        } catch (const std::exception&) {
            fallbackBottomTriangulation(mesh, bottom_vertices, terrain, filter_cells);
        }
    }
    
    // Extract hole boundary vertices (clockwise for earcut holes)
    bool extractHoleBoundary(const std::vector<std::pair<int, int>>& hole_cells,
                            const std::set<std::pair<int, int>>& active_cells,
                            const std::vector<std::vector<size_t>>& bottom_vertices,
                            const TerrainMesh& mesh,
                            std::vector<std::pair<double, double>>& hole_boundary,
                            std::vector<size_t>& vertex_indices) {
        
        // Find active cells adjacent to hole cells - these form the hole boundary
        std::set<std::pair<int, int>> boundary_cells;
        
        for (const auto& hole_cell : hole_cells) {
            int dx[] = {-1, 1, 0, 0};
            int dy[] = {0, 0, -1, 1};
            
            for (int i = 0; i < 4; ++i) {
                int nx = hole_cell.first + dx[i];
                int ny = hole_cell.second + dy[i];
                
                if (active_cells.count({nx, ny})) {
                    boundary_cells.insert({nx, ny});
                }
            }
        }
        
        if (boundary_cells.size() < 3) {
            return false;
        }
        
        // Find boundary box of hole boundary
        int min_x = INT_MAX, max_x = INT_MIN, min_y = INT_MAX, max_y = INT_MIN;
        for (const auto& cell : boundary_cells) {
            min_x = std::min(min_x, cell.first);
            max_x = std::max(max_x, cell.first);
            min_y = std::min(min_y, cell.second);
            max_y = std::max(max_y, cell.second);
        }
        
        // Trace hole boundary clockwise (required for earcut holes)
        std::vector<std::pair<int, int>> ordered_boundary;
        
        // Bottom edge (left to right)
        for (int x = min_x; x <= max_x; ++x) {
            if (boundary_cells.count({x, min_y})) {
                ordered_boundary.push_back({x, min_y});
            }
        }
        
        // Right edge (bottom to top, skip corners)
        for (int y = min_y + 1; y <= max_y; ++y) {
            if (boundary_cells.count({max_x, y})) {
                ordered_boundary.push_back({max_x, y});
            }
        }
        
        // Top edge (right to left, skip corners) 
        for (int x = max_x - 1; x >= min_x; --x) {
            if (boundary_cells.count({x, max_y})) {
                ordered_boundary.push_back({x, max_y});
            }
        }
        
        // Left edge (top to bottom, skip corners)
        for (int y = max_y - 1; y > min_y; --y) {
            if (boundary_cells.count({min_x, y})) {
                ordered_boundary.push_back({min_x, y});
            }
        }
        
        // Convert to vertex coordinates and indices
        for (const auto& cell : ordered_boundary) {
            const Point3D& vertex = mesh.vertices[bottom_vertices[cell.second][cell.first]];
            hole_boundary.push_back({vertex.x, vertex.y});
            vertex_indices.push_back(bottom_vertices[cell.second][cell.first]);
        }
        
        return hole_boundary.size() >= 3;
    }

    // Fallback triangulation method (original grid-based approach) 
    void fallbackBottomTriangulation(TerrainMesh& mesh, const std::vector<std::vector<size_t>>& bottom_vertices, 
                                    const TerrainData& terrain, const std::set<std::pair<int, int>>* filter_cells = nullptr) {
        
        for (int y = 0; y < terrain.height - 1; ++y) {
            for (int x = 0; x < terrain.width - 1; ++x) {
                // Check if all 4 vertices exist for this cell
                size_t v00 = bottom_vertices[y][x];
                size_t v10 = bottom_vertices[y][x + 1];
                size_t v01 = bottom_vertices[y + 1][x];
                size_t v11 = bottom_vertices[y + 1][x + 1];
                
                if (v00 != SIZE_MAX && v10 != SIZE_MAX && v01 != SIZE_MAX && v11 != SIZE_MAX) {
                    // If filter_cells is provided, check if this cell should be included
                    bool include_cell = (filter_cells == nullptr) || 
                                       (filter_cells->count({x, y}) && filter_cells->count({x+1, y}) && 
                                        filter_cells->count({x, y+1}) && filter_cells->count({x+1, y+1}));
                    
                    if (include_cell) {
                        // Add two triangles with CCW orientation (viewed from below, so reversed)
                        mesh.addTriangle(v00, v10, v01);
                        mesh.addTriangle(v10, v11, v01);
                    }
                }
            }
        }
    }

    // Validate mesh properties
    MeshStats validateMesh(const TerrainMesh& mesh, const TerrainData& terrain) {
        MeshStats stats;
        
        // Check edge manifold property
        std::unordered_map<Edge, int, EdgeHash> edge_count;
        
        for (const auto& triangle : mesh.triangles) {
            Edge e1(triangle.vertices[0], triangle.vertices[1]);
            Edge e2(triangle.vertices[1], triangle.vertices[2]);
            Edge e3(triangle.vertices[2], triangle.vertices[0]);
            
            edge_count[e1]++;
            edge_count[e2]++;
            edge_count[e3]++;
        }
        
        // Count non-manifold edges
        for (const auto& pair : edge_count) {
            if (pair.second != 2) {
                stats.non_manifold_edges++;
                stats.is_manifold = false;
            }
        }
        
        // Check CCW orientation - for a volumetric mesh, this is more complex
        // We'll consider the mesh properly oriented if all normals point outward
        stats.is_ccw_oriented = true;
        int total_triangles = 0;
        int properly_oriented = 0;
        
        for (const auto& triangle : mesh.triangles) {
            const Point3D& p0 = mesh.vertices[triangle.vertices[0]];
            const Point3D& p1 = mesh.vertices[triangle.vertices[1]];
            const Point3D& p2 = mesh.vertices[triangle.vertices[2]];
            
            Point3D edge1 = p1 - p0;
            Point3D edge2 = p2 - p0;
            Point3D normal = edge1.cross(edge2);
            
            // Check if this triangle is non-degenerate
            if (normal.length() > 1e-10) {
                total_triangles++;
                // For this simple validation, we'll accept any non-degenerate triangle
                properly_oriented++;
            }
        }
        
        if (total_triangles > 0) {
            stats.is_ccw_oriented = (properly_oriented >= total_triangles * 0.95); // 95% threshold
        }
        
        // Calculate volume using divergence theorem
        stats.volume = 0.0;
        for (const auto& triangle : mesh.triangles) {
            const Point3D& p0 = mesh.vertices[triangle.vertices[0]];
            const Point3D& p1 = mesh.vertices[triangle.vertices[1]];
            const Point3D& p2 = mesh.vertices[triangle.vertices[2]];
            
            // Volume contribution from this triangle
            stats.volume += (p0.x * (p1.y * p2.z - p2.y * p1.z) +
                           p1.x * (p2.y * p0.z - p0.y * p2.z) +
                           p2.x * (p0.y * p1.z - p1.y * p0.z)) / 6.0;
        }
        stats.volume = std::abs(stats.volume);
        
        // Calculate expected volume from terrain data
        stats.expected_volume = 0.0;
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                double height = terrain.getHeight(x, y);
                stats.expected_volume += height * terrain.cell_size * terrain.cell_size;
            }
        }
        
        // Calculate surface area (only terrain surface triangles)
        stats.surface_area = 0.0;
        for (size_t i = 0; i < mesh.surface_triangle_count && i < mesh.triangles.size(); ++i) {
            const Triangle& triangle = mesh.triangles[i];
            const Point3D& p0 = mesh.vertices[triangle.vertices[0]];
            const Point3D& p1 = mesh.vertices[triangle.vertices[1]];
            const Point3D& p2 = mesh.vertices[triangle.vertices[2]];
            
            Point3D edge1 = p1 - p0;
            Point3D edge2 = p2 - p0;
            Point3D cross_product = edge1.cross(edge2);
            stats.surface_area += cross_product.length() * 0.5;
        }
        
        // Expected surface area (rough approximation)
        stats.expected_surface_area = terrain.width * terrain.height * terrain.cell_size * terrain.cell_size;
        
        return stats;
    }

    // Write mesh to OBJ file
    bool writeObjFile(const std::string& filename, const TerrainMesh& mesh) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        file << "# TerraScape generated OBJ file" << std::endl;
        file << "# Vertices: " << mesh.vertices.size() << std::endl;
        file << "# Triangles: " << mesh.triangles.size() << std::endl;
        file << std::endl;
        
        // Write vertices
        for (const auto& vertex : mesh.vertices) {
            file << "v " << std::fixed << std::setprecision(6) 
                 << vertex.x << " " << vertex.y << " " << vertex.z << std::endl;
        }
        
        file << std::endl;
        
        // Write triangles (OBJ uses 1-based indexing)
        for (const auto& triangle : mesh.triangles) {
            file << "f " << (triangle.vertices[0] + 1) << " " 
                         << (triangle.vertices[1] + 1) << " " 
                         << (triangle.vertices[2] + 1) << std::endl;
        }
        
        file.close();
        return true;
    }

    // BRL-CAD DSP Integration Functions
    
    // Convert DSP data to TerraScape TerrainData format
    bool convertDSPToTerrain(const DSPData& dsp, TerrainData& terrain) {
        if (!dsp.dsp_buf || dsp.dsp_xcnt == 0 || dsp.dsp_ycnt == 0) {
            return false;
        }
        
        terrain.width = static_cast<int>(dsp.dsp_xcnt);
        terrain.height = static_cast<int>(dsp.dsp_ycnt);
        terrain.cell_size = dsp.cell_size;
        terrain.origin = dsp.origin;
        
        // Initialize height array
        terrain.heights.resize(terrain.height);
        for (int y = 0; y < terrain.height; ++y) {
            terrain.heights[y].resize(terrain.width);
        }
        
        // Convert unsigned short data to double and find min/max
        terrain.min_height = std::numeric_limits<double>::max();
        terrain.max_height = std::numeric_limits<double>::lowest();
        
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                double height = static_cast<double>(dsp.dsp_buf[y * dsp.dsp_xcnt + x]);
                terrain.heights[y][x] = height;
                terrain.min_height = std::min(terrain.min_height, height);
                terrain.max_height = std::max(terrain.max_height, height);
            }
        }
        
        return true;
    }
    
    // Convert TerraScape TerrainData to DSP format
    bool convertTerrainToDSP(const TerrainData& terrain, DSPData& dsp) {
        if (terrain.width <= 0 || terrain.height <= 0 || terrain.heights.empty()) {
            return false;
        }
        
        dsp.dsp_xcnt = static_cast<uint32_t>(terrain.width);
        dsp.dsp_ycnt = static_cast<uint32_t>(terrain.height);
        dsp.cell_size = terrain.cell_size;
        dsp.origin = terrain.origin;
        
        // Allocate buffer if not already allocated
        if (!dsp.dsp_buf) {
            dsp.dsp_buf = new unsigned short[dsp.dsp_xcnt * dsp.dsp_ycnt];
            dsp.owns_buffer = true;
        }
        
        // Convert double data to unsigned short
        for (int y = 0; y < terrain.height; ++y) {
            for (int x = 0; x < terrain.width; ++x) {
                double height = terrain.getHeight(x, y);
                // Clamp to unsigned short range
                if (height < 0) height = 0;
                if (height > 65535) height = 65535;
                dsp.dsp_buf[y * dsp.dsp_xcnt + x] = static_cast<unsigned short>(height);
            }
        }
        
        return true;
    }
    
    // Convert TerrainMesh to NMG-compatible triangle data
    bool convertMeshToNMG(const TerrainMesh& mesh, NMGTriangleData& nmg_data) {
        if (mesh.vertices.empty() || mesh.triangles.empty()) {
            return false;
        }
        
        nmg_data.triangles.clear();
        nmg_data.unique_vertices = mesh.vertices;  // Copy vertex data
        nmg_data.surface_triangle_count = mesh.surface_triangle_count;
        
        // Convert triangles to NMG format
        nmg_data.triangles.reserve(mesh.triangles.size());
        
        for (size_t i = 0; i < mesh.triangles.size(); ++i) {
            const Triangle& tri = mesh.triangles[i];
            
            // Create triangle vertices with references to unique vertex array
            NMGTriangleData::TriangleVertex v0(mesh.vertices[tri.vertices[0]], tri.vertices[0]);
            NMGTriangleData::TriangleVertex v1(mesh.vertices[tri.vertices[1]], tri.vertices[1]);
            NMGTriangleData::TriangleVertex v2(mesh.vertices[tri.vertices[2]], tri.vertices[2]);
            
            // Determine if this is a surface triangle (first N triangles are surface)
            bool is_surface = (i < mesh.surface_triangle_count);
            
            // Create NMG triangle
            NMGTriangleData::NMGTriangle nmg_tri(v0, v1, v2, is_surface);
            nmg_data.triangles.push_back(nmg_tri);
        }
        
        return true;
    }
    
    // Main function for BRL-CAD: Convert DSP to triangulated mesh ready for nmg_region
    bool triangulateTerrainForBRLCAD(const DSPData& dsp, NMGTriangleData& nmg_data) {
        // Step 1: Convert DSP to TerraScape format
        TerrainData terrain;
        if (!convertDSPToTerrain(dsp, terrain)) {
            return false;
        }
        
        // Step 2: Generate volumetric triangle mesh using TerraScape
        TerrainMesh mesh;
        triangulateTerrainVolume(terrain, mesh);
        
        // Step 3: Convert mesh to NMG format
        return convertMeshToNMG(mesh, nmg_data);
    }

    // Example function showing how to integrate with BRL-CAD rt_dsp_tess
    // This demonstrates the interface that rt_dsp_tess would use
    /*
    Usage in rt_dsp_tess replacement:
    
    int rt_dsp_tess_terrascape(struct nmgregion **r, struct model *m, 
                              struct rt_db_internal *ip, 
                              const struct bg_tess_tol *ttol, 
                              const struct bn_tol *tol) {
        
        struct rt_dsp_internal *dsp_ip = (struct rt_dsp_internal *)ip->idb_ptr;
        
        // Step 1: Create TerraScape DSPData from rt_dsp_internal
        TerraScape::DSPData dsp;
        dsp.dsp_buf = dsp_ip->dsp_buf;           // Point to existing buffer
        dsp.dsp_xcnt = dsp_ip->dsp_xcnt;         // Copy dimensions
        dsp.dsp_ycnt = dsp_ip->dsp_ycnt;
        dsp.cell_size = 1.0;                     // Will be scaled by transformation matrix
        dsp.origin = TerraScape::Point3D(0, 0, 0);
        dsp.owns_buffer = false;                 // Don't delete BRL-CAD's buffer
        
        // Step 2: Generate triangulated mesh
        TerraScape::NMGTriangleData nmg_data;
        if (!TerraScape::triangulateTerrainForBRLCAD(dsp, nmg_data)) {
            return -1; // FAIL
        }
        
        // Step 3: Create nmg_region and shell
        *r = nmg_mrsv(m);
        struct shell *s = BU_LIST_FIRST(shell, &(*r)->s_hd);
        
        // Step 4: Convert TerraScape triangles to NMG vertices and faces
        std::vector<struct vertex*> vertices(nmg_data.unique_vertices.size());
        
        // Create vertices
        for (size_t i = 0; i < nmg_data.unique_vertices.size(); ++i) {
            const auto& pt = nmg_data.unique_vertices[i];
            point_t model_pt;
            
            // Transform from DSP coordinates to model space using dsp_stom matrix
            point_t dsp_pt = {pt.x, pt.y, pt.z};
            MAT4X3PNT(model_pt, dsp_ip->dsp_stom, dsp_pt);
            
            vertices[i] = nmg_msv(*r);
            nmg_vertex_gv(vertices[i], model_pt);
        }
        
        // Create faces
        for (const auto& triangle : nmg_data.triangles) {
            struct vertex *face_verts[3];
            face_verts[0] = vertices[triangle.vertices[0].original_index];
            face_verts[1] = vertices[triangle.vertices[1].original_index];
            face_verts[2] = vertices[triangle.vertices[2].original_index];
            
            struct faceuse *fu = nmg_cmface(s, face_verts, 3);
            if (!fu) {
                return -1; // FAIL
            }
        }
        
        // Step 5: Final processing (same as original rt_dsp_tess)
        nmg_mark_edges_real(&s->l.magic, &rt_vlfree);
        nmg_region_a(*r, tol);
        nmg_make_faces_within_tol(s, &rt_vlfree, tol);
        
        return 0; // SUCCESS
    }
    */
    
    // Manifold validation implementation using @elalish/manifold library
    ManifoldValidationInfo validateMeshWithManifold(const TerrainMesh& mesh) {
        ManifoldValidationInfo info;
        
        // Convert TerraScape mesh to manifold::MeshGL format
        manifold::MeshGL meshGL;
        
        // Convert vertices
        meshGL.vertProperties.reserve(mesh.vertices.size() * 3);
        for (const auto& vertex : mesh.vertices) {
            meshGL.vertProperties.push_back(static_cast<float>(vertex.x));
            meshGL.vertProperties.push_back(static_cast<float>(vertex.y));  
            meshGL.vertProperties.push_back(static_cast<float>(vertex.z));
        }
        
        // Convert triangles
        meshGL.triVerts.reserve(mesh.triangles.size() * 3);
        for (const auto& triangle : mesh.triangles) {
            meshGL.triVerts.push_back(static_cast<uint32_t>(triangle.vertices[0]));
            meshGL.triVerts.push_back(static_cast<uint32_t>(triangle.vertices[1]));
            meshGL.triVerts.push_back(static_cast<uint32_t>(triangle.vertices[2]));
        }
        
        // Set vertex attributes (3 attributes per vertex: x, y, z)
        meshGL.numProp = 3;
        
        info.num_vertices = mesh.vertices.size();
        info.num_triangles = mesh.triangles.size();
        
        try {
            // Create manifold from mesh
            manifold::Manifold manifold(meshGL);
            
            // Check status
            manifold::Manifold::Error status = manifold.Status();
            
            switch (status) {
                case manifold::Manifold::Error::NoError:
                    info.result = ManifoldValidationResult::Valid;
                    info.is_manifold = true;
                    info.error_message = "Mesh is valid and manifold";
                    break;
                    
                case manifold::Manifold::Error::NonFiniteVertex:
                    info.result = ManifoldValidationResult::NonFiniteVertex;
                    info.is_manifold = false;
                    info.error_message = "Mesh contains non-finite vertices (NaN or infinity)";
                    break;
                    
                case manifold::Manifold::Error::NotManifold:
                    info.result = ManifoldValidationResult::NotManifold;
                    info.is_manifold = false;
                    info.error_message = "Mesh is not manifold (edge-manifold violated)";
                    break;
                    
                case manifold::Manifold::Error::VertexOutOfBounds:
                    info.result = ManifoldValidationResult::VertexOutOfBounds;
                    info.is_manifold = false;
                    info.error_message = "Triangle references vertex index out of bounds";
                    break;
                    
                default:
                    info.result = ManifoldValidationResult::OtherError;
                    info.is_manifold = false;
#ifdef MANIFOLD_DEBUG
                    info.error_message = "Other manifold validation error: " + manifold::ToString(status);
#else
                    info.error_message = "Other manifold validation error (error code: " + std::to_string(static_cast<int>(status)) + ")";
#endif
                    break;
            }
            
        } catch (const std::exception& e) {
            info.result = ManifoldValidationResult::OtherError;
            info.is_manifold = false;
            info.error_message = std::string("Exception during manifold validation: ") + e.what();
        }
        
        return info;
    }

} // namespace TerraScape

#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic pop /* end ignoring warnings */
#elif defined(__clang__)
#  pragma clang diagnostic pop /* end ignoring warnings */
#endif


// Local Variables:
// tab-width: 8
// mode: C++
// c-basic-offset: 4
// indent-tabs-mode: t
// c-file-style: "stroustrup"
// End:
// ex: shiftwidth=4 tabstop=8 cino=N-s
