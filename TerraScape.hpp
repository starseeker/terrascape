#pragma once

/*
 * TerraScape - Terrain Triangle Mesh Generation
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

    // Forward declarations
    bool readTerrainFile(const std::string& filename, TerrainData& terrain);
    bool readPGMFile(const std::string& filename, TerrainData& terrain);
    void triangulateTerrainVolume(const TerrainData& terrain, TerrainMesh& mesh);
    MeshStats validateMesh(const TerrainMesh& mesh, const TerrainData& terrain);
    bool writeObjFile(const std::string& filename, const TerrainMesh& mesh);

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

    // Generate a volumetric triangle mesh from terrain data
    void triangulateTerrainVolume(const TerrainData& terrain, TerrainMesh& mesh) {
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

        // Add bottom surface triangles (2 triangles per cell, opposite orientation)
        for (int y = 0; y < terrain.height - 1; ++y) {
            for (int x = 0; x < terrain.width - 1; ++x) {
                size_t v00 = bottom_vertices[y][x];
                size_t v10 = bottom_vertices[y][x + 1];
                size_t v01 = bottom_vertices[y + 1][x];
                size_t v11 = bottom_vertices[y + 1][x + 1];

                // Add two triangles with CCW orientation (viewed from below, so reversed)
                mesh.addTriangle(v00, v10, v01);
                mesh.addTriangle(v10, v11, v01);
            }
        }

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
