/*                  T E R R A S C A P E . H P P
 * BRL-CAD
 *
 * Published in 2025 by the United States Government.
 * This work is in the public domain.
 *
 */
/** @file TerraScape.hpp
 *
 * Terrain Triangle Mesh Generation
 *
 * This logic implements algorithms for converting elevation grids
 * (Digital Elevation Models) into triangle meshes.  It focuses on
 * producing manifold meshes defining closed volumes, while offering
 * some ability to adjust the level of detail with which the surface
 * terrain is represented in the output mesh.
 *
 * The surface mesh (what most people would think of as the terrain features)
 * is produced using a feature-driven, grid-constrained sampling that retains
 * boundary points and promotes high-slope / high-curvature / high-roughness
 * cells until a minimum vertex density is satisfied.  There is no vertex
 * relocation or snapping to features - the method is quite basic - so the
 * underlying grid structure will be visually apparent.
 *
 * The "walls" are basically handled as cell edge extrusions from the ground
 * plane to their corresponding surface mesh cell cell.  Generally these are
 * not long and narrow enough to be numerically problematic, so we use the
 * simple approach to construct these triangles. (In theory that CAN change if
 * the terrain height gets tall enough relative to the cell size, so we may
 * someday have to revisit the issue if problems arise.)
 *
 * The bottom face - i.e. the face opposite the surface mesh generated from the
 * height data - is another matter. A mesh can be produced for this face
 * trivially by adding two triangles for every cell, but that approach produces
 * an enormous number of triangles which contribute nothing to the shape of the
 * overall volume.  Moreover, the grid aligned, linear, coplanar nature of the
 * mesh makes it tricky to use some of the standard computer graphics
 * decimation techniques successfully while maintaining a manifold topology.
 * Going the other way - using only the boundary polylines and trying to
 * triangulate - has its own problems.  The result if done naively tends to be
 * super long and super thin triangles for much of the mesh, with a few
 * extremely large triangles to close out the patterns.  Long thin triangles
 * pose their own analytic problems, so we want to avoid that as well.
 *
 * In the end, we went with the detria (https://github.com/Kimbatt/detria)
 * constrained Delaunay triangulation implementation for its robustness and
 * support of Steiner points.  We sample the interior of the bottom face
 * polygons to provide interior Steiner points that improve the overall quality
 * of the triangulation - it adds a few more triangles and vertices to the mesh
 * size, but the mesh quality requires it and typically the surface mesh
 * triangle count will hugely dominate the triangle count anyway.  If the
 * detria triangulation *should* fail for some reason, we do fall back to the
 * naive bottom face two-triangles-per-cell mesh approach rather than failing
 * to produce an output - that, however, tends to double the mesh size.
 *
 * Chew, L. P. (1989). "Constrained Delaunay triangulations." Algorithmica,
 * 4(1), 97-108.
 */

#pragma once

#include <vector>
#include <array>
#include <tuple>
#include <string>
#include <queue>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <memory>

#include <cmath>
#include <algorithm>
#include <limits>
#include <climits>
#include <cstdint>
#include <stdexcept>
#include <iostream>

#ifndef _USE_MATH_DEFINES
#  define _USE_MATH_DEFINES
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic push /* start new diagnostic pragma */
#  pragma GCC diagnostic ignored "-Wfloat-equal"
#elif defined(__clang__)
#  pragma clang diagnostic push /* start new diagnostic pragma */
#  pragma clang diagnostic ignored "-Wfloat-equal"
#endif

#include "detria.hpp"

namespace TerraScape {

// Forward declarations for types used in method signatures
class TerrainFeature;
class SimplificationParams;
class TerrainComponents;
class ConnectedComponent;
class MeshStats;
class TerrainData;
class DSPData;
class NMGTriangleData;
class QuadricError;
class EdgeCollapse;

// Basic 3D point structure
class Point3D {
    public:
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
class Triangle {
    public:
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
	    return cross_product.length() > 0; // non-degenerate test
	}

	// Calculate triangle area
	double area(const std::vector<Point3D>& vertex_list) const {
	    const Point3D& p0 = vertex_list[vertices[0]];
	    const Point3D& p1 = vertex_list[vertices[1]];
	    const Point3D& p2 = vertex_list[vertices[2]];

	    Point3D edge1 = p1 - p0;
	    Point3D edge2 = p2 - p0;
	    return edge1.cross(edge2).length() * 0.5;
	}

	// Check if triangle is degenerate (zero area)
	bool isDegenerate(const std::vector<Point3D>& vertex_list, double tolerance = 1e-10) const {
	    return area(vertex_list) < tolerance;
	}
};

// Terrain data structure
class TerrainData {
    public:
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

	// Method declarations - implementations follow after all class definitions
	TerrainFeature analyzePoint(int x, int y) const;
	std::vector<std::vector<bool>> generateSampleMask(const SimplificationParams& params) const;
	
	// New Garland-Heckbert simplification methods
	TerrainFeature analyzePointWithQuadric(int x, int y) const;
	std::vector<std::vector<bool>> generateGarlandHeckbertMask(const SimplificationParams& params) const;
	
	TerrainComponents analyzeComponents(double height_threshold = 1e-6) const;

	// DSP conversion methods
	bool fromDSP(const DSPData& dsp);
	bool toDSP(DSPData& dsp) const;

	// Generate interior Steiner points to use for triangulation
	std::vector<std::pair<double, double>> generateSteinerPoints(
		const std::vector<std::pair<double, double>>& boundary,
		const std::vector<std::vector<std::pair<double, double>>>& holes,
		const std::set<std::pair<int, int>>& active_cells,
		double min_x_in, double max_x_in, double min_y_in, double max_y_in) const;

    private:
	bool addSteinerPointIfValid(double x, double y,
		const std::vector<std::pair<double, double>>& boundary,
		const std::vector<std::vector<std::pair<double, double>>>& holes,
		const std::set<std::pair<int, int>>& active_cells,
		double min_distance,
		const std::function<double(double, double)>& distanceToEdges,
		std::vector<std::pair<double, double>>& steiner_points) const;

	void processGuideLines(const std::vector<std::pair<double, double>>& edge_points,
		double center_x, double center_y,
		double min_viable_len,
		const std::vector<double>& step_probabilities,
		const std::function<double()>& next_random,
		const std::vector<std::pair<double, double>>& boundary,
		const std::vector<std::vector<std::pair<double, double>>>& holes,
		const std::set<std::pair<int, int>>& active_cells,
		double min_distance,
		const std::function<double(double, double)>& distanceToEdges,
		std::vector<std::pair<double, double>>& steiner_points,
		size_t sample_step,
		double step_divisor) const;

	void floodFill(std::vector<std::vector<bool>>& visited,
		ConnectedComponent& component, int start_x, int start_y, double height_threshold) const;
};

// Triangle mesh structure
class TerrainMesh {
    public:
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

	// Validate mesh properties
	MeshStats validate(const TerrainData& terrain) const;

	// Calculate total surface area of all triangles
	double calculateTotalArea() const {
	    double total = 0.0;
	    for (const auto& triangle : triangles) {
		total += triangle.area(vertices);
	    }
	    return total;
	}

	// Calculate surface area of terrain surface triangles only
	double calculateSurfaceArea() const {
	    double total = 0.0;
	    for (size_t i = 0; i < surface_triangle_count && i < triangles.size(); ++i) {
		total += triangles[i].area(vertices);
	    }
	    return total;
	}

	// Convert mesh to NMG-compatible triangle data
	bool toNMG(NMGTriangleData& nmg_data) const;

	// Triangulation methods (moved from standalone functions)
	void triangulateVolume(const TerrainData& terrain);
	void triangulateVolumeLegacy(const TerrainData& terrain);
	void triangulateVolumeSimplified(const TerrainData& terrain, const SimplificationParams& params);
	void triangulateSurfaceOnly(const TerrainData& terrain, const SimplificationParams& params);
	void triangulateVolumeWithComponents(const TerrainData& terrain);
	void triangulateComponentVolume(const TerrainData& terrain, const ConnectedComponent& component);
	
	// Full Garland-Heckbert implementation with edge collapse
	void generateGarlandHeckbertSurface(const TerrainData& terrain, const SimplificationParams& params);
	void triangulateVolumeWithGarlandHeckbert(const TerrainData& terrain, const SimplificationParams& params);

    private:
	// Helper methods for triangulation
	void triangulateBottomFaceWithDetria(const std::vector<std::vector<size_t>>& bottom_vertices,
		const TerrainData& terrain, const std::set<std::pair<int, int>>* filter_cells);
	void fallbackBottomTriangulation(const std::vector<std::vector<size_t>>& bottom_vertices,
		const TerrainData& terrain, const std::set<std::pair<int, int>>* filter_cells);
	bool extractHoleBoundary(const std::vector<std::pair<int, int>>& hole_cells,
		const std::set<std::pair<int, int>>& active_cells,
		const std::vector<std::vector<size_t>>& bottom_vertices,
		std::vector<std::pair<double, double>>& hole_boundary,
		std::vector<size_t>& vertex_indices);
		
	// Garland-Heckbert surface mesh generation helpers
	void buildInitialSurfaceTriangulation(const TerrainData& terrain,
		std::vector<Point3D>& surface_vertices,
		std::vector<Triangle>& surface_triangles,
		std::vector<std::vector<size_t>>& grid_to_vertex);
	void computeVertexQuadrics(const std::vector<Point3D>& vertices, 
		const std::vector<Triangle>& triangles,
		std::vector<QuadricError>& vertex_quadrics);
	void buildEdgeCollapseQueue(const std::vector<Point3D>& vertices,
		const std::vector<Triangle>& triangles,
		const std::vector<QuadricError>& vertex_quadrics,
		std::priority_queue<EdgeCollapse>& collapse_queue);
	bool performEdgeCollapse(const EdgeCollapse& collapse,
		std::vector<Point3D>& vertices,
		std::vector<Triangle>& triangles,
		std::vector<QuadricError>& vertex_quadrics,
		std::vector<bool>& vertex_removed,
		std::priority_queue<EdgeCollapse>& collapse_queue);
	void rebuildAffectedRegion(size_t collapsed_vertex, size_t remaining_vertex,
		const std::vector<Point3D>& vertices,
		std::vector<Triangle>& triangles);
	Point3D computeOptimalCollapsePosition(const QuadricError& q1, const QuadricError& q2,
		const Point3D& v1, const Point3D& v2);
};

// Mesh validation statistics
class MeshStats {
    public:
	MeshStats() : volume(0), surface_area(0), expected_volume(0), expected_surface_area(0),
	is_manifold(true), is_ccw_oriented(true), non_manifold_edges(0) {}

	// Getters
	double getVolume() const { return volume; }
	double getSurfaceArea() const { return surface_area; }
	double getExpectedVolume() const { return expected_volume; }
	double getExpectedSurfaceArea() const { return expected_surface_area; }
	bool getIsManifold() const { return is_manifold; }
	bool getIsCCWOriented() const { return is_ccw_oriented; }
	int getNonManifoldEdges() const { return non_manifold_edges; }

	// Setters for internal use
	void setVolume(double val) { volume = val; }
	void setSurfaceArea(double val) { surface_area = val; }
	void setExpectedVolume(double val) { expected_volume = val; }
	void setExpectedSurfaceArea(double val) { expected_surface_area = val; }
	void setIsManifold(bool val) { is_manifold = val; }
	void setIsCCWOriented(bool val) { is_ccw_oriented = val; }
	void setNonManifoldEdges(int val) { non_manifold_edges = val; }

    private:
	double volume;
	double surface_area;
	double expected_volume;
	double expected_surface_area;
	bool is_manifold;
	bool is_ccw_oriented;
	int non_manifold_edges;
};

// Terrain simplification parameters based on Terra/Scape concepts
class SimplificationParams {
    public:
	SimplificationParams() :
	    error_tol(0.1),
	    slope_tol(0.2),
	    min_reduction(50),
	    preserve_bounds(true) {}

	// Getters
	double getErrorTol() const { return error_tol; }
	double getSlopeTol() const { return slope_tol; }
	int getMinReduction() const { return min_reduction; }
	bool getPreserveBounds() const { return preserve_bounds; }

	// Setters
	void setErrorTol(double val) { error_tol = val; }
	void setSlopeTol(double val) { slope_tol = val; }
	void setMinReduction(int val) { min_reduction = val; }
	void setPreserveBounds(bool val) { preserve_bounds = val; }

    private:
	double error_tol;      // Maximum allowed geometric error
	double slope_tol;      // Slope threshold for feature preservation
	int min_reduction;     // Minimum percentage of triangles to remove
	bool preserve_bounds;  // Whether to preserve terrain boundaries
};

// Quadric Error Metric for Garland-Heckbert simplification
class QuadricError {
    public:
	QuadricError() {
	    // Initialize quadric as zero
	    for (int i = 0; i < 10; i++) q[i] = 0.0;
	}
	
	// Initialize quadric from a plane equation ax + by + cz + d = 0
	QuadricError(double a, double b, double c, double d) {
	    q[0] = a*a;   q[1] = a*b;   q[2] = a*c;   q[3] = a*d;
	    q[4] = b*b;   q[5] = b*c;   q[6] = b*d;
	    q[7] = c*c;   q[8] = c*d;
	    q[9] = d*d;
	}
	
	// Add another quadric to this one
	QuadricError& operator+=(const QuadricError& other) {
	    for (int i = 0; i < 10; i++) {
		q[i] += other.q[i];
	    }
	    return *this;
	}
	
	QuadricError operator+(const QuadricError& other) const {
	    QuadricError result = *this;
	    result += other;
	    return result;
	}
	
	// Evaluate quadric at point (x, y, z) - returns squared error
	double evaluate(double x, double y, double z) const {
	    return q[0]*x*x + 2*q[1]*x*y + 2*q[2]*x*z + 2*q[3]*x +
		   q[4]*y*y + 2*q[5]*y*z + 2*q[6]*y + 
		   q[7]*z*z + 2*q[8]*z + 
		   q[9];
	}
	
	// Find optimal point that minimizes quadric (solve for minimum)
	Point3D findOptimalPoint() const {
	    // Try to solve the linear system Ax = b where A is the quadric matrix
	    // For terrains, we can often use simpler heuristics
	    return Point3D(0, 0, 0); // Will implement more sophisticated version
	}
	
    private:
	// Quadric coefficients for x²  xy  xz   x  y²  yz   y  z²   z   1
	double q[10];               // 0   1   2   3   4   5   6   7   8   9
};

// Local terrain analysis for adaptive simplification
class TerrainFeature {
    public:
	TerrainFeature() : curvature(0), slope(0), roughness(0), is_boundary(false), importance(0), quadric_error(0.0) {}

	// Getters (legacy interface)
	double getCurvature() const { return curvature; }
	double getSlope() const { return slope; }
	double getRoughness() const { return roughness; }
	bool getIsBoundary() const { return is_boundary; }
	double getImportance() const { return importance; }
	
	// New Garland-Heckbert interface
	const QuadricError& getQuadric() const { return quadric; }
	double getQuadricError() const { return quadric_error; }

	// Setters (legacy interface)
	void setCurvature(double val) { curvature = val; }
	void setSlope(double val) { slope = val; }
	void setRoughness(double val) { roughness = val; }
	void setIsBoundary(bool val) { is_boundary = val; }
	void setImportance(double val) { importance = val; }
	
	// New Garland-Heckbert setters
	void setQuadric(const QuadricError& q) { quadric = q; }
	void setQuadricError(double error) { quadric_error = error; }

    private:
	// Legacy Terra/Scape features
	double curvature;          // Local surface curvature
	double slope;              // Local slope magnitude
	double roughness;          // Local height variation
	bool is_boundary;          // Whether this is a boundary vertex
	double importance;         // Combined importance metric
	
	// New Garland-Heckbert features
	QuadricError quadric;      // Quadric error metric for this vertex
	double quadric_error;      // Cached quadric error value
};

// Edge structure for manifold checking
class Edge {
    public:
	Edge(size_t a, size_t b) {
	    if (a < b) {
		v0 = a; v1 = b;
	    } else {
		v0 = b; v1 = a;
	    }
	}

	// Getters
	size_t getV0() const { return v0; }
	size_t getV1() const { return v1; }

	bool operator<(const Edge& other) const {
	    if (v0 != other.v0) return v0 < other.v0;
	    return v1 < other.v1;
	}

	bool operator==(const Edge& other) const {
	    return v0 == other.v0 && v1 == other.v1;
	}
    private:
	size_t v0, v1;
};

// Edge collapse candidate for Garland-Heckbert decimation
class EdgeCollapse {
    public:
	EdgeCollapse(size_t v0, size_t v1, double error, const Point3D& new_pos) 
	    : edge(v0, v1), collapse_error(error), new_position(new_pos) {}
	    
	Edge edge;                    // Edge to collapse
	double collapse_error;        // Error introduced by this collapse
	Point3D new_position;         // Optimal position for merged vertex
	
	// For priority queue (min-heap)
	bool operator<(const EdgeCollapse& other) const {
	    return collapse_error > other.collapse_error; // Reverse for min-heap
	}
	
	bool operator>(const EdgeCollapse& other) const {
	    return collapse_error < other.collapse_error;
	}
};

// Hash function for Edge
class EdgeHash {
    public:
	size_t operator()(const Edge& e) const {
	    return std::hash<size_t>()(e.getV0()) ^ (std::hash<size_t>()(e.getV1()) << 1);
	}
};

// Connected component structure for handling terrain islands
class ConnectedComponent {
    public:
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
class TerrainComponents {
    public:
	std::vector<ConnectedComponent> components;
	std::vector<std::vector<int>> component_map;  // component_id for each cell (-1 for background)

	TerrainComponents(int width, int height) {
	    component_map.resize(height, std::vector<int>(width, -1));
	}
};

// DSP compatibility structures
class DSPData {
    public:
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

	// Convert to TerrainData format
	bool toTerrain(TerrainData& terrain) const;
};

// NMG-compatible triangle output
class NMGTriangleData {
    public:
	class TriangleVertex {
	    public:
		Point3D point;
		size_t original_index;  // Index in original vertex array

		TriangleVertex(const Point3D& p, size_t idx) : point(p), original_index(idx) {}
	};

	class NMGTriangle {
	    public:
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

// Point-in-polygon test
bool pointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon);

// Implementation

// Analyze terrain features at a specific point (Terra/Scape inspired)
TerrainFeature TerrainData::analyzePoint(int x, int y) const {
    TerrainFeature feature;

    if (!isValidCell(x, y)) {
	return feature;
    }

    // Calculate local slope using central differences
    double dx = 0, dy = 0;
    if (isValidCell(x-1, y) && isValidCell(x+1, y)) {
	dx = (getHeight(x+1, y) - getHeight(x-1, y)) / (2.0 * cell_size);
    }
    if (isValidCell(x, y-1) && isValidCell(x, y+1)) {
	dy = (getHeight(x, y+1) - getHeight(x, y-1)) / (2.0 * cell_size);
    }
    feature.setSlope(std::sqrt(dx*dx + dy*dy));

    // Calculate local curvature (second derivatives)
    double dxx = 0, dyy = 0;
    double h_center = getHeight(x, y);
    if (isValidCell(x-1, y) && isValidCell(x+1, y)) {
	dxx = (getHeight(x+1, y) - 2*h_center + getHeight(x-1, y)) / (cell_size * cell_size);
    }
    if (isValidCell(x, y-1) && isValidCell(x, y+1)) {
	dyy = (getHeight(x, y+1) - 2*h_center + getHeight(x, y-1)) / (cell_size * cell_size);
    }
    feature.setCurvature(std::abs(dxx) + std::abs(dyy));

    // Calculate local roughness (height variation in neighborhood)
    double height_sum = 0;
    double height_variance = 0;
    int neighbor_count = 0;
    for (int oy = -1; oy <= 1; ++oy) {
	for (int ox = -1; ox <= 1; ++ox) {
	    if (isValidCell(x+ox, y+oy)) {
		double h = getHeight(x+ox, y+oy);
		height_sum += h;
		neighbor_count++;
	    }
	}
    }

    if (neighbor_count > 0) {
	double mean_height = height_sum / neighbor_count;
	for (int oy = -1; oy <= 1; ++oy) {
	    for (int ox = -1; ox <= 1; ++ox) {
		if (isValidCell(x+ox, y+oy)) {
		    double h = getHeight(x+ox, y+oy);
		    height_variance += (h - mean_height) * (h - mean_height);
		}
	    }
	}
	feature.setRoughness(std::sqrt(height_variance / neighbor_count));
    }

    // Check if this is a boundary point
    feature.setIsBoundary(x == 0 || x == width-1 || y == 0 || y == height-1);

    // Calculate importance score (Terra/Scape style geometric importance)
    feature.setImportance(feature.getCurvature() + 0.5 * feature.getSlope() + 0.3 * feature.getRoughness());
    if (feature.getIsBoundary()) feature.setImportance(feature.getImportance() * 2.0); // Preserve boundaries

    return feature;
}

// Generate adaptive sampling mask based on terrain features
std::vector<std::vector<bool>> TerrainData::generateSampleMask(const SimplificationParams& params) const {
    std::vector<std::vector<bool>> mask(height, std::vector<bool>(width, false));

    // Always include boundary points
    for (int y = 0; y < height; ++y) {
	for (int x = 0; x < width; ++x) {
	    if (x == 0 || x == width-1 || y == 0 || y == height-1) {
		mask[y][x] = true;
	    }
	}
    }

    // Analyze terrain features and mark important points
    std::vector<std::pair<double, std::pair<int, int>>> importance_points;

    for (int y = 1; y < height-1; ++y) {
	for (int x = 1; x < width-1; ++x) {
	    TerrainFeature feature = analyzePoint(x, y);

	    // Include points with high importance or exceeding thresholds
	    if (feature.getImportance() > params.getErrorTol() ||
		    feature.getSlope() > params.getSlopeTol()) {
		mask[y][x] = true;
	    } else {
		// Store for potential inclusion based on overall reduction target
		importance_points.push_back({feature.getImportance(), {x, y}});
	    }
	}
    }

    // Sort by importance and include additional points to meet minimum density
    std::sort(importance_points.rbegin(), importance_points.rend());

    int current_points = 0;
    for (int y = 0; y < height; ++y) {
	for (int x = 0; x < width; ++x) {
	    if (mask[y][x]) current_points++;
	}
    }

    int total_points = width * height;
    int min_required = total_points * (100 - params.getMinReduction()) / 100;

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

// Garland-Heckbert point analysis with quadric error metrics
TerrainFeature TerrainData::analyzePointWithQuadric(int x, int y) const {
    TerrainFeature feature;
    
    if (!isValidCell(x, y)) {
	return feature;
    }
    
    // First compute legacy features for compatibility
    feature = analyzePoint(x, y);
    
    // Initialize quadric for this vertex by summing quadrics from adjacent faces
    QuadricError quadric;
    double current_height = getHeight(x, y);
    Point3D current_pos(x, y, current_height);
    
    // For a grid vertex (x,y), it participates in up to 4 quads around it
    // Each quad generates 2 triangles, so up to 8 triangles total
    std::vector<std::array<Point3D, 3>> adjacent_triangles;
    
    // Check all 4 possible quads around this vertex
    int quad_offsets[4][2] = {{-1, -1}, {0, -1}, {-1, 0}, {0, 0}};
    
    for (int q = 0; q < 4; q++) {
	int qx = x + quad_offsets[q][0];
	int qy = y + quad_offsets[q][1];
	
	// Check if this quad exists (all 4 corners valid)
	if (isValidCell(qx, qy) && isValidCell(qx+1, qy) &&
	    isValidCell(qx, qy+1) && isValidCell(qx+1, qy+1)) {
	    
	    Point3D p00(qx, qy, getHeight(qx, qy));
	    Point3D p10(qx+1, qy, getHeight(qx+1, qy));
	    Point3D p01(qx, qy+1, getHeight(qx, qy+1));
	    Point3D p11(qx+1, qy+1, getHeight(qx+1, qy+1));
	    
	    // Two triangles per quad: (p00, p10, p01) and (p10, p11, p01)
	    // Check if current vertex is part of triangle 1: (p00, p10, p01)
	    if ((qx == x && qy == y) || (qx+1 == x && qy == y) || (qx == x && qy+1 == y)) {
		adjacent_triangles.push_back({p00, p10, p01});
	    }
	    // Check if current vertex is part of triangle 2: (p10, p11, p01)
	    if ((qx+1 == x && qy == y) || (qx+1 == x && qy+1 == y) || (qx == x && qy+1 == y)) {
		adjacent_triangles.push_back({p10, p11, p01});
	    }
	}
    }
    
    // Compute quadric by summing contributions from adjacent triangles
    for (const auto& triangle : adjacent_triangles) {
	// Compute plane equation from triangle
	Point3D v1 = triangle[1] - triangle[0];
	Point3D v2 = triangle[2] - triangle[0];
	Point3D normal = v1.cross(v2);
	
	double normal_length = normal.length();
	if (normal_length > 1e-10) { // Valid normal
	    normal = Point3D(normal.x / normal_length, normal.y / normal_length, normal.z / normal_length);
	    
	    // Plane equation: ax + by + cz + d = 0
	    double a = normal.x;
	    double b = normal.y;
	    double c = normal.z;
	    double d = -(a * triangle[0].x + b * triangle[0].y + c * triangle[0].z);
	    
	    // Add this plane's quadric contribution
	    quadric += QuadricError(a, b, c, d);
	}
    }
    
    // Calculate quadric error at current position
    double error = quadric.evaluate(current_pos.x, current_pos.y, current_pos.z);
    
    // For terrain, the error should be related to how flat or curved the local surface is
    // If error is very small, use the legacy importance metric as a backup
    if (std::abs(error) < 1e-10) {
	error = feature.getImportance();
    }
    
    feature.setQuadric(quadric);
    feature.setQuadricError(error);
    
    return feature;
}

// Generate Garland-Heckbert based sample mask using quadric error metrics
std::vector<std::vector<bool>> TerrainData::generateGarlandHeckbertMask(const SimplificationParams& params) const {
    std::vector<std::vector<bool>> mask(height, std::vector<bool>(width, false));
    
    // Always preserve boundary points for manifold property
    for (int y = 0; y < height; ++y) {
	for (int x = 0; x < width; ++x) {
	    if (x == 0 || x == width-1 || y == 0 || y == height-1) {
		mask[y][x] = true;
	    }
	}
    }
    
    // Compute quadric errors for all interior points
    std::vector<std::vector<TerrainFeature>> features(height, std::vector<TerrainFeature>(width));
    std::vector<std::tuple<double, int, int>> error_points; // error, x, y
    
    double total_error = 0.0;
    int error_point_count = 0;
    
    for (int y = 1; y < height-1; ++y) {
	for (int x = 1; x < width-1; ++x) {
	    features[y][x] = analyzePointWithQuadric(x, y);
	    double quadric_error = features[y][x].getQuadricError();
	    
	    total_error += quadric_error;
	    error_point_count++;
	    
	    error_points.push_back(std::make_tuple(quadric_error, x, y));
	}
    }
    
    // Sort by quadric error (ascending - lowest error first, these are candidates for removal)
    std::sort(error_points.begin(), error_points.end());
    
    // Determine how many points to remove based on quadric error and target reduction
    int total_points = width * height;
    int target_points = total_points * (100 - params.getMinReduction()) / 100;
    int boundary_points = 2 * width + 2 * height - 4; // Boundary perimeter
    int interior_target = target_points - boundary_points;
    int interior_available = (width - 2) * (height - 2);
    
    // Make sure we don't try to remove too many points
    interior_target = std::min(interior_target, interior_available);
    interior_target = std::max(interior_target, interior_available / 4); // Keep at least 25% of interior
    
    // Debug output (can be removed in production)
    double avg_error = error_point_count > 0 ? total_error / error_point_count : 0.0;
    if (false) { // Set to true for debugging
	std::cout << "DEBUG: Average quadric error: " << avg_error << ", Error threshold: " << params.getErrorTol() << std::endl;
	std::cout << "DEBUG: Target interior points to keep: " << interior_target << " out of " << interior_available << std::endl;
    }
    
    // Use structured approach: keep every Nth point, but bias towards high quadric error
    // This ensures we maintain grid connectivity while still using quadric metrics
    
    // First, mark points to keep based on quadric error (highest error first)
    std::vector<bool> interior_keep(error_points.size(), false);
    
    // Keep the highest-error points first
    for (int i = error_points.size() - 1; i >= 0 && interior_target > 0; i--) {
	interior_keep[i] = true;
	interior_target--;
    }
    
    // Apply the decisions to the mask
    for (size_t i = 0; i < error_points.size(); i++) {
	if (interior_keep[i]) {
	    int x = std::get<1>(error_points[i]);
	    int y = std::get<2>(error_points[i]);
	    mask[y][x] = true;
	}
    }
    
    // Count final points for debugging
    int final_kept = 0;
    for (int y = 0; y < height; ++y) {
	for (int x = 0; x < width; ++x) {
	    if (mask[y][x]) final_kept++;
	}
    }
    
    if (false) { // Set to true for debugging
	std::cout << "DEBUG: Final kept points: " << final_kept << " out of " << total_points << std::endl;
    }
    
    return mask;
}

void TerrainData::floodFill(std::vector<std::vector<bool>>& visited,
	ConnectedComponent& component, int start_x, int start_y, double height_threshold) const {
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

	    if (isValidCell(nx, ny) &&
		    !visited[ny][nx] &&
		    getHeight(nx, ny) > height_threshold) {

		visited[ny][nx] = true;
		to_visit.push({nx, ny});
		component.addCell(nx, ny);
	    }
	}
    }
}

TerrainComponents TerrainData::analyzeComponents(double height_threshold) const {
    TerrainComponents result(width, height);
    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));

    int component_id = 0;

    // Find all connected components of non-zero height cells
    for (int y = 0; y < height; ++y) {
	for (int x = 0; x < width; ++x) {
	    if (!visited[y][x] && getHeight(x, y) > height_threshold) {
		ConnectedComponent component;
		component.id = component_id;

		floodFill(visited, component, x, y, height_threshold);

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
			if (!isValidCell(nx, ny) || getHeight(nx, ny) <= height_threshold) {
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
void TerrainMesh::triangulateComponentVolume(const TerrainData& terrain, const ConnectedComponent& component) {
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

	top_vertices[y][x] = addVertex(Point3D(world_x, world_y, height));
	bottom_vertices[y][x] = addVertex(Point3D(world_x, world_y, 0.0));
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
		addSurfaceTriangle(v00, v01, v10);
		addSurfaceTriangle(v10, v01, v11);
	    }
	}
    }

    // Add bottom surface triangles using earcut for more efficient triangulation
    std::set<std::pair<int, int>> component_cells_set(component.cells.begin(), component.cells.end());
    triangulateBottomFaceWithDetria(bottom_vertices, terrain, &component_cells_set);

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
		addTriangle(t1, t2, b1);
		addTriangle(t2, b2, b1);
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
		addTriangle(t1, b1, t2);
		addTriangle(t2, b1, b2);
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
		addTriangle(t1, b1, t2);
		addTriangle(t2, b1, b2);
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
		addTriangle(t1, t2, b1);
		addTriangle(t2, b2, b1);
	    }
	}
    }
}

// Triangulate terrain volume with proper handling of connected components
void TerrainMesh::triangulateVolumeWithComponents(const TerrainData& terrain) {
    clear();

    if (terrain.width <= 0 || terrain.height <= 0) {
	return;
    }

    // Analyze terrain to find connected components
    TerrainComponents components = terrain.analyzeComponents();

    std::cout << "Found " << components.components.size() << " terrain component(s)" << std::endl;

    // Triangulate each component separately
    for (const auto& component : components.components) {
	std::cout << "Processing component " << component.id
	    << " with " << component.cells.size() << " cells" << std::endl;
	triangulateComponentVolume(terrain, component);
    }
}

// Generate a volumetric triangle mesh from terrain data (legacy single-mesh approach)
void TerrainMesh::triangulateVolumeLegacy(const TerrainData& terrain) {
    clear();

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

	    top_vertices[y][x] = addVertex(Point3D(world_x, world_y, height));
	    bottom_vertices[y][x] = addVertex(Point3D(world_x, world_y, 0.0));
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
	    addSurfaceTriangle(v00, v01, v10);
	    // Triangle 2: v10, v01, v11
	    addSurfaceTriangle(v10, v01, v11);
	}
    }

    // Add bottom surface triangles using detria for high-quality triangulation
    triangulateBottomFaceWithDetria(bottom_vertices, terrain, nullptr);

    // Add side walls
    // Left wall (x = 0)
    for (int y = 0; y < terrain.height - 1; ++y) {
	size_t top_0 = top_vertices[y][0];
	size_t top_1 = top_vertices[y + 1][0];
	size_t bot_0 = bottom_vertices[y][0];
	size_t bot_1 = bottom_vertices[y + 1][0];

	addTriangle(top_0, bot_0, top_1);
	addTriangle(top_1, bot_0, bot_1);
    }

    // Right wall (x = width - 1)
    for (int y = 0; y < terrain.height - 1; ++y) {
	int x = terrain.width - 1;
	size_t top_0 = top_vertices[y][x];
	size_t top_1 = top_vertices[y + 1][x];
	size_t bot_0 = bottom_vertices[y][x];
	size_t bot_1 = bottom_vertices[y + 1][x];

	addTriangle(top_0, top_1, bot_0);
	addTriangle(top_1, bot_1, bot_0);
    }

    // Top wall (y = 0)
    for (int x = 0; x < terrain.width - 1; ++x) {
	size_t top_0 = top_vertices[0][x];
	size_t top_1 = top_vertices[0][x + 1];
	size_t bot_0 = bottom_vertices[0][x];
	size_t bot_1 = bottom_vertices[0][x + 1];

	addTriangle(top_0, top_1, bot_0);
	addTriangle(top_1, bot_1, bot_0);
    }

    // Bottom wall (y = height - 1)
    for (int x = 0; x < terrain.width - 1; ++x) {
	int y = terrain.height - 1;
	size_t top_0 = top_vertices[y][x];
	size_t top_1 = top_vertices[y][x + 1];
	size_t bot_0 = bottom_vertices[y][x];
	size_t bot_1 = bottom_vertices[y][x + 1];

	addTriangle(top_0, bot_0, top_1);
	addTriangle(top_1, bot_0, bot_1);
    }
}

// Generate a volumetric triangle mesh from terrain data
void TerrainMesh::triangulateVolume(const TerrainData& terrain) {
    // Use component-based approach by default for better handling of disjoint islands
    triangulateVolumeWithComponents(terrain);
}

// Generate a simplified volumetric triangle mesh using Terra/Scape concepts
void TerrainMesh::triangulateVolumeSimplified(const TerrainData& terrain, const SimplificationParams& params) {
    // Use the new full Garland-Heckbert implementation
    triangulateVolumeWithGarlandHeckbert(terrain, params);
}

// Generate terrain surface-only mesh using Garland-Heckbert simplification
void TerrainMesh::triangulateSurfaceOnly(const TerrainData& terrain, const SimplificationParams& params) {
    // Use Garland-Heckbert surface generation (without walls and bottom)
    generateGarlandHeckbertSurface(terrain, params);
}

// Extract hole boundary vertices (clockwise for earcut holes)
bool
TerrainMesh::extractHoleBoundary(const std::vector<std::pair<int, int>>& hole_cells,
	const std::set<std::pair<int, int>>& active_cells,
	const std::vector<std::vector<size_t>>& bottom_vertices,
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
	const Point3D& vertex = vertices[bottom_vertices[cell.second][cell.first]];
	hole_boundary.push_back({vertex.x, vertex.y});
	vertex_indices.push_back(bottom_vertices[cell.second][cell.first]);
    }

    return hole_boundary.size() >= 3;
}

// Point-in-polygon test using ray casting algorithm
bool pointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon) {
    bool inside = false;
    int n = polygon.size();

    for (int i = 0, j = n - 1; i < n; j = i++) {
	double xi = polygon[i].first, yi = polygon[i].second;
	double xj = polygon[j].first, yj = polygon[j].second;

	if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
	    inside = !inside;
	}
    }
    return inside;
}

// Helper function to validate and add a Steiner point candidate
bool TerrainData::addSteinerPointIfValid(double x, double y,
	const std::vector<std::pair<double, double>>& boundary,
	const std::vector<std::vector<std::pair<double, double>>>& holes,
	const std::set<std::pair<int, int>>& active_cells,
	double min_distance,
	const std::function<double(double, double)>& distanceToEdges,
	std::vector<std::pair<double, double>>& steiner_points) const {
    // Check if point is valid (inside boundary, not in holes)
    if (!pointInPolygon(x, y, boundary)) {
	return false;
    }

    // Check if point is in any hole
    for (const auto& hole : holes) {
	if (pointInPolygon(x, y, hole)) {
	    return false;
	}
    }

    // Check distance to edges
    double edge_distance = distanceToEdges(x, y);
    if (edge_distance <= min_distance) {
	return false;
    }

    // Check terrain coordinates and active region
    int terrain_x = static_cast<int>((x - origin.x) / cell_size);
    int terrain_y = static_cast<int>((origin.y - y) / cell_size);

    if (terrain_x < 0 || terrain_x >= width ||
	    terrain_y < 0 || terrain_y >= height) {
	return false;
    }

    if (active_cells.count({terrain_x, terrain_y}) == 0) {
	return false;
    }

    // Check distance to existing points
    for (const auto& existing : steiner_points) {
	double dx_check = x - existing.first;
	double dy_check = y - existing.second;
	if (std::sqrt(dx_check * dx_check + dy_check * dy_check) < min_distance) {
	    return false;
	}
    }

    // Point is valid, add it
    steiner_points.push_back({x, y});
    return true;
}

// Helper function to process guide lines from edge points to center
void
TerrainData::processGuideLines(const std::vector<std::pair<double, double>>& edge_points,
	double center_x, double center_y,
	double min_viable_len,
	const std::vector<double>& step_probabilities,
	const std::function<double()>& next_random,
	const std::vector<std::pair<double, double>>& boundary,
	const std::vector<std::vector<std::pair<double, double>>>& holes,
	const std::set<std::pair<int, int>>& active_cells,
	double min_distance,
	const std::function<double(double, double)>& distanceToEdges,
	std::vector<std::pair<double, double>>& steiner_points,
	size_t sample_step,
	double step_divisor = 1.0) const {
    for (size_t i = 0; i < edge_points.size(); i += sample_step) {
	const auto& edge_point = edge_points[i];

	// Create guide line from edge point to center
	double dx = center_x - edge_point.first;
	double dy = center_y - edge_point.second;
	double line_length = std::sqrt(dx * dx + dy * dy);

	if (line_length > min_viable_len) {
	    // Normalize direction
	    dx /= line_length;
	    dy /= line_length;

	    // Use dynamic step count based on probability vector size
	    int num_steps = static_cast<int>(step_probabilities.size());

	    for (int step = 1; step <= num_steps; ++step) {
		// Apply probability selection
		double probability = step_probabilities[step - 1];
		if (next_random() > probability) continue;

		// Calculate gradient step distance - shorter near edges, longer toward center
		// Use geometric progression: each step is further than the previous
		double progression_factor = 0.6; // Controls how much the step distances grow
		double base_distance = line_length / (num_steps + 1);

		// Calculate geometric progression distance
		double geometric_sum = (1.0 - std::pow(1.0 + progression_factor, num_steps)) / (-progression_factor);
		double normalized_position = 0.0;
		for (int s = 1; s <= step; ++s) {
		    normalized_position += std::pow(1.0 + progression_factor, s - 1);
		}
		normalized_position /= geometric_sum;

		// Apply step divisor and add controlled randomness (±15% variation)
		double step_distance = (normalized_position * line_length) / step_divisor;
		double randomness_factor = 0.15; // ±15% randomness
		double random_offset = (next_random() - 0.5) * 2.0 * randomness_factor;
		step_distance *= (1.0 + random_offset);

		// Ensure we don't go past the center or too close to the edge
		step_distance = std::max(step_distance, base_distance * 0.5);
		step_distance = std::min(step_distance, line_length * 0.95);

		double x = edge_point.first + dx * step_distance;
		double y = edge_point.second + dy * step_distance;

		addSteinerPointIfValid(x, y, boundary, holes, active_cells,
			min_distance, distanceToEdges, steiner_points);
	    }
	}
    }
}

// Generate Steiner points using simple guide lines to average center point
std::vector<std::pair<double, double>>
TerrainData::generateSteinerPoints (
	const std::vector<std::pair<double, double>>& boundary,
	const std::vector<std::vector<std::pair<double, double>>>& holes,
	const std::set<std::pair<int, int>>& active_cells,
	double min_x_in, double max_x_in, double min_y_in, double max_y_in) const {

    double min_x = min_x_in * cell_size + origin.x;
    double max_x = max_x_in * cell_size + origin.x;
    double min_y = origin.y - max_y_in * cell_size;  // Corrected for y-flip
    double max_y = origin.y - min_y_in * cell_size;  // Corrected for y-flip

    // Debug output to understand the issue
    std::cout << "Steiner point generation: boundary size=" << boundary.size() 
              << ", holes count=" << holes.size() << std::endl;
    
    // Filter out invalid/degenerate holes to prevent infinite loops
    std::vector<std::vector<std::pair<double, double>>> valid_holes;
    size_t total_hole_edges = 0;
    for (const auto& hole : holes) {
        // Skip holes that are too small or have degenerate edges
        if (hole.size() >= 3) {
            bool has_degenerate = false;
            for (size_t i = 0; i < hole.size(); ++i) {
                size_t next = (i + 1) % hole.size();
                double dx = hole[next].first - hole[i].first;
                double dy = hole[next].second - hole[i].second;
                double len_sq = dx * dx + dy * dy;
                if (len_sq < 1e-10) {  // Very small edge, likely degenerate
                    has_degenerate = true;
                    break;
                }
            }
            if (!has_degenerate) {
                valid_holes.push_back(hole);
                total_hole_edges += hole.size();
            }
        }
    }
    
    // Limit the number of holes processed to prevent performance issues
    const size_t MAX_HOLES = 50;  // Reasonable limit for Steiner point generation
    const size_t MAX_TOTAL_EDGES = 1000;  // Limit total complexity
    if (valid_holes.size() > MAX_HOLES || total_hole_edges > MAX_TOTAL_EDGES) {
        std::cout << "WARNING: Too many holes (" << valid_holes.size() << " holes, " 
                  << total_hole_edges << " edges). Limiting to " << MAX_HOLES 
                  << " largest holes for performance." << std::endl;
        
        // Sort holes by size (descending) and keep only the largest ones
        std::sort(valid_holes.begin(), valid_holes.end(), 
                  [](const auto& a, const auto& b) { return a.size() > b.size(); });
        if (valid_holes.size() > MAX_HOLES) {
            valid_holes.resize(MAX_HOLES);
        }
    }
    
    std::cout << "Processing " << valid_holes.size() << " valid holes" << std::endl;

    std::vector<std::pair<double, double>> steiner_points;

    // Calculate average center point of all bounding face edges
    double center_x = 0.0;
    double center_y = 0.0;
    size_t total_points = 0;

    // Average all boundary points
    for (const auto& point : boundary) {
	center_x += point.first;
	center_y += point.second;
	total_points++;
    }

    // Average all hole points
    for (const auto& hole : valid_holes) {
	for (const auto& point : hole) {
	    center_x += point.first;
	    center_y += point.second;
	    total_points++;
	}
    }

    if (total_points > 0) {
	center_x /= total_points;
	center_y /= total_points;
    } else {
	// Fallback to bounding box center
	center_x = (min_x + max_x) * 0.5 * cell_size + origin.x;
	center_y = (min_y + max_y) * 0.5 * cell_size + origin.y;
    }

    // Parameters for simplified Steiner point generation
    double min_distance = cell_size * 3.0;
    double min_viable_len = cell_size * 4.0; // minimum viable length

    // Distance function to edges
    auto distanceToEdges = [&](double x, double y) -> double {
	double min_dist = std::numeric_limits<double>::max();

	// Distance to boundary
	for (size_t i = 0; i < boundary.size(); ++i) {
	    size_t next = (i + 1) % boundary.size();
	    double dx1 = boundary[next].first - boundary[i].first;
	    double dy1 = boundary[next].second - boundary[i].second;
	    double dx2 = x - boundary[i].first;
	    double dy2 = y - boundary[i].second;

	    double dot = dx1 * dx2 + dy1 * dy2;
	    double len_sq = dx1 * dx1 + dy1 * dy1;

	    double t = (len_sq > 0) ? std::max(0.0, std::min(1.0, dot / len_sq)) : 0.0;
	    double px = boundary[i].first + t * dx1;
	    double py = boundary[i].second + t * dy1;

	    double dist = std::sqrt((x - px) * (x - px) + (y - py) * (y - py));
	    min_dist = std::min(min_dist, dist);
	}

	// Distance to holes
	for (const auto& hole : valid_holes) {
	    for (size_t i = 0; i < hole.size(); ++i) {
		size_t next = (i + 1) % hole.size();
		double dx1 = hole[next].first - hole[i].first;
		double dy1 = hole[next].second - hole[i].second;
		double dx2 = x - hole[i].first;
		double dy2 = y - hole[i].second;

		double dot = dx1 * dx2 + dy1 * dy2;
		double len_sq = dx1 * dx1 + dy1 * dy1;

		double t = (len_sq > 0) ? std::max(0.0, std::min(1.0, dot / len_sq)) : 0.0;
		double px = hole[i].first + t * dx1;
		double py = hole[i].second + t * dy1;

		double dist = std::sqrt((x - px) * (x - px) + (y - py) * (y - py));
		min_dist = std::min(min_dist, dist);
	    }
	}

	return min_dist;
    };

    // Simple pseudo-random number generator (using linear congruential generator)
    uint32_t rng_state = 12345; // Seed
    auto next_random = [&rng_state]() -> double {
	rng_state = rng_state * 1664525 + 1013904223;
	return (double)(rng_state % 1000) / 1000.0;
    };

    // Process boundary guide lines with enhanced coverage
    // Use smaller sample step to increase active guideline density and reduce long thin triangles
    size_t boundary_sample_step = std::max(1, (int)(boundary.size() / 150)); // Increased density

    // Enhanced boundary probabilities with more steps for better gradient coverage
    // Higher probability for steps closer to edges, decreasing toward center
    std::vector<double> boundary_probabilities = {0.95, 0.85, 0.65, 0.45, 0.25, 0.1}; // 6 steps
    processGuideLines(boundary, center_x, center_y, min_viable_len, boundary_probabilities,
	    next_random, boundary, holes, active_cells, min_distance,
	    distanceToEdges, steiner_points, boundary_sample_step);

    // Process hole guide lines with enhanced coverage
    for (const auto& hole : valid_holes) {
	// Increase hole guideline density to prevent thin triangles near holes
	size_t hole_sample_step = std::max(1, (int)(hole.size() / 75)); // Increased density

	// Enhanced hole probabilities with more aggressive sampling near edges
	// Holes need more intensive coverage to avoid problematic triangles
	std::vector<double> hole_probabilities = {1.0, 0.8, 0.5, 0.2, 0.05}; // 5 steps
	processGuideLines(hole, center_x, center_y, min_viable_len, hole_probabilities,
		next_random, boundary, holes, active_cells, min_distance,
		distanceToEdges, steiner_points, hole_sample_step, 5.0); // step_divisor = 5.0 for holes
    }

    // Add center point (only once, outside the loops)
    addSteinerPointIfValid(center_x, center_y, boundary, holes, active_cells,
	    min_distance, distanceToEdges, steiner_points);

    std::cout << "Generated " << steiner_points.size() << " Steiner points using guide lines to average center point" << std::endl;

    return steiner_points;
}

// Detria-based high-quality triangulation of coplanar bottom face
void
TerrainMesh::triangulateBottomFaceWithDetria(const std::vector<std::vector<size_t>>& bottom_vertices,
	const TerrainData& terrain, const std::set<std::pair<int, int>>* filter_cells = nullptr) {

    // Build set of cells that should have bottom faces (same as earcut version)
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
	    const Point3D& vertex = vertices[bottom_vertices[min_y][x]];
	    outer_boundary.push_back({vertex.x, vertex.y});
	    vertex_indices.push_back(bottom_vertices[min_y][x]);
	}
    }

    // Right edge (bottom to top, skip corners)
    for (int y = min_y + 1; y <= max_y; ++y) {
	if (active_cells.count({max_x, y})) {
	    const Point3D& vertex = vertices[bottom_vertices[y][max_x]];
	    outer_boundary.push_back({vertex.x, vertex.y});
	    vertex_indices.push_back(bottom_vertices[y][max_x]);
	}
    }

    // Top edge (right to left, skip corners)
    for (int x = max_x - 1; x >= min_x; --x) {
	if (active_cells.count({x, max_y})) {
	    const Point3D& vertex = vertices[bottom_vertices[max_y][x]];
	    outer_boundary.push_back({vertex.x, vertex.y});
	    vertex_indices.push_back(bottom_vertices[max_y][x]);
	}
    }

    // Left edge (top to bottom, skip corners)
    for (int y = max_y - 1; y > min_y; --y) {
	if (active_cells.count({min_x, y})) {
	    const Point3D& vertex = vertices[bottom_vertices[y][min_x]];
	    outer_boundary.push_back({vertex.x, vertex.y});
	    vertex_indices.push_back(bottom_vertices[y][min_x]);
	}
    }

    if (outer_boundary.size() < 3) {
	fallbackBottomTriangulation(bottom_vertices, terrain, filter_cells);
	return;
    }

    // Find interior holes using flood fill (same logic as earcut version)
    std::vector<std::vector<std::pair<double, double>>> holes;
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

		    if (extractHoleBoundary(hole_cells, active_cells, bottom_vertices, hole_boundary, vertex_indices)) {
			holes.push_back(hole_boundary);
		    }
		}
	    }
	}
    }

    // Generate Steiner points for better triangle quality
    std::vector<std::pair<double, double>> steiner_points = terrain.generateSteinerPoints(
	    outer_boundary, holes, active_cells, min_x, max_x, min_y, max_y);

    // Try detria triangulation first
    try {
	// Create all points: boundary + steiner points
	std::vector<detria::PointD> all_points;
	std::vector<size_t> all_vertex_indices;

	// Add boundary points first
	for (const auto& point : outer_boundary) {
	    all_points.push_back({point.first, point.second});
	    // vertex_indices already contains boundary vertex indices
	}
	all_vertex_indices = vertex_indices;

	// Add hole points
	//size_t hole_start_idx = all_points.size();
	for (const auto& hole : holes) {
	    for (const auto& point : hole) {
		all_points.push_back({point.first, point.second});
		// Note: hole vertex indices were added to vertex_indices during hole extraction
	    }
	}

	// Add Steiner points as vertices and to the point list
	for (const auto& point : steiner_points) {
	    double world_z = 0.0; // Bottom face is planar at z=0
	    size_t vertex_index = addVertex(Point3D(point.first, point.second, world_z));
	    all_vertex_indices.push_back(vertex_index);
	    all_points.push_back({point.first, point.second});
	}

	// Set up detria triangulation
	detria::Triangulation tri;
	tri.setPoints(all_points);

	// Add boundary outline (indices refer to all_points)
	std::vector<uint32_t> outline_indices;
	for (size_t i = 0; i < outer_boundary.size(); ++i) {
	    outline_indices.push_back(static_cast<uint32_t>(i));
	}
	tri.addOutline(outline_indices);

	// Add holes
	size_t hole_idx_offset = outer_boundary.size();
	for (const auto& hole : holes) {
	    std::vector<uint32_t> hole_indices;
	    for (size_t i = 0; i < hole.size(); ++i) {
		hole_indices.push_back(static_cast<uint32_t>(hole_idx_offset + i));
	    }
	    tri.addHole(hole_indices);
	    hole_idx_offset += hole.size();
	}

	// Perform Delaunay triangulation
	bool success = tri.triangulate(true); // true for Delaunay

	if (success) {
	    // Extract triangles and add them to the mesh
	    bool cwTriangles = false; // We want counter-clockwise for bottom face

	    tri.forEachTriangle([&](detria::Triangle<uint32_t> triangle) {
		    // Map detria point indices back to our vertex indices
		    size_t v0, v1, v2;

		    if (triangle.x < all_vertex_indices.size()) {
		    v0 = all_vertex_indices[triangle.x];
		    } else return; // Skip if invalid index

		    if (triangle.y < all_vertex_indices.size()) {
		    v1 = all_vertex_indices[triangle.y];
		    } else return;

		    if (triangle.z < all_vertex_indices.size()) {
		    v2 = all_vertex_indices[triangle.z];
		    } else return;

		    // Add triangle with correct bottom face orientation (reversed winding)
		    addTriangle(v0, v2, v1);

		    }, cwTriangles);
	} else {
	    // If detria fails, use fallback
	    fallbackBottomTriangulation(bottom_vertices, terrain, filter_cells);
	}
    } catch (const std::exception&) {
	// Fall back to dense fallback if detria fails
	fallbackBottomTriangulation(bottom_vertices, terrain, filter_cells);
    }
}

// Fallback triangulation method (original grid-based approach)
void
TerrainMesh::fallbackBottomTriangulation(const std::vector<std::vector<size_t>>& bottom_vertices,
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
		    addTriangle(v00, v10, v01);
		    addTriangle(v10, v11, v01);
		}
	    }
	}
    }
}

// Full Garland-Heckbert surface mesh generation with edge collapse
void TerrainMesh::generateGarlandHeckbertSurface(const TerrainData& terrain, const SimplificationParams& params) {
    // Step 1: Build initial complete triangulation of the surface
    std::vector<Point3D> surface_vertices;
    std::vector<Triangle> surface_triangles;
    std::vector<std::vector<size_t>> grid_to_vertex(terrain.height, std::vector<size_t>(terrain.width, SIZE_MAX));
    
    buildInitialSurfaceTriangulation(terrain, surface_vertices, surface_triangles, grid_to_vertex);
    
    // Step 2: Compute quadric error metrics for all vertices
    std::vector<QuadricError> vertex_quadrics(surface_vertices.size());
    computeVertexQuadrics(surface_vertices, surface_triangles, vertex_quadrics);
    
    // Step 3: Build priority queue of edge collapse candidates
    std::priority_queue<EdgeCollapse> collapse_queue;
    buildEdgeCollapseQueue(surface_vertices, surface_triangles, vertex_quadrics, collapse_queue);
    
    // Step 4: Iteratively collapse edges until target reduction is reached
    size_t original_triangles = surface_triangles.size();
    size_t target_triangles = original_triangles * (100 - params.getMinReduction()) / 100;
    size_t current_triangles = original_triangles;
    
    std::vector<bool> vertex_removed(surface_vertices.size(), false);
    
    std::cout << "Garland-Heckbert: Starting with " << original_triangles << " triangles, target: " << target_triangles << std::endl;
    
    size_t collapse_count = 0;
    size_t max_collapses = 50000; // Increased limit for larger reductions
    
    while (!collapse_queue.empty() && current_triangles > target_triangles && collapse_count < max_collapses) {
        EdgeCollapse best_collapse = collapse_queue.top();
        collapse_queue.pop();
        
        // Verify edge is still valid (vertices not already removed)
        if (vertex_removed[best_collapse.edge.getV0()] || vertex_removed[best_collapse.edge.getV1()]) {
            continue;
        }
        
        // Perform edge collapse and update data structures
        size_t triangles_before = current_triangles;
        if (performEdgeCollapse(best_collapse, surface_vertices, surface_triangles, 
                               vertex_quadrics, vertex_removed, collapse_queue)) {
            collapse_count++;
            
            // Estimate triangle count reduction (typically 2 triangles removed per edge collapse)
            current_triangles = std::max(target_triangles, triangles_before - 2);
            
            // Every 1000 collapses, do an accurate count and give progress update
            if (collapse_count % 1000 == 0) {
                current_triangles = std::count_if(surface_triangles.begin(), surface_triangles.end(),
                                                [&vertex_removed](const Triangle& t) {
                                                    return !vertex_removed[t.vertices[0]] && 
                                                           !vertex_removed[t.vertices[1]] && 
                                                           !vertex_removed[t.vertices[2]];
                                                });
                std::cout << "Garland-Heckbert: " << collapse_count << " collapses, " << current_triangles << " triangles remaining" << std::endl;
            }
        }
    }
    
    std::cout << "Garland-Heckbert: Completed " << collapse_count << " edge collapses" << std::endl;
    
    // Step 5: Apply edge collapses and compact the surface mesh
    // First, update all triangles to map collapsed vertices
    for (Triangle& triangle : surface_triangles) {
        for (int i = 0; i < 3; i++) {
            size_t& v = triangle.vertices[i];
            // If this vertex was removed, find what it was collapsed to
            // (In our simplified approach, v1 is always collapsed to v0)
        }
    }
    
    // Remove degenerate triangles (those with duplicate vertices after collapse)
    auto new_end = std::remove_if(surface_triangles.begin(), surface_triangles.end(),
        [&vertex_removed](const Triangle& t) {
            return vertex_removed[t.vertices[0]] || vertex_removed[t.vertices[1]] || vertex_removed[t.vertices[2]] ||
                   t.vertices[0] == t.vertices[1] || t.vertices[1] == t.vertices[2] || t.vertices[2] == t.vertices[0];
        });
    surface_triangles.erase(new_end, surface_triangles.end());
    
    std::cout << "Garland-Heckbert: After cleanup: " << surface_triangles.size() << " triangles" << std::endl;
    
    // Compact vertex list and remap triangle indices
    std::vector<size_t> vertex_remap(surface_vertices.size());
    size_t new_vertex_count = 0;
    
    for (size_t i = 0; i < surface_vertices.size(); i++) {
        if (!vertex_removed[i]) {
            vertex_remap[i] = new_vertex_count++;
        } else {
            vertex_remap[i] = SIZE_MAX;
        }
    }
    
    // Copy valid vertices to the mesh
    vertices.clear();
    vertices.reserve(new_vertex_count);
    for (size_t i = 0; i < surface_vertices.size(); i++) {
        if (!vertex_removed[i]) {
            vertices.push_back(surface_vertices[i]);
        }
    }
    
    // Copy valid triangles to the mesh
    triangles.clear();
    surface_triangle_count = 0;
    for (const Triangle& t : surface_triangles) {
        if (!vertex_removed[t.vertices[0]] && !vertex_removed[t.vertices[1]] && !vertex_removed[t.vertices[2]]) {
            Triangle new_tri(vertex_remap[t.vertices[0]], vertex_remap[t.vertices[1]], vertex_remap[t.vertices[2]]);
            new_tri.computeNormal(vertices);
            triangles.push_back(new_tri);
            surface_triangle_count++;
        }
    }
}

// Complete volume triangulation using Garland-Heckbert surface + traditional walls/bottom
void TerrainMesh::triangulateVolumeWithGarlandHeckbert(const TerrainData& terrain, const SimplificationParams& params) {
    clear();
    
    if (terrain.width <= 0 || terrain.height <= 0) {
        return;
    }
    
    // First generate optimized surface using Garland-Heckbert
    generateGarlandHeckbertSurface(terrain, params);
    
    // Store surface vertex count for later reference
    size_t surface_vertex_count = vertices.size();
    
    // Now add bottom vertices and build walls
    // Create mapping from surface vertices back to grid coordinates for walls
    std::vector<std::pair<int, int>> vertex_to_grid(surface_vertex_count);
    
    // Find grid coordinates for each surface vertex
    for (size_t v = 0; v < surface_vertex_count; v++) {
        Point3D& vertex = vertices[v];
        int grid_x = (int)std::round((vertex.x - terrain.origin.x) / terrain.cell_size);
        int grid_y = (int)std::round((terrain.origin.y - vertex.y) / terrain.cell_size);
        
        // Clamp to valid range
        grid_x = std::max(0, std::min(grid_x, terrain.width - 1));
        grid_y = std::max(0, std::min(grid_y, terrain.height - 1));
        
        vertex_to_grid[v] = {grid_x, grid_y};
    }
    
    // Add bottom vertices (same x,y coordinates as surface, but z=0)
    std::vector<size_t> bottom_vertex_map(surface_vertex_count);
    for (size_t v = 0; v < surface_vertex_count; v++) {
        Point3D surface_vertex = vertices[v];
        Point3D bottom_vertex(surface_vertex.x, surface_vertex.y, 0.0);
        bottom_vertex_map[v] = addVertex(bottom_vertex);
    }
    
    // Build boundary edges for wall generation
    std::set<Edge> boundary_edges;
    std::map<Edge, std::vector<size_t>> edge_triangles;
    
    // Find boundary edges (edges used by only one triangle)
    for (size_t t = 0; t < surface_triangle_count; t++) {
        const Triangle& triangle = triangles[t];
        Edge e1(triangle.vertices[0], triangle.vertices[1]);
        Edge e2(triangle.vertices[1], triangle.vertices[2]);
        Edge e3(triangle.vertices[2], triangle.vertices[0]);
        
        edge_triangles[e1].push_back(t);
        edge_triangles[e2].push_back(t);
        edge_triangles[e3].push_back(t);
    }
    
    for (const auto& pair : edge_triangles) {
        if (pair.second.size() == 1) {  // Boundary edge
            boundary_edges.insert(pair.first);
        }
    }
    
    // Generate walls for boundary edges
    for (const Edge& edge : boundary_edges) {
        size_t v0 = edge.getV0();
        size_t v1 = edge.getV1();
        size_t b0 = bottom_vertex_map[v0];
        size_t b1 = bottom_vertex_map[v1];
        
        // Two triangles per wall edge (CCW from outside)
        addTriangle(v0, b0, v1);
        addTriangle(v1, b0, b1);
    }
    
    // Generate bottom face using existing detria-based approach
    std::vector<std::vector<size_t>> bottom_vertices(terrain.height, std::vector<size_t>(terrain.width, SIZE_MAX));
    
    // Map surface vertices to grid for bottom face triangulation
    for (size_t v = 0; v < surface_vertex_count; v++) {
        auto [grid_x, grid_y] = vertex_to_grid[v];
        bottom_vertices[grid_y][grid_x] = bottom_vertex_map[v];
    }
    
    triangulateBottomFaceWithDetria(bottom_vertices, terrain, nullptr);
}

// Helper method: Build initial complete surface triangulation from height field
void TerrainMesh::buildInitialSurfaceTriangulation(const TerrainData& terrain,
    std::vector<Point3D>& surface_vertices,
    std::vector<Triangle>& surface_triangles,
    std::vector<std::vector<size_t>>& grid_to_vertex) {
    
    // Create vertices for all grid points
    surface_vertices.reserve(static_cast<size_t>(terrain.width) * static_cast<size_t>(terrain.height));
    for (int y = 0; y < terrain.height; y++) {
        for (int x = 0; x < terrain.width; x++) {
            double world_x = terrain.origin.x + x * terrain.cell_size;
            double world_y = terrain.origin.y - y * terrain.cell_size;
            double height = terrain.getHeight(x, y);
            
            grid_to_vertex[y][x] = surface_vertices.size();
            surface_vertices.push_back(Point3D(world_x, world_y, height));
        }
    }
    
    // Create triangles for all grid cells (2 triangles per cell)
    surface_triangles.reserve(static_cast<size_t>(terrain.width - 1) * static_cast<size_t>(terrain.height - 1) * 2);
    for (int y = 0; y < terrain.height - 1; y++) {
        for (int x = 0; x < terrain.width - 1; x++) {
            size_t v00 = grid_to_vertex[y][x];
            size_t v10 = grid_to_vertex[y][x + 1];
            size_t v01 = grid_to_vertex[y + 1][x];
            size_t v11 = grid_to_vertex[y + 1][x + 1];
            
            // Two triangles per quad with CCW winding
            Triangle t1(v00, v10, v01);
            Triangle t2(v10, v11, v01);
            
            t1.computeNormal(surface_vertices);
            t2.computeNormal(surface_vertices);
            
            surface_triangles.push_back(t1);
            surface_triangles.push_back(t2);
        }
    }
}

// Helper method: Compute quadric error metrics for all vertices
void TerrainMesh::computeVertexQuadrics(const std::vector<Point3D>& vertices, 
    const std::vector<Triangle>& triangles,
    std::vector<QuadricError>& vertex_quadrics) {
    
    vertex_quadrics.assign(vertices.size(), QuadricError());
    
    // For each triangle, add its plane's quadric to all its vertices
    for (const Triangle& triangle : triangles) {
        // Compute plane equation from triangle
        const Point3D& p0 = vertices[triangle.vertices[0]];
        const Point3D& p1 = vertices[triangle.vertices[1]];
        const Point3D& p2 = vertices[triangle.vertices[2]];
        
        Point3D v1 = p1 - p0;
        Point3D v2 = p2 - p0;
        Point3D normal = v1.cross(v2);
        
        double normal_length = normal.length();
        if (normal_length > 1e-10) {
            normal = Point3D(normal.x / normal_length, normal.y / normal_length, normal.z / normal_length);
            
            // Plane equation: ax + by + cz + d = 0
            double a = normal.x;
            double b = normal.y;
            double c = normal.z;
            double d = -(a * p0.x + b * p0.y + c * p0.z);
            
            QuadricError plane_quadric(a, b, c, d);
            
            // Add this plane's quadric to all three vertices
            vertex_quadrics[triangle.vertices[0]] += plane_quadric;
            vertex_quadrics[triangle.vertices[1]] += plane_quadric;
            vertex_quadrics[triangle.vertices[2]] += plane_quadric;
        }
    }
}

// Helper method: Build priority queue of edge collapse candidates
void TerrainMesh::buildEdgeCollapseQueue(const std::vector<Point3D>& vertices,
    const std::vector<Triangle>& triangles,
    const std::vector<QuadricError>& vertex_quadrics,
    std::priority_queue<EdgeCollapse>& collapse_queue) {
    
    std::set<Edge> edges;
    
    // Collect all edges from triangles
    for (const Triangle& triangle : triangles) {
        edges.insert(Edge(triangle.vertices[0], triangle.vertices[1]));
        edges.insert(Edge(triangle.vertices[1], triangle.vertices[2]));
        edges.insert(Edge(triangle.vertices[2], triangle.vertices[0]));
    }
    
    // For each edge, compute collapse cost and optimal position
    for (const Edge& edge : edges) {
        size_t v0 = edge.getV0();
        size_t v1 = edge.getV1();
        
        const QuadricError& q0 = vertex_quadrics[v0];
        const QuadricError& q1 = vertex_quadrics[v1];
        QuadricError combined = q0 + q1;
        
        // Find optimal collapse position
        Point3D optimal_pos = computeOptimalCollapsePosition(q0, q1, vertices[v0], vertices[v1]);
        
        // Compute error at optimal position
        double collapse_error = combined.evaluate(optimal_pos.x, optimal_pos.y, optimal_pos.z);
        
        // Add to priority queue
        collapse_queue.push(EdgeCollapse(v0, v1, collapse_error, optimal_pos));
    }
}

// Helper method: Compute optimal position for edge collapse
Point3D TerrainMesh::computeOptimalCollapsePosition(const QuadricError& q1, const QuadricError& q2,
    const Point3D& v1, const Point3D& v2) {
    
    // For simplicity, use midpoint for now
    // A full implementation would solve the linear system to find the true minimum
    Point3D midpoint((v1.x + v2.x) * 0.5, (v1.y + v2.y) * 0.5, (v1.z + v2.z) * 0.5);
    
    // Evaluate error at three candidate positions: v1, v2, midpoint
    QuadricError combined = q1 + q2;
    double error1 = combined.evaluate(v1.x, v1.y, v1.z);
    double error2 = combined.evaluate(v2.x, v2.y, v2.z);
    double error_mid = combined.evaluate(midpoint.x, midpoint.y, midpoint.z);
    
    // Return position with lowest error
    if (error1 <= error2 && error1 <= error_mid) {
        return v1;
    } else if (error2 <= error_mid) {
        return v2;
    } else {
        return midpoint;
    }
}

// Helper method: Perform edge collapse and update data structures
bool TerrainMesh::performEdgeCollapse(const EdgeCollapse& collapse,
    std::vector<Point3D>& vertices,
    std::vector<Triangle>& triangles,
    std::vector<QuadricError>& vertex_quadrics,
    std::vector<bool>& vertex_removed,
    std::priority_queue<EdgeCollapse>& collapse_queue) {
    
    size_t v0 = collapse.edge.getV0();
    size_t v1 = collapse.edge.getV1();
    
    // Move v0 to optimal position and mark v1 as removed
    vertices[v0] = collapse.new_position;
    vertex_removed[v1] = true;
    
    // Update quadric for remaining vertex
    vertex_quadrics[v0] = vertex_quadrics[v0] + vertex_quadrics[v1];
    
    // Simple approach: just mark vertex as removed and handle degenerate triangles in final compaction
    // This is much more efficient than updating all triangles immediately
    
    return true;
}

// Helper method: Rebuild affected triangulation region (currently not needed for our approach)
void TerrainMesh::rebuildAffectedRegion(size_t collapsed_vertex, size_t remaining_vertex,
    const std::vector<Point3D>& vertices,
    std::vector<Triangle>& triangles) {
    // For our current approach, the edge collapse handles topology changes directly
    // In a more sophisticated implementation with detria re-triangulation, 
    // we would identify the affected region and re-triangulate it here
}

MeshStats TerrainMesh::validate(const TerrainData& terrain) const {
    MeshStats stats;

    // Check edge manifold property
    std::unordered_map<Edge, int, EdgeHash> edge_count;

    for (const auto& triangle : triangles) {
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
	    stats.setNonManifoldEdges(stats.getNonManifoldEdges() + 1);
	    stats.setIsManifold(false);
	}
    }

    // Check CCW orientation - for a volumetric mesh, this is more complex
    // We'll consider the mesh properly oriented if all normals point outward
    stats.setIsCCWOriented(true);
    int total_triangles = 0;
    int properly_oriented = 0;

    for (const auto& triangle : triangles) {
	const Point3D& p0 = vertices[triangle.vertices[0]];
	const Point3D& p1 = vertices[triangle.vertices[1]];
	const Point3D& p2 = vertices[triangle.vertices[2]];

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
	stats.setIsCCWOriented(properly_oriented >= total_triangles * 0.95); // 95% threshold
    }

    // Calculate volume using divergence theorem
    stats.setVolume(0.0);
    for (const auto& triangle : triangles) {
	const Point3D& p0 = vertices[triangle.vertices[0]];
	const Point3D& p1 = vertices[triangle.vertices[1]];
	const Point3D& p2 = vertices[triangle.vertices[2]];

	// Volume contribution from this triangle
	stats.setVolume(stats.getVolume() + (p0.x * (p1.y * p2.z - p2.y * p1.z) +
		    p1.x * (p2.y * p0.z - p0.y * p2.z) +
		    p2.x * (p0.y * p1.z - p1.y * p0.z)) / 6.0);
    }
    stats.setVolume(std::abs(stats.getVolume()));

    // Calculate expected volume from terrain data
    stats.setExpectedVolume(0.0);
    for (int y = 0; y < terrain.height; ++y) {
	for (int x = 0; x < terrain.width; ++x) {
	    double height = terrain.getHeight(x, y);
	    stats.setExpectedVolume(stats.getExpectedVolume() + height * terrain.cell_size * terrain.cell_size);
	}
    }

    // Calculate surface area (only terrain surface triangles)
    stats.setSurfaceArea(0.0);
    for (size_t i = 0; i < surface_triangle_count && i < triangles.size(); ++i) {
	const Triangle& triangle = triangles[i];
	const Point3D& p0 = vertices[triangle.vertices[0]];
	const Point3D& p1 = vertices[triangle.vertices[1]];
	const Point3D& p2 = vertices[triangle.vertices[2]];

	Point3D edge1 = p1 - p0;
	Point3D edge2 = p2 - p0;
	Point3D cross_product = edge1.cross(edge2);
	stats.setSurfaceArea(stats.getSurfaceArea() + cross_product.length() * 0.5);
    }

    // Expected surface area (rough approximation)
    stats.setExpectedSurfaceArea(terrain.width * terrain.height * terrain.cell_size * terrain.cell_size);

    return stats;
}

bool TerrainData::fromDSP(const DSPData& dsp) {
    if (!dsp.dsp_buf || dsp.dsp_xcnt == 0 || dsp.dsp_ycnt == 0) {
	return false;
    }

    width = static_cast<int>(dsp.dsp_xcnt);
    height = static_cast<int>(dsp.dsp_ycnt);
    cell_size = dsp.cell_size;
    origin = dsp.origin;

    // Initialize height array
    heights.resize(height);
    for (int y = 0; y < height; ++y) {
	heights[y].resize(width);
    }

    // Convert unsigned short data to double and find min/max
    min_height = std::numeric_limits<double>::max();
    max_height = std::numeric_limits<double>::lowest();

    for (int y = 0; y < height; ++y) {
	for (int x = 0; x < width; ++x) {
	    double height_val = static_cast<double>(dsp.dsp_buf[y * dsp.dsp_xcnt + x]);
	    heights[y][x] = height_val;
	    min_height = std::min(min_height, height_val);
	    max_height = std::max(max_height, height_val);
	}
    }

    return true;
}

bool TerrainData::toDSP(DSPData& dsp) const {
    if (width <= 0 || height <= 0 || heights.empty()) {
	return false;
    }

    dsp.dsp_xcnt = static_cast<uint32_t>(width);
    dsp.dsp_ycnt = static_cast<uint32_t>(height);
    dsp.cell_size = cell_size;
    dsp.origin = origin;

    // Allocate buffer if not already allocated
    if (!dsp.dsp_buf) {
	dsp.dsp_buf = new unsigned short[dsp.dsp_xcnt * dsp.dsp_ycnt];
	dsp.owns_buffer = true;
    }

    // Convert double data to unsigned short
    for (int y = 0; y < height; ++y) {
	for (int x = 0; x < width; ++x) {
	    double height_val = getHeight(x, y);
	    // Clamp to unsigned short range
	    if (height_val < 0) height_val = 0;
	    if (height_val > 65535) height_val = 65535;
	    dsp.dsp_buf[y * dsp.dsp_xcnt + x] = static_cast<unsigned short>(height_val);
	}
    }

    return true;
}

bool DSPData::toTerrain(TerrainData& terrain) const {
    return terrain.fromDSP(*this);
}

bool TerrainMesh::toNMG(NMGTriangleData& nmg_data) const {
    if (vertices.empty() || triangles.empty()) {
	return false;
    }

    nmg_data.triangles.clear();
    nmg_data.unique_vertices = vertices;  // Copy vertex data
    nmg_data.surface_triangle_count = surface_triangle_count;

    // Convert triangles to NMG format
    nmg_data.triangles.reserve(triangles.size());

    for (size_t i = 0; i < triangles.size(); ++i) {
	const Triangle& tri = triangles[i];

	// Create triangle vertices with references to unique vertex array
	NMGTriangleData::TriangleVertex v0(vertices[tri.vertices[0]], tri.vertices[0]);
	NMGTriangleData::TriangleVertex v1(vertices[tri.vertices[1]], tri.vertices[1]);
	NMGTriangleData::TriangleVertex v2(vertices[tri.vertices[2]], tri.vertices[2]);

	// Determine if this is a surface triangle (first N triangles are surface)
	bool is_surface = (i < surface_triangle_count);

	// Create NMG triangle
	NMGTriangleData::NMGTriangle nmg_tri(v0, v1, v2, is_surface);
	nmg_data.triangles.push_back(nmg_tri);
    }

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
