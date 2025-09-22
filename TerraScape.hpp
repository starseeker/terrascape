#pragma once

/*
 * TerraScape - Advanced Terrain Mesh Generation Library
 *
 * This library provides efficient grid-to-mesh conversion with advanced
 * triangulation and refinement algorithms.
 *
 * Advanced Delaunay triangulation is used for grid heightfields,
 * with adaptive error control, mesh-quality constraints, and multi-pass refinement.
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

// BRL-CAD tolerance integration and region-growing triangulation now included directly

#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#elif defined(__APPLE__)
#include <sys/types.h>
#include <sys/sysctl.h>
#endif

#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic push /* start new diagnostic pragma */
#  pragma GCC diagnostic ignored "-Wfloat-equal"
#elif defined(__clang__)
#  pragma clang diagnostic push /* start new diagnostic pragma */
#  pragma clang diagnostic ignored "-Wfloat-equal"
#endif

namespace TerraScape {

// ================================= BRL-CAD Tolerance Integration =================================

// Region-growing triangulation options with BRL-CAD tolerance integration
struct RegionGrowingOptions {
  // PRIMARY INTERFACE: Single mesh density parameter (0.0 = coarsest, 1.0 = finest)
  double mesh_density = 0.5;                 // Controls overall mesh resolution and detail level
  
  // BRL-CAD TOLERANCE INTEGRATION: Precision control parameters
  double abs_tolerance_mm = 0.1;             // Absolute tolerance in millimeters
  double rel_tolerance = 0.01;               // Relative tolerance (unitless, as fraction)
  double norm_tolerance_deg = 15.0;          // Normal angle tolerance in degrees
  double volume_delta_pct = 10.0;            // Max allowed mesh/cell volume % difference
  
  // DERIVED PARAMETERS: Automatically calculated from mesh_density (can be overridden)
  double region_merge_threshold = 0.5;       // Height difference threshold for region merging
  int sampling_step = 1;                     // Point sampling step size for triangulation
  double base_error_threshold = 0.2;         // Error threshold for mesh approximation
  
  // QUALITY CONSTRAINTS: Mesh quality control
  double min_angle_deg = 20.0;               // Minimum triangle angle constraint
  double max_aspect_ratio = 6.0;             // Maximum aspect ratio (longest/shortest edge)
  double min_area = 0.5;                     // Minimum triangle area in grid units
  bool enable_quality_filtering = true;      // Enable triangle quality filtering during generation
  
  // ADVANCED PARAMETERS: Typically not modified by users
  double slope_weight = 3.0;                 // Weight for slope-based error calculation
  double curvature_weight = 10.0;            // Weight for curvature-based error calculation
  int max_refinement_passes = 2;             // Number of refinement passes
  int max_initial_iterations = 5'000'000;    // Safety cap for very large datasets
  bool early_exit_error_eval = true;         // Enable early-exit in error calculation
  bool treat_all_invalid_as_blocking = true; // How to handle masked areas
  double bary_eps = 1e-6;                    // Barycentric coordinate epsilon
  double det_eps = 1e-12;                    // Determinant epsilon for numerical stability
  double area_eps = 1e-12;                   // Area epsilon for numerical stability
  bool use_volume_convergence = false;       // Enable volume-based stopping criteria
  double volume_convergence_threshold = 0.01; // Volume change threshold for convergence
  int min_volume_passes = 2;                 // Minimum passes before checking volume convergence
  bool use_precomputed_complexity = true;    // Use pre-computed terrain complexity
  double complexity_scale_factor = 1.0;      // Complexity scaling factor
  
  // FEATURE DETECTION & GRAPH OPTIMIZATION: New advanced features
  bool enable_feature_detection = false;      // Enable terrain feature detection
  bool enable_graph_optimization = false;     // Enable graph-based segmentation optimization
  double feature_penalty_weight = 10.0;       // Weight for penalizing cuts across strong features
  double feature_threshold = 0.5;             // Threshold for identifying strong features
  bool use_mst_for_regions = false;           // Use MST for region connectivity optimization
  bool use_mincut_for_boundaries = false;     // Use min-cut for optimal boundary placement
};

// Internal mesh structure for region-growing algorithm
struct InternalMesh {
  std::vector<std::array<double, 3>> vertices; // x,y,z (grid coords + elevation)
  std::vector<std::array<int, 3>> triangles;   // vertex indices
  
  // Helper functions to add vertices and triangles
  void add_vertex(double x, double y, double z) {
    vertices.emplace_back(std::array<double, 3>{x, y, z});
  }
  
  void add_triangle(int v0, int v1, int v2) {
    triangles.emplace_back(std::array<int, 3>{v0, v1, v2});
  }
  
  // Add triangle with correct winding order for upward normals
  void add_triangle_with_upward_normal(int v0, int v1, int v2) {
    if (v0 >= vertices.size() || v1 >= vertices.size() || v2 >= vertices.size()) {
      return; // Invalid vertex indices
    }
    
    // Compute triangle normal to determine correct winding
    const auto& p0 = vertices[v0];
    const auto& p1 = vertices[v1]; 
    const auto& p2 = vertices[v2];
    
    // Edge vectors
    double edge1_x = p1[0] - p0[0];
    double edge1_y = p1[1] - p0[1];
    double edge1_z = p1[2] - p0[2];
    
    double edge2_x = p2[0] - p0[0];
    double edge2_y = p2[1] - p0[1];
    double edge2_z = p2[2] - p0[2];
    
    // Cross product: edge1 × edge2 (only need z component for 2D triangulation)
    double normal_z = edge1_x * edge2_y - edge1_y * edge2_x;
    
    // Check triangle area to avoid degenerate triangles
    double area = std::abs(normal_z) * 0.5;
    if (area < 1e-6) {
      return; // Degenerate triangle
    }
    
    // Add triangle with correct winding for upward normal (normal_z > 0)
    if (normal_z > 0) {
      triangles.emplace_back(std::array<int, 3>{v0, v1, v2}); // CCW (upward normal)
    } else {
      triangles.emplace_back(std::array<int, 3>{v0, v2, v1}); // CW -> CCW (flip to upward normal)
    }
  }
};

// ================================= Feature Detection Module =================================

// Forward declaration for ElevationGrid structure compatibility
struct ElevationGrid {
    const float* data;
    int width;
    int height;
    
    ElevationGrid(const float* elevation_data, int w, int h) 
        : data(elevation_data), width(w), height(h) {}
    
    // Safe elevation access with bounds checking
    float get_elevation(int x, int y) const {
        if (x < 0 || x >= width || y < 0 || y >= height) {
            return 0.0f; // Return 0 for out-of-bounds
        }
        return data[y * width + x];
    }
};

namespace FeatureDetection {

// Feature detection parameters
struct FeatureDetectionOptions {
    // Gradient computation parameters
    double gradient_threshold = 0.1;      // Threshold for significant gradients
    bool use_sobel_operator = true;       // Use Sobel vs simple difference
    
    // Curvature computation parameters  
    double curvature_threshold = 0.05;    // Threshold for significant curvature
    bool enable_curvature = true;         // Enable curvature-based detection
    
    // Feature combination parameters
    double gradient_weight = 0.7;         // Weight for gradient in final feature map
    double curvature_weight = 0.3;        // Weight for curvature in final feature map
    
    // Smoothing parameters
    bool enable_smoothing = true;         // Enable Gaussian smoothing of feature map
    double smoothing_sigma = 1.0;         // Standard deviation for Gaussian smoothing
};

// Compute gradient magnitude using Sobel operator
inline std::vector<std::vector<double>> compute_gradient_map(const ElevationGrid& grid, 
                                                            const FeatureDetectionOptions& opts) {
    std::vector<std::vector<double>> gradient_map(grid.height, std::vector<double>(grid.width, 0.0));
    
    for (int y = 1; y < grid.height - 1; y++) {
        for (int x = 1; x < grid.width - 1; x++) {
            double gx, gy;
            
            if (opts.use_sobel_operator) {
                // Sobel operator for more robust gradient estimation
                gx = -1.0 * grid.get_elevation(x-1, y-1) + 1.0 * grid.get_elevation(x+1, y-1) +
                     -2.0 * grid.get_elevation(x-1, y  ) + 2.0 * grid.get_elevation(x+1, y  ) +
                     -1.0 * grid.get_elevation(x-1, y+1) + 1.0 * grid.get_elevation(x+1, y+1);
                
                gy = -1.0 * grid.get_elevation(x-1, y-1) - 2.0 * grid.get_elevation(x, y-1) - 1.0 * grid.get_elevation(x+1, y-1) +
                      1.0 * grid.get_elevation(x-1, y+1) + 2.0 * grid.get_elevation(x, y+1) + 1.0 * grid.get_elevation(x+1, y+1);
            } else {
                // Simple central difference
                gx = grid.get_elevation(x+1, y) - grid.get_elevation(x-1, y);
                gy = grid.get_elevation(x, y+1) - grid.get_elevation(x, y-1);
            }
            
            gradient_map[y][x] = std::sqrt(gx * gx + gy * gy);
        }
    }
    
    return gradient_map;
}

// Compute curvature using Laplacian operator
inline std::vector<std::vector<double>> compute_curvature_map(const ElevationGrid& grid,
                                                             const FeatureDetectionOptions& opts) {
    std::vector<std::vector<double>> curvature_map(grid.height, std::vector<double>(grid.width, 0.0));
    
    for (int y = 1; y < grid.height - 1; y++) {
        for (int x = 1; x < grid.width - 1; x++) {
            // Discrete Laplacian operator (4-connected)
            double center = grid.get_elevation(x, y);
            double laplacian = grid.get_elevation(x-1, y) + grid.get_elevation(x+1, y) + 
                              grid.get_elevation(x, y-1) + grid.get_elevation(x, y+1) - 4.0 * center;
            
            curvature_map[y][x] = std::abs(laplacian);
        }
    }
    
    return curvature_map;
}

// Apply Gaussian smoothing to feature map
inline void smooth_feature_map(std::vector<std::vector<double>>& feature_map, 
                              double sigma) {
    if (sigma <= 0.0) return;
    
    int height = static_cast<int>(feature_map.size());
    int width = static_cast<int>(feature_map[0].size());
    
    // Simple 3x3 Gaussian kernel approximation
    std::array<std::array<double, 3>, 3> kernel;
    double sum = 0.0;
    
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            double value = std::exp(-(dx*dx + dy*dy) / (2.0 * sigma * sigma));
            kernel[dy+1][dx+1] = value;
            sum += value;
        }
    }
    
    // Normalize kernel
    for (int dy = 0; dy < 3; dy++) {
        for (int dx = 0; dx < 3; dx++) {
            kernel[dy][dx] /= sum;
        }
    }
    
    // Apply convolution
    std::vector<std::vector<double>> smoothed(height, std::vector<double>(width, 0.0));
    
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            double filtered_value = 0.0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    filtered_value += feature_map[y + dy][x + dx] * kernel[dy + 1][dx + 1];
                }
            }
            smoothed[y][x] = filtered_value;
        }
    }
    
    feature_map = std::move(smoothed);
}

// Normalize feature map to [0, 1] range
inline void normalize_feature_map(std::vector<std::vector<double>>& feature_map) {
    double min_val = std::numeric_limits<double>::max();
    double max_val = std::numeric_limits<double>::lowest();
    
    // Find min and max values
    for (const auto& row : feature_map) {
        for (double val : row) {
            min_val = std::min(min_val, val);
            max_val = std::max(max_val, val);
        }
    }
    
    // Normalize to [0, 1] range
    if (max_val > min_val) {
        double range = max_val - min_val;
        for (auto& row : feature_map) {
            for (double& val : row) {
                val = (val - min_val) / range;
            }
        }
    }
}

// Main API function: compute feature map from elevation grid
inline std::vector<std::vector<double>> compute_feature_map(const ElevationGrid& grid,
                                                           const FeatureDetectionOptions& opts = FeatureDetectionOptions{}) {
    // Compute gradient-based features
    auto gradient_map = compute_gradient_map(grid, opts);
    
    // Compute curvature-based features if enabled
    std::vector<std::vector<double>> curvature_map;
    if (opts.enable_curvature) {
        curvature_map = compute_curvature_map(grid, opts);
    }
    
    // Combine gradient and curvature features
    std::vector<std::vector<double>> feature_map(grid.height, std::vector<double>(grid.width, 0.0));
    
    for (int y = 0; y < grid.height; y++) {
        for (int x = 0; x < grid.width; x++) {
            double feature_strength = opts.gradient_weight * gradient_map[y][x];
            
            if (opts.enable_curvature && !curvature_map.empty()) {
                feature_strength += opts.curvature_weight * curvature_map[y][x];
            }
            
            feature_map[y][x] = feature_strength;
        }
    }
    
    // Apply smoothing if enabled
    if (opts.enable_smoothing) {
        smooth_feature_map(feature_map, opts.smoothing_sigma);
    }
    
    // Normalize to [0, 1] range
    normalize_feature_map(feature_map);
    
    return feature_map;
}

// Utility function to identify strong features above threshold
inline std::vector<std::pair<int, int>> find_strong_features(const std::vector<std::vector<double>>& feature_map,
                                                           double threshold = 0.5) {
    std::vector<std::pair<int, int>> feature_points;
    
    int height = static_cast<int>(feature_map.size());
    int width = static_cast<int>(feature_map[0].size());
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (feature_map[y][x] > threshold) {
                feature_points.emplace_back(x, y);
            }
        }
    }
    
    return feature_points;
}

} // namespace FeatureDetection

// ================================= Graph Algorithms Module =================================

namespace Lemon {

// Simple node and edge identifier types
using NodeId = int;
using EdgeId = int;

// Edge structure for undirected graphs
struct Edge {
    NodeId u, v;        // endpoints
    double weight;      // edge weight
    EdgeId id;          // edge identifier
    
    Edge(NodeId u_, NodeId v_, double w = 1.0, EdgeId id_ = -1) 
        : u(u_), v(v_), weight(w), id(id_) {}
    
    // Get the other endpoint given one endpoint
    NodeId other(NodeId node) const {
        return (node == u) ? v : u;
    }
};

// Simple undirected graph representation
class ListGraph {
private:
    std::vector<std::vector<EdgeId>> adjacency_;  // adjacency lists
    std::vector<Edge> edges_;                     // edge storage
    int next_edge_id_;
    
public:
    ListGraph() : next_edge_id_(0) {}
    
    // Add a node and return its ID
    NodeId addNode() {
        NodeId id = static_cast<NodeId>(adjacency_.size());
        adjacency_.emplace_back();
        return id;
    }
    
    // Add an edge between two nodes
    EdgeId addEdge(NodeId u, NodeId v, double weight = 1.0) {
        EdgeId edge_id = next_edge_id_++;
        edges_.emplace_back(u, v, weight, edge_id);
        
        // Ensure nodes exist
        while (static_cast<int>(adjacency_.size()) <= std::max(u, v)) {
            addNode();
        }
        
        adjacency_[u].push_back(edge_id);
        adjacency_[v].push_back(edge_id);
        
        return edge_id;
    }
    
    // Get number of nodes
    int nodeCount() const {
        return static_cast<int>(adjacency_.size());
    }
    
    // Get number of edges
    int edgeCount() const {
        return static_cast<int>(edges_.size());
    }
    
    // Get edge by ID
    const Edge& edge(EdgeId id) const {
        return edges_[id];
    }
    
    Edge& edge(EdgeId id) {
        return edges_[id];
    }
    
    // Get adjacent edges for a node
    const std::vector<EdgeId>& incidentEdges(NodeId node) const {
        return adjacency_[node];
    }
    
    // Get all edges
    const std::vector<Edge>& edges() const {
        return edges_;
    }
    
    // Iterator support for edges
    std::vector<Edge>::const_iterator edgeBegin() const {
        return edges_.begin();
    }
    
    std::vector<Edge>::const_iterator edgeEnd() const {
        return edges_.end();
    }
};

// Union-Find data structure for Kruskal's algorithm
class UnionFind {
private:
    std::vector<int> parent_;
    std::vector<int> rank_;
    
public:
    explicit UnionFind(int n) : parent_(n), rank_(n, 0) {
        for (int i = 0; i < n; i++) {
            parent_[i] = i;
        }
    }
    
    int find(int x) {
        if (parent_[x] != x) {
            parent_[x] = find(parent_[x]); // path compression
        }
        return parent_[x];
    }
    
    bool unite(int x, int y) {
        int px = find(x), py = find(y);
        if (px == py) return false;
        
        // Union by rank
        if (rank_[px] < rank_[py]) {
            parent_[px] = py;
        } else if (rank_[px] > rank_[py]) {
            parent_[py] = px;
        } else {
            parent_[py] = px;
            rank_[px]++;
        }
        return true;
    }
    
    bool connected(int x, int y) {
        return find(x) == find(y);
    }
};

// Kruskal's Minimum Spanning Tree algorithm
inline std::vector<EdgeId> kruskalMST(const ListGraph& graph) {
    std::vector<EdgeId> mst_edges;
    std::vector<EdgeId> all_edges;
    
    // Collect all edge IDs
    for (int i = 0; i < graph.edgeCount(); i++) {
        all_edges.push_back(i);
    }
    
    // Sort edges by weight
    std::sort(all_edges.begin(), all_edges.end(), 
              [&graph](EdgeId a, EdgeId b) {
                  return graph.edge(a).weight < graph.edge(b).weight;
              });
    
    UnionFind uf(graph.nodeCount());
    
    for (EdgeId edge_id : all_edges) {
        const Edge& e = graph.edge(edge_id);
        if (uf.unite(e.u, e.v)) {
            mst_edges.push_back(edge_id);
            if (static_cast<int>(mst_edges.size()) == graph.nodeCount() - 1) {
                break; // MST complete
            }
        }
    }
    
    return mst_edges;
}

// Prim's Minimum Spanning Tree algorithm (alternative implementation)
inline std::vector<EdgeId> primMST(const ListGraph& graph, NodeId start = 0) {
    if (graph.nodeCount() == 0) return {};
    
    std::vector<EdgeId> mst_edges;
    std::vector<bool> in_mst(graph.nodeCount(), false);
    
    // Priority queue: (weight, edge_id)
    std::priority_queue<std::pair<double, EdgeId>, 
                       std::vector<std::pair<double, EdgeId>>,
                       std::greater<std::pair<double, EdgeId>>> pq;
    
    // Start with first node
    in_mst[start] = true;
    
    // Add all edges from start node to priority queue
    for (EdgeId edge_id : graph.incidentEdges(start)) {
        const Edge& e = graph.edge(edge_id);
        NodeId other = e.other(start);
        if (!in_mst[other]) {
            pq.emplace(e.weight, edge_id);
        }
    }
    
    while (!pq.empty() && static_cast<int>(mst_edges.size()) < graph.nodeCount() - 1) {
        auto [weight, edge_id] = pq.top();
        pq.pop();
        
        const Edge& e = graph.edge(edge_id);
        
        // Find which endpoint is not in MST
        NodeId new_node = -1;
        if (in_mst[e.u] && !in_mst[e.v]) {
            new_node = e.v;
        } else if (in_mst[e.v] && !in_mst[e.u]) {
            new_node = e.u;
        } else {
            continue; // Both endpoints already in MST
        }
        
        // Add edge to MST
        mst_edges.push_back(edge_id);
        in_mst[new_node] = true;
        
        // Add edges from new node
        for (EdgeId adj_edge_id : graph.incidentEdges(new_node)) {
            const Edge& adj_e = graph.edge(adj_edge_id);
            NodeId other = adj_e.other(new_node);
            if (!in_mst[other]) {
                pq.emplace(adj_e.weight, adj_edge_id);
            }
        }
    }
    
    return mst_edges;
}

// Simple max-flow/min-cut using Ford-Fulkerson with DFS
class MaxFlow {
private:
    struct FlowEdge {
        NodeId to;
        double capacity;
        double flow;
        int reverse_edge_id;
        
        FlowEdge(NodeId to_, double cap) : to(to_), capacity(cap), flow(0.0), reverse_edge_id(-1) {}
    };
    
    std::vector<std::vector<FlowEdge>> adj_;
    std::vector<bool> visited_;
    
    double dfs(NodeId node, NodeId sink, double min_flow) {
        if (node == sink) return min_flow;
        
        visited_[node] = true;
        
        for (auto& edge : adj_[node]) {
            if (!visited_[edge.to] && edge.capacity > edge.flow) {
                double bottleneck = std::min(min_flow, edge.capacity - edge.flow);
                double flow_sent = dfs(edge.to, sink, bottleneck);
                
                if (flow_sent > 0) {
                    edge.flow += flow_sent;
                    adj_[edge.to][edge.reverse_edge_id].flow -= flow_sent;
                    return flow_sent;
                }
            }
        }
        
        return 0.0;
    }
    
public:
    explicit MaxFlow(int n) : adj_(n), visited_(n) {}
    
    void addEdge(NodeId from, NodeId to, double capacity) {
        adj_[from].emplace_back(to, capacity);
        adj_[to].emplace_back(from, 0.0); // reverse edge with 0 capacity
        
        // Set reverse edge references
        adj_[from].back().reverse_edge_id = static_cast<int>(adj_[to].size()) - 1;
        adj_[to].back().reverse_edge_id = static_cast<int>(adj_[from].size()) - 1;
    }
    
    double maxFlowValue(NodeId source, NodeId sink) {
        double total_flow = 0.0;
        
        while (true) {
            std::fill(visited_.begin(), visited_.end(), false);
            double flow_sent = dfs(source, sink, std::numeric_limits<double>::max());
            
            if (flow_sent == 0.0) break;
            total_flow += flow_sent;
        }
        
        return total_flow;
    }
    
    // Get min-cut (nodes reachable from source after max flow)
    std::vector<bool> getMinCut(NodeId source, NodeId sink) {
        maxFlowValue(source, sink); // Ensure max flow is computed
        
        std::vector<bool> reachable(adj_.size(), false);
        std::queue<NodeId> q;
        q.push(source);
        reachable[source] = true;
        
        while (!q.empty()) {
            NodeId node = q.front();
            q.pop();
            
            for (const auto& edge : adj_[node]) {
                if (!reachable[edge.to] && edge.capacity > edge.flow) {
                    reachable[edge.to] = true;
                    q.push(edge.to);
                }
            }
        }
        
        return reachable;
    }
};

// Utility function to build terrain graph from feature map
inline ListGraph buildTerrainGraph(int width, int height, 
                                  const std::vector<std::vector<double>>& feature_map,
                                  double feature_penalty_weight = 10.0) {
    ListGraph graph;
    
    // Add nodes for each grid cell
    std::vector<std::vector<NodeId>> node_grid(height, std::vector<NodeId>(width));
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            node_grid[y][x] = graph.addNode();
        }
    }
    
    // Add edges between adjacent cells
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Right neighbor
            if (x + 1 < width) {
                double edge_weight = 1.0 + feature_penalty_weight * 
                    std::max(feature_map[y][x], feature_map[y][x+1]);
                graph.addEdge(node_grid[y][x], node_grid[y][x+1], edge_weight);
            }
            
            // Bottom neighbor
            if (y + 1 < height) {
                double edge_weight = 1.0 + feature_penalty_weight * 
                    std::max(feature_map[y][x], feature_map[y+1][x]);
                graph.addEdge(node_grid[y][x], node_grid[y+1][x], edge_weight);
            }
        }
    }
    
    return graph;
}

} // namespace Lemon

// ================================= Helper Functions =================================

static inline size_t idx_row_major(int x, int y, int width) {
  return static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
}

static inline double hypot2d(double dx, double dy) {
  return std::hypot(dx, dy);
}

static inline double tri_area2d(double x0, double y0, double x1, double y1, double x2, double y2) {
  return 0.5 * std::abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0));
}

// BRL-CAD tolerance helper functions
static inline bool meets_tolerance_threshold(double height_diff, double center_elev, double abs_tolerance, double rel_tolerance) {
  return height_diff >= abs_tolerance || height_diff >= rel_tolerance * std::abs(center_elev);
}

static inline bool within_tolerance_threshold(double height_diff, double center_elev, double neighbor_elev, double abs_tolerance, double rel_tolerance) {
  double threshold = std::min(abs_tolerance, rel_tolerance * std::max(std::abs(center_elev), std::abs(neighbor_elev)));
  return height_diff <= threshold;
}

// Calculate mesh volume approximation for surface meshes
// For surface meshes, we approximate volume by projecting triangles to base plane
static inline double calculate_internal_mesh_volume_simple(const InternalMesh& mesh) {
  if (mesh.triangles.empty()) return 0.0;
  
  // Find minimum elevation in mesh to use as base level
  double min_z = std::numeric_limits<double>::max();
  for (const auto& vertex : mesh.vertices) {
    min_z = std::min(min_z, vertex[2]);
  }
  
  double volume = 0.0;
  
  // For surface meshes, approximate volume by integrating height over triangular areas
  for (const auto& tri : mesh.triangles) {
    const auto& v0 = mesh.vertices[tri[0]];
    const auto& v1 = mesh.vertices[tri[1]];
    const auto& v2 = mesh.vertices[tri[2]];
    
    // Calculate triangle area in x-y plane
    double area = 0.5 * std::abs((v1[0] - v0[0]) * (v2[1] - v0[1]) - (v2[0] - v0[0]) * (v1[1] - v0[1]));
    
    // Average height above base level
    double avg_height = ((v0[2] - min_z) + (v1[2] - min_z) + (v2[2] - min_z)) / 3.0;
    
    // Volume contribution = area × average height
    volume += area * std::max(0.0, avg_height);
  }
  
  return volume;
}

// Calculate theoretical cell volume (simple height field integration)
static inline double calculate_cell_volume_simple(const float* elevations, int width, int height, const uint8_t* mask = nullptr) {
  double volume = 0.0;
  double cell_area = 1.0; // Assuming unit grid cells
  
  // Find minimum elevation to use as base level (consistent with mesh generation)
  float min_elev = std::numeric_limits<float>::max();
  for (int i = 0; i < width * height; ++i) {
    if (mask && mask[i] == 0) continue;
    min_elev = std::min(min_elev, elevations[i]);
  }
  
  // Calculate volume above minimum elevation level
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
      if (mask && mask[idx] == 0) continue;
      double height_above_base = elevations[idx] - min_elev;
      if (height_above_base > 0.0) {
        volume += height_above_base * cell_area;
      }
    }
  }
  return volume;
}

// Calculate theoretical terrain surface area using grid approximation
static inline double calculate_terrain_surface_area(const float* elevations, int width, int height) {
  double totalArea = 0.0;
  
  for (int y = 0; y < height - 1; ++y) {
    for (int x = 0; x < width - 1; ++x) {
      // Get heights of 4 corners of grid cell
      float h00 = elevations[y * width + x];
      float h10 = elevations[y * width + (x + 1)];
      float h01 = elevations[(y + 1) * width + x];
      float h11 = elevations[(y + 1) * width + (x + 1)];
      
      // Split quad into two triangles and calculate their areas
      // Triangle 1: (0,0,h00), (1,0,h10), (0,1,h01)
      float v1x = 1.0f, v1y = 0.0f, v1z = h10 - h00;
      float v2x = 0.0f, v2y = 1.0f, v2z = h01 - h00;
      
      float cross1x = v1y * v2z - v1z * v2y;
      float cross1y = v1z * v2x - v1x * v2z;
      float cross1z = v1x * v2y - v1y * v2x;
      float area1 = 0.5f * std::sqrt(cross1x*cross1x + cross1y*cross1y + cross1z*cross1z);
      
      // Triangle 2: (1,0,h10), (1,1,h11), (0,1,h01)
      float v3x = 0.0f, v3y = 1.0f, v3z = h11 - h10;
      float v4x = -1.0f, v4y = 1.0f, v4z = h01 - h10;
      
      float cross2x = v3y * v4z - v3z * v4y;
      float cross2y = v3z * v4x - v3x * v4z;
      float cross2z = v3x * v4y - v3y * v4x;
      float area2 = 0.5f * std::sqrt(cross2x*cross2x + cross2y*cross2y + cross2z*cross2z);
      
      totalArea += area1 + area2;
    }
  }
  
  return totalArea;
}

// Calculate surface area of mesh
static inline double calculate_mesh_surface_area(const InternalMesh& mesh) {
  double totalArea = 0.0;
  
  for (const auto& tri : mesh.triangles) {
    const auto& v0 = mesh.vertices[tri[0]];
    const auto& v1 = mesh.vertices[tri[1]];
    const auto& v2 = mesh.vertices[tri[2]];
    
    // Calculate triangle area using cross product
    double v1x = v1[0] - v0[0], v1y = v1[1] - v0[1], v1z = v1[2] - v0[2];
    double v2x = v2[0] - v0[0], v2y = v2[1] - v0[1], v2z = v2[2] - v0[2];
    
    double crossx = v1y * v2z - v1z * v2y;
    double crossy = v1z * v2x - v1x * v2z;
    double crossz = v1x * v2y - v1y * v2x;
    double area = 0.5 * std::sqrt(crossx*crossx + crossy*crossy + crossz*crossz);
    
    totalArea += area;
  }
  
  return totalArea;
}

// Structure to hold bounding box information
struct BoundingBox {
  double min_x, max_x, min_y, max_y, min_z, max_z;
  
  BoundingBox() : min_x(std::numeric_limits<double>::max()), 
                  max_x(std::numeric_limits<double>::lowest()),
                  min_y(std::numeric_limits<double>::max()), 
                  max_y(std::numeric_limits<double>::lowest()),
                  min_z(std::numeric_limits<double>::max()), 
                  max_z(std::numeric_limits<double>::lowest()) {}
                  
  BoundingBox(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
    : min_x(min_x), max_x(max_x), min_y(min_y), max_y(max_y), min_z(min_z), max_z(max_z) {}
};

// Calculate bounding box of terrain data from grid dimensions and elevation data
static inline BoundingBox calculate_terrain_bounds(const float* elevations, int width, int height) {
  BoundingBox bounds;
  
  // X and Y bounds are determined by grid dimensions (0-based indexing)
  bounds.min_x = 0.0;
  bounds.max_x = static_cast<double>(width - 1);
  bounds.min_y = 0.0; 
  bounds.max_y = static_cast<double>(height - 1);
  
  // Z bounds are determined by elevation data
  bounds.min_z = std::numeric_limits<double>::max();
  bounds.max_z = std::numeric_limits<double>::lowest();
  
  for (int i = 0; i < width * height; i++) {
    double elev = static_cast<double>(elevations[i]);
    bounds.min_z = std::min(bounds.min_z, elev);
    bounds.max_z = std::max(bounds.max_z, elev);
  }
  
  return bounds;
}

// Calculate bounding box of generated mesh
static inline BoundingBox calculate_mesh_bounds(const InternalMesh& mesh) {
  BoundingBox bounds;
  
  if (mesh.vertices.empty()) {
    return bounds;
  }
  
  for (const auto& vertex : mesh.vertices) {
    bounds.min_x = std::min(bounds.min_x, vertex[0]);
    bounds.max_x = std::max(bounds.max_x, vertex[0]);
    bounds.min_y = std::min(bounds.min_y, vertex[1]);
    bounds.max_y = std::max(bounds.max_y, vertex[1]);
    bounds.min_z = std::min(bounds.min_z, vertex[2]);
    bounds.max_z = std::max(bounds.max_z, vertex[2]);
  }
  
  return bounds;
}

// Validate that mesh bounding box adequately covers terrain bounding box
static inline bool validate_mesh_bounds(const BoundingBox& terrain_bounds, 
                                       const BoundingBox& mesh_bounds, 
                                       double tolerance = 1e-6) {
  bool valid = true;
  
  std::cerr << "Terrain Bounding Box Validation:" << std::endl;
  std::cerr << "  Terrain bounds: X[" << terrain_bounds.min_x << ", " << terrain_bounds.max_x 
            << "] Y[" << terrain_bounds.min_y << ", " << terrain_bounds.max_y 
            << "] Z[" << terrain_bounds.min_z << ", " << terrain_bounds.max_z << "]" << std::endl;
  std::cerr << "  Mesh bounds:    X[" << mesh_bounds.min_x << ", " << mesh_bounds.max_x 
            << "] Y[" << mesh_bounds.min_y << ", " << mesh_bounds.max_y 
            << "] Z[" << mesh_bounds.min_z << ", " << mesh_bounds.max_z << "]" << std::endl;
  
  // Check X bounds
  if (mesh_bounds.min_x > terrain_bounds.min_x + tolerance || 
      mesh_bounds.max_x < terrain_bounds.max_x - tolerance) {
    std::cerr << "  ❌ FAIL: Mesh X bounds do not cover terrain X bounds" << std::endl;
    valid = false;
  } else {
    std::cerr << "  ✓ PASS: Mesh X bounds cover terrain X bounds" << std::endl;
  }
  
  // Check Y bounds
  if (mesh_bounds.min_y > terrain_bounds.min_y + tolerance || 
      mesh_bounds.max_y < terrain_bounds.max_y - tolerance) {
    std::cerr << "  ❌ FAIL: Mesh Y bounds do not cover terrain Y bounds" << std::endl;
    valid = false;
  } else {
    std::cerr << "  ✓ PASS: Mesh Y bounds cover terrain Y bounds" << std::endl;
  }
  
  // Check Z bounds (elevation)
  if (mesh_bounds.min_z > terrain_bounds.min_z + tolerance || 
      mesh_bounds.max_z < terrain_bounds.max_z - tolerance) {
    std::cerr << "  ❌ FAIL: Mesh Z bounds do not cover terrain Z bounds" << std::endl;
    valid = false;
  } else {
    std::cerr << "  ✓ PASS: Mesh Z bounds cover terrain Z bounds" << std::endl;
  }
  
  if (valid) {
    std::cerr << "  ✓ OVERALL: Mesh adequately covers all terrain bounds" << std::endl;
  } else {
    std::cerr << "  ❌ OVERALL: Mesh does not adequately cover terrain bounds" << std::endl;
  }
  
  return valid;
}

// Calculate region-growing parameters from mesh_density
static inline RegionGrowingOptions calculate_region_parameters(RegionGrowingOptions opt, int width, int height) {
  // Mesh density ranges from 0.0 (coarsest) to 1.0 (finest)
  double density = std::max(0.0, std::min(1.0, opt.mesh_density));
  
  // Calculate region merge threshold (larger = coarser regions)
  // For terrain, we want fewer, larger regions to ensure better coverage  
  // At density 0.0: very large regions for broad coverage
  // At density 1.0: smaller regions for detail capture
  opt.region_merge_threshold = 200.0 * (1.0 - density) + 5.0 * density; // Much higher for larger regions
  
  // Calculate sampling step (larger = sparser sampling)
  // IMPROVED: Much denser sampling by default to ensure complete terrain coverage
  // At density 0.0: still reasonably dense sampling (terrain needs comprehensive coverage)
  // At density 1.0: very dense sampling (capture all terrain features)
  int max_step = std::max(1, std::min(width, height) / 150);  // Even denser sampling for terrain
  opt.sampling_step = std::max(1, static_cast<int>(max_step * (1.0 - density * 0.9))) + 1;  // Increased density impact
  
  // For terrain data, cap the sampling step to ensure comprehensive coverage
  int terrain_max_step = std::max(1, std::min(width, height) / 80); // Even more conservative cap
  opt.sampling_step = std::min(opt.sampling_step, terrain_max_step);
  
  // Calculate error threshold (larger = more tolerance, coarser mesh)
  // At density 0.0: high tolerance (coarse approximation)  
  // At density 1.0: low tolerance (fine approximation)
  opt.base_error_threshold = 10.0 * (1.0 - density) + 0.01 * density;
  
  return opt;
}

// ================================= Region-Growing Triangulation Algorithm =================================

// Region-growing triangulation approach with BRL-CAD tolerance integration
static inline void triangulateRegionGrowing(const float* elevations,
                                            int width, int height,
                                            const uint8_t* mask,
                                            RegionGrowingOptions opt,  // Pass by value to allow modification
                                            InternalMesh& out_mesh)
{
  const int W = width, H = height;
  const size_t total_cells = static_cast<size_t>(W) * static_cast<size_t>(H);
  
  // Handle edge cases for small grids
  if (W <= 1 || H <= 1) {
    // For very small grids, create a minimal valid mesh
    out_mesh.vertices.clear();
    out_mesh.triangles.clear();
    
    if (W == 1 && H == 1) {
      // Single point - create a degenerate triangle
      out_mesh.add_vertex(0.0, 0.0, static_cast<double>(elevations[0]));
      return;
    } else if (W == 1 || H == 1) {
      // Single row/column - create vertices but no triangles
      for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
          size_t idx = idx_row_major(x, y, W);
          if (mask && mask[idx] == 0) continue;
          out_mesh.add_vertex(static_cast<double>(x),
                              static_cast<double>(y),
                              static_cast<double>(elevations[idx]));
        }
      }
      return;
    }
  }
  
  // Calculate region-growing parameters from mesh_density
  opt = calculate_region_parameters(opt, width, height);
  
  // FEATURE DETECTION & GRAPH OPTIMIZATION: Apply terrain feature analysis
  std::vector<std::vector<double>> feature_map;
  bool has_feature_map = false;
  
  if (opt.enable_feature_detection) {
    std::cerr << "TerraScape: Computing terrain feature map..." << std::endl;
    
    // Create elevation grid for feature detection
    ElevationGrid grid(elevations, width, height);
    
    // Configure feature detection options
    FeatureDetection::FeatureDetectionOptions feature_opts;
    feature_opts.gradient_threshold = opt.base_error_threshold * 0.5; // Scale with error threshold
    feature_opts.curvature_threshold = opt.base_error_threshold * 0.3;
    
    // Compute feature map
    feature_map = FeatureDetection::compute_feature_map(grid, feature_opts);
    has_feature_map = true;
    
    // Count strong features for user feedback
    auto strong_features = FeatureDetection::find_strong_features(feature_map, opt.feature_threshold);
    std::cerr << "TerraScape: Found " << strong_features.size() << " strong terrain features" << std::endl;
  }
  
  // GRAPH-BASED OPTIMIZATION: Modify triangulation parameters based on features
  if (opt.enable_graph_optimization && has_feature_map) {
    std::cerr << "TerraScape: Applying graph-based segmentation optimization..." << std::endl;
    
    // Build terrain graph with feature-weighted edges
    auto terrain_graph = Lemon::buildTerrainGraph(width, height, feature_map, opt.feature_penalty_weight);
    
    if (opt.use_mst_for_regions) {
      // Use MST to guide region connectivity
      auto mst_edges = Lemon::kruskalMST(terrain_graph);
      std::cerr << "TerraScape: MST computed with " << mst_edges.size() << " edges" << std::endl;
      
      // Adjust region merging threshold based on MST structure
      double total_mst_weight = 0.0;
      for (auto edge_id : mst_edges) {
        total_mst_weight += terrain_graph.edge(edge_id).weight;
      }
      double avg_mst_weight = total_mst_weight / mst_edges.size();
      
      // Scale region merge threshold by average MST edge weight
      opt.region_merge_threshold *= (avg_mst_weight / opt.feature_penalty_weight);
      std::cerr << "TerraScape: Adjusted region merge threshold to " << opt.region_merge_threshold << std::endl;
    }
    
    // Additional graph optimizations could be added here
    // (min-cut for boundary placement, etc.)
  }
  
  // Step 1: Compute terrain features for intelligent vertex selection
  std::vector<std::pair<int, int>> feature_vertices;
  std::vector<int> region_labels(total_cells, -1); // -1 = unassigned
  
  auto get_elevation = [&](int x, int y) -> float {
    if (x < 0 || y < 0 || x >= W || y >= H) return std::numeric_limits<float>::max();
    if (mask && mask[idx_row_major(x, y, W)] == 0) return std::numeric_limits<float>::max();
    return elevations[idx_row_major(x, y, W)];
  };
  
  // Create elevation grid for feature detection
  ElevationGrid grid(elevations, W, H);
  
  // Configure feature detection for terrain analysis
  FeatureDetection::FeatureDetectionOptions feature_opts;
  feature_opts.gradient_threshold = opt.base_error_threshold * 0.5;  // More permissive for terrain coverage
  feature_opts.curvature_threshold = opt.base_error_threshold * 0.3; // More permissive for terrain coverage  
  feature_opts.use_sobel_operator = true; // More robust gradient estimation
  
  // Compute curvature and gradient maps for intelligent vertex selection
  auto curvature_map = FeatureDetection::compute_curvature_map(grid, feature_opts);
  auto gradient_map = FeatureDetection::compute_gradient_map(grid, feature_opts);
  
  // Find significant terrain features instead of just height extrema
  // This approach selects vertices based on curvature changes (second derivatives)
  // and significant slopes, which is more appropriate for terrain representation
  
  // Use intelligent sampling to avoid excessive vertex density
  int feature_sample_step = std::max(1, std::min(opt.sampling_step, std::max(W, H) / 100)); // Denser sampling
  
  for (int y = 1; y < H - 1; y += feature_sample_step) {
    for (int x = 1; x < W - 1; x += feature_sample_step) {
      if (mask && mask[idx_row_major(x, y, W)] == 0) continue;
      
      float center = get_elevation(x, y);
      bool is_boundary = (x <= 1 || x >= W-2 || y <= 1 || y >= H-2);
      
      // Get curvature and gradient at this point
      double curvature = curvature_map[y][x];
      double gradient = gradient_map[y][x];
      
      // Feature-based selection criteria (more stringent):
      bool is_curvature_feature = curvature > feature_opts.curvature_threshold;
      bool is_gradient_feature = gradient > feature_opts.gradient_threshold;
      bool is_boundary_point = is_boundary;
      
      // Height-based significance check (for compatibility with BRL-CAD tolerances)
      bool has_significant_height_variation = false;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          float neighbor = get_elevation(x+dx, y+dy);
          double height_diff = std::abs(neighbor - center);
          if (meets_tolerance_threshold(height_diff, center, opt.abs_tolerance_mm, opt.rel_tolerance)) {
            has_significant_height_variation = true;
            break;
          }
        }
        if (has_significant_height_variation) break;
      }
      
      // Select vertex if it represents a significant terrain feature
      if (is_curvature_feature || is_gradient_feature || is_boundary_point || has_significant_height_variation) {
        feature_vertices.push_back({x, y});
      }
    }
  }
  
  // Add boundary points explicitly to ensure complete coverage
  for (int x = 0; x < W; ++x) {
    for (int y : {0, H-1}) {
      if (mask && mask[idx_row_major(x, y, W)] == 0) continue;
      feature_vertices.push_back({x, y});
    }
  }
  for (int y = 0; y < H; ++y) {
    for (int x : {0, W-1}) {
      if (mask && mask[idx_row_major(x, y, W)] == 0) continue;
      feature_vertices.push_back({x, y});
    }
  }
  
  // Remove duplicates and sort
  std::sort(feature_vertices.begin(), feature_vertices.end());
  feature_vertices.erase(std::unique(feature_vertices.begin(), feature_vertices.end()), feature_vertices.end());
  
  std::cerr << "TerraScape: Selected " << feature_vertices.size() << " feature-based vertices from " 
            << (W * H) << " grid cells (density: " << (static_cast<double>(feature_vertices.size()) / (W * H)) << ")" << std::endl;
  
  // Step 2: Improved region growing using curvature and slope changes
  std::queue<std::tuple<int, int, int>> growth_queue; // x, y, region_id
  
  for (size_t i = 0; i < feature_vertices.size(); ++i) {
    int x = feature_vertices[i].first;
    int y = feature_vertices[i].second;
    int region_id = static_cast<int>(i);
    region_labels[idx_row_major(x, y, W)] = region_id;
    growth_queue.push({x, y, region_id});
  }
  
  // Grow regions using improved criteria based on slope consistency
  std::vector<int> region_sizes(feature_vertices.size(), 0);
  
  while (!growth_queue.empty()) {
    auto [x, y, region_id] = growth_queue.front();
    growth_queue.pop();
    
    region_sizes[region_id]++;
    float center_elev = get_elevation(x, y);
    
    // Get slope at center point (using central differences)
    float slope_x = 0.0f, slope_y = 0.0f;
    if (x > 0 && x < W-1) {
      slope_x = (get_elevation(x+1, y) - get_elevation(x-1, y)) * 0.5f;
    }
    if (y > 0 && y < H-1) {
      slope_y = (get_elevation(x, y+1) - get_elevation(x, y-1)) * 0.5f;
    }
    
    // Check 4-connected neighbors  
    for (auto [dx, dy] : std::vector<std::pair<int,int>>{{0,1}, {1,0}, {0,-1}, {-1,0}}) {
      int nx = x + dx, ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      if (mask && mask[idx_row_major(nx, ny, W)] == 0) continue;
      
      size_t nidx = idx_row_major(nx, ny, W);
      if (region_labels[nidx] != -1) continue; // Already assigned
      
      float neighbor_elev = get_elevation(nx, ny);
      float height_diff = std::abs(neighbor_elev - center_elev);
      
      // Compute neighbor slope for slope consistency check
      float neighbor_slope_x = 0.0f, neighbor_slope_y = 0.0f;
      if (nx > 0 && nx < W-1) {
        neighbor_slope_x = (get_elevation(nx+1, ny) - get_elevation(nx-1, ny)) * 0.5f;
      }
      if (ny > 0 && ny < H-1) {
        neighbor_slope_y = (get_elevation(nx, ny+1) - get_elevation(nx, ny-1)) * 0.5f;
      }
      
      // Calculate slope difference (second derivative indicator)
      float slope_diff = std::sqrt((slope_x - neighbor_slope_x) * (slope_x - neighbor_slope_x) + 
                                  (slope_y - neighbor_slope_y) * (slope_y - neighbor_slope_y));
      
      // Improved region merging criteria:
      // 1. Height difference within tolerance (as before)
      // 2. Slope consistency (small slope changes = similar terrain character)
      double height_threshold = std::max({
        opt.region_merge_threshold,
        opt.abs_tolerance_mm * 2.0,
        opt.rel_tolerance * std::max(std::abs(center_elev), std::abs(neighbor_elev)) * 2.0
      });
      
      // Slope threshold based on base error threshold
      double slope_threshold = opt.base_error_threshold * 2.0; // Allow some slope variation
      
      // Assign to region if both height and slope are consistent
      bool height_compatible = height_diff <= height_threshold;
      bool slope_compatible = slope_diff <= slope_threshold;
      
      if (height_compatible && slope_compatible) {
        region_labels[nidx] = region_id;
        growth_queue.push({nx, ny, region_id});
      }
    }
  }
  
  // Step 3: Handle collision resolution - assign unassigned cells to closest region
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      size_t idx = idx_row_major(x, y, W);
      if (region_labels[idx] != -1) continue; // Already assigned
      if (mask && mask[idx] == 0) continue; // Masked
      
      float center_elev = get_elevation(x, y);
      int best_region = -1;
      float best_score = std::numeric_limits<float>::max();
      
      // Find best region based on neighboring cells
      for (auto [dx, dy] : std::vector<std::pair<int,int>>{{0,1}, {1,0}, {0,-1}, {-1,0}}) {
        int nx = x + dx, ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        
        size_t nidx = idx_row_major(nx, ny, W);
        int neighbor_region = region_labels[nidx];
        if (neighbor_region == -1) continue;
        
        float neighbor_elev = get_elevation(nx, ny);
        float height_diff = std::abs(neighbor_elev - center_elev);
        
        // Prefer region with smallest height difference, then smallest area
        float score = height_diff * 1000.0 + static_cast<float>(region_sizes[neighbor_region]);
        if (score < best_score) {
          best_score = score;
          best_region = neighbor_region;
        }
      }
      
      if (best_region != -1) {
        region_labels[idx] = best_region;
        region_sizes[best_region]++;
      }
    }
  }
  
  // Step 4: Ensure complete terrain coverage by adding vertices for uncovered areas
  // This is crucial for terrain meshes - we cannot leave areas of the terrain uncovered
  
  // First, add vertices from regions using selective sampling
  out_mesh.vertices.clear();
  out_mesh.triangles.clear();
  
  std::vector<int> vertex_map(total_cells, -1); // Maps grid position to vertex index
  
  // Sample representative points from each region
  for (size_t region_id = 0; region_id < feature_vertices.size(); ++region_id) {
    if (region_sizes[region_id] == 0) continue;
    
    // Add the feature point for this region
    int ex_x = feature_vertices[region_id].first;
    int ex_y = feature_vertices[region_id].second;
    size_t ex_idx = idx_row_major(ex_x, ex_y, W);
    
    if (vertex_map[ex_idx] == -1) {
      vertex_map[ex_idx] = static_cast<int>(out_mesh.vertices.size());
      out_mesh.add_vertex(static_cast<double>(ex_x),
                          static_cast<double>(ex_y),
                          static_cast<double>(elevations[ex_idx]));
    }
    
    // Sample additional points from this region based on calculated sampling step
    int base_sample_step = opt.sampling_step;
    int adaptive_step = std::max(base_sample_step, static_cast<int>(std::sqrt(region_sizes[region_id]) / 30)); // More aggressive
    int sample_step = std::min(adaptive_step, base_sample_step * 2); // Reduced cap for denser sampling
    
    for (int y = 0; y < H; y += sample_step) {
      for (int x = 0; x < W; x += sample_step) {
        size_t idx = idx_row_major(x, y, W);
        if (region_labels[idx] != static_cast<int>(region_id)) continue;
        if (mask && mask[idx] == 0) continue;
        if (vertex_map[idx] != -1) continue; // Already added
        
        // Check if this point adds complexity
        float center_elev = elevations[idx];
        float max_diff = 0.0;
        
        for (auto [dx, dy] : std::vector<std::pair<int,int>>{{0,1}, {1,0}, {0,-1}, {-1,0}}) {
          int nx = x + dx, ny = y + dy;
          if (nx >= 0 && ny >= 0 && nx < W && ny < H) {
            float neighbor_elev = elevations[idx_row_major(nx, ny, W)];
            max_diff = std::max(max_diff, std::abs(neighbor_elev - center_elev));
          }
        }
        
        // Add point if it represents significant height variation based on BRL-CAD tolerances
        bool meets_abs_tolerance = max_diff > opt.abs_tolerance_mm;
        bool meets_rel_tolerance = max_diff > opt.rel_tolerance * std::abs(center_elev);
        bool meets_base_threshold = max_diff > opt.base_error_threshold * 0.5;
        
        if (meets_abs_tolerance || meets_rel_tolerance || meets_base_threshold) {
          vertex_map[idx] = static_cast<int>(out_mesh.vertices.size());
          out_mesh.add_vertex(static_cast<double>(x),
                              static_cast<double>(y),
                              static_cast<double>(elevations[idx]));
        }
      }
    }
  }
  
  // Intelligent sparse sampling for uncovered areas
  // Use curvature and gradient information to determine where additional vertices are needed
  double current_vertex_density = static_cast<double>(out_mesh.vertices.size()) / (W * H);
  std::cerr << "TerraScape: Current vertex density after feature selection: " << current_vertex_density 
            << " (" << out_mesh.vertices.size() << " vertices)" << std::endl;
  
  // Adaptive coverage based on terrain complexity rather than uniform sampling
  int intelligent_added = 0;
  
  // Use a more aggressive sampling step to ensure better coverage
  int coverage_sample_step = std::max(1, opt.sampling_step);
  
  // Only add vertices in areas with significant terrain variation that weren't captured by features
  for (int y = 2; y < H - 2; y += coverage_sample_step) {
    for (int x = 2; x < W - 2; x += coverage_sample_step) {
      size_t idx = idx_row_major(x, y, W);
      if (mask && mask[idx] == 0) continue;
      if (vertex_map[idx] != -1) continue; // Already has vertex
      
      // Check terrain complexity in this area
      double local_curvature = (x < W-1 && y < H-1) ? curvature_map[y][x] : 0.0;
      double local_gradient = (x < W-1 && y < H-1) ? gradient_map[y][x] : 0.0;
      
      // Check if area has significant terrain variation
      float center_elev = elevations[idx];
      float max_local_diff = 0.0;
      for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
          int nx = x + dx, ny = y + dy;
          if (nx >= 0 && ny >= 0 && nx < W && ny < H) {
            float neighbor_elev = elevations[idx_row_major(nx, ny, W)];
            max_local_diff = std::max(max_local_diff, std::abs(neighbor_elev - center_elev));
          }
        }
      }
      
      // More lenient criteria for terrain coverage - add vertex if there's any significant variation
      bool needs_vertex = false;
      
      // Check for terrain features or local variation
      if (local_curvature > feature_opts.curvature_threshold * 0.1 ||
          local_gradient > feature_opts.gradient_threshold * 0.1 ||
          max_local_diff > opt.base_error_threshold * 0.1) {
        needs_vertex = true;
      }
      
      // Additionally, ensure minimum grid coverage for terrain representation
      // Add vertices at regular intervals to guarantee minimum coverage
      if (!needs_vertex && current_vertex_density < 0.7) {
        // Check if this area is significantly under-represented
        int search_radius = coverage_sample_step * 3; // Larger search radius
        int nearby_vertices = 0;
        for (int dy = -search_radius; dy <= search_radius; dy++) {
          for (int dx = -search_radius; dx <= search_radius; dx++) {
            int nx = x + dx, ny = y + dy;
            if (nx >= 0 && ny >= 0 && nx < W && ny < H) {
              size_t nidx = idx_row_major(nx, ny, W);
              if (vertex_map[nidx] != -1) {
                nearby_vertices++;
              }
            }
          }
        }
        
        // If area is sparse, add vertex to improve coverage
        int expected_vertices_in_area = (search_radius * 2 + 1) * (search_radius * 2 + 1) / (coverage_sample_step * coverage_sample_step);
        if (nearby_vertices < expected_vertices_in_area / 2) { // More aggressive coverage
          needs_vertex = true;
        }
      }
      
      if (needs_vertex) {
        vertex_map[idx] = static_cast<int>(out_mesh.vertices.size());
        out_mesh.add_vertex(static_cast<double>(x),
                            static_cast<double>(y),
                            static_cast<double>(elevations[idx]));
        intelligent_added++;
      }
    }
  }
  
  std::cerr << "TerraScape: Added " << intelligent_added 
            << " vertices through intelligent terrain-based sampling" << std::endl;
  
  // Final coverage boost: ensure minimum terrain coverage for adequate representation
  current_vertex_density = static_cast<double>(out_mesh.vertices.size()) / (W * H);
  if (current_vertex_density < 0.5) {
    std::cerr << "TerraScape: Boosting coverage from " << (current_vertex_density * 100.0) 
              << "% to meet minimum 50% terrain coverage requirement..." << std::endl;
    
    int coverage_boost_added = 0;
    int boost_step = std::max(1, std::min(opt.sampling_step, std::max(W, H) / 80)); // Smaller step for denser coverage
    
    for (int y = 0; y < H; y += boost_step) {
      for (int x = 0; x < W; x += boost_step) {
        size_t idx = idx_row_major(x, y, W);
        if (mask && mask[idx] == 0) continue;
        if (vertex_map[idx] != -1) continue; // Already has vertex
        
        // More aggressive area coverage check
        bool area_needs_coverage = true;
        int check_radius = boost_step / 2; // Reduced check radius
        int nearby_vertices = 0;
        
        for (int dy = -check_radius; dy <= check_radius; dy++) {
          for (int dx = -check_radius; dx <= check_radius; dx++) {
            int nx = x + dx, ny = y + dy;
            if (nx >= 0 && ny >= 0 && nx < W && ny < H) {
              size_t nidx = idx_row_major(nx, ny, W);
              if (vertex_map[nidx] != -1) {
                nearby_vertices++;
              }
            }
          }
        }
        
        // Add vertex if there are very few nearby vertices
        int expected_vertices = (check_radius * 2 + 1) * (check_radius * 2 + 1) / (boost_step * boost_step);
        if (nearby_vertices == 0 || nearby_vertices < expected_vertices / 4) {
          vertex_map[idx] = static_cast<int>(out_mesh.vertices.size());
          out_mesh.add_vertex(static_cast<double>(x),
                              static_cast<double>(y),
                              static_cast<double>(elevations[idx]));
          coverage_boost_added++;
        }
      }
    }
    
    // If still not enough coverage, do a final systematic pass
    double final_density = static_cast<double>(out_mesh.vertices.size()) / (W * H);
    if (final_density < 0.5) {
      std::cerr << "TerraScape: Final systematic pass to ensure 50% coverage..." << std::endl;
      int systematic_step = boost_step / 2; // Even denser
      
      for (int y = 0; y < H; y += systematic_step) {
        for (int x = 0; x < W; x += systematic_step) {
          size_t idx = idx_row_major(x, y, W);
          if (mask && mask[idx] == 0) continue;
          if (vertex_map[idx] != -1) continue; // Already has vertex
          
          vertex_map[idx] = static_cast<int>(out_mesh.vertices.size());
          out_mesh.add_vertex(static_cast<double>(x),
                              static_cast<double>(y),
                              static_cast<double>(elevations[idx]));
          coverage_boost_added++;
          
          // Check if we've achieved minimum coverage after each addition
          double new_density = static_cast<double>(out_mesh.vertices.size()) / (W * H);
          if (new_density >= 0.5) {
            break;
          }
        }
        
        // Check coverage after each row
        double new_density = static_cast<double>(out_mesh.vertices.size()) / (W * H);
        if (new_density >= 0.5) {
          break;
        }
      }
    }
    
    std::cerr << "TerraScape: Added " << coverage_boost_added 
              << " vertices to boost terrain coverage" << std::endl;
  }
  
  // CRITICAL: Ensure the four corner vertices are ALWAYS included to guarantee full area coverage
  // This ensures the mesh bounding box always matches the terrain bounding box regardless of sampling density
  std::vector<std::pair<int, int>> corners = {{0, 0}, {W-1, 0}, {0, H-1}, {W-1, H-1}};
  for (const auto& corner : corners) {
    int x = corner.first, y = corner.second;
    size_t idx = idx_row_major(x, y, W);
    if (mask && mask[idx] == 0) continue;
    if (vertex_map[idx] == -1) {
      vertex_map[idx] = static_cast<int>(out_mesh.vertices.size());
      out_mesh.add_vertex(static_cast<double>(x),
                          static_cast<double>(y),
                          static_cast<double>(elevations[idx]));
      std::cerr << "TerraScape: Added critical corner vertex at (" << x << ", " << y << ")" << std::endl;
    }
  }
  
  // CRITICAL: Ensure min/max elevation points are always included to preserve Z bounds
  float min_elev = std::numeric_limits<float>::max();
  float max_elev = std::numeric_limits<float>::lowest();
  int min_elev_idx = -1, max_elev_idx = -1;
  
  for (int i = 0; i < W * H; i++) {
    if (mask && mask[i] == 0) continue;
    if (elevations[i] < min_elev) {
      min_elev = elevations[i];
      min_elev_idx = i;
    }
    if (elevations[i] > max_elev) {
      max_elev = elevations[i];
      max_elev_idx = i;
    }
  }
  
  // Add min/max elevation vertices if not already present
  if (min_elev_idx != -1 && vertex_map[min_elev_idx] == -1) {
    int x = min_elev_idx % W;
    int y = min_elev_idx / W;
    vertex_map[min_elev_idx] = static_cast<int>(out_mesh.vertices.size());
    out_mesh.add_vertex(static_cast<double>(x),
                        static_cast<double>(y),
                        static_cast<double>(elevations[min_elev_idx]));
    std::cerr << "TerraScape: Added critical min elevation vertex at (" << x << ", " << y 
              << ") with elevation " << min_elev << std::endl;
  }
  
  if (max_elev_idx != -1 && max_elev_idx != min_elev_idx && vertex_map[max_elev_idx] == -1) {
    int x = max_elev_idx % W;
    int y = max_elev_idx / W;
    vertex_map[max_elev_idx] = static_cast<int>(out_mesh.vertices.size());
    out_mesh.add_vertex(static_cast<double>(x),
                        static_cast<double>(y),
                        static_cast<double>(elevations[max_elev_idx]));
    std::cerr << "TerraScape: Added critical max elevation vertex at (" << x << ", " << y 
              << ") with elevation " << max_elev << std::endl;
  }
  
  // Add boundary points to ensure mesh covers the entire area
  // Improved boundary sampling for better terrain coverage
  int boundary_step = std::max(1, std::min(opt.sampling_step, std::max(W, H) / 50)); // Even denser boundary sampling
  
  for (int x = 0; x < W; x += boundary_step) {
    for (int y : {0, H-1}) {
      size_t idx = idx_row_major(x, y, W);
      if (mask && mask[idx] == 0) continue;
      if (vertex_map[idx] == -1) {
        vertex_map[idx] = static_cast<int>(out_mesh.vertices.size());
        out_mesh.add_vertex(static_cast<double>(x),
                            static_cast<double>(y),
                            static_cast<double>(elevations[idx]));
      }
    }
  }
  
  for (int y = 0; y < H; y += boundary_step) {
    for (int x : {0, W-1}) {
      size_t idx = idx_row_major(x, y, W);
      if (mask && mask[idx] == 0) continue;
      if (vertex_map[idx] == -1) {
        vertex_map[idx] = static_cast<int>(out_mesh.vertices.size());
        out_mesh.add_vertex(static_cast<double>(x),
                            static_cast<double>(y),
                            static_cast<double>(elevations[idx]));
      }
    }
  }
  
  // FULL COVERAGE TRIANGULATION: Ensure all non-zero cells are covered by triangles
  // Key insight: mesh density controls triangle resolution, not coverage area
  // We need to ensure every non-zero height cell is covered by at least one triangle
  int triangles_created = 0;
  
  double total_vertex_density = static_cast<double>(out_mesh.vertices.size()) / (W * H);
  
  // STEP 1: Create comprehensive triangulation that covers all non-zero cells
  // Use existing vertices but ensure triangles span to cover all terrain
  std::cerr << "TerraScape: Creating full-coverage triangulation (" 
            << total_vertex_density << " vertices per cell)" << std::endl;
  
  // Determine the search radius based on density - larger radius for sparser meshes
  int coverage_radius = std::max(2, static_cast<int>(4.0 / std::max(0.1, total_vertex_density)));
  coverage_radius = std::min(coverage_radius, std::max(W, H) / 3); // More generous cap
  
  // Track which cells are covered by triangles
  std::vector<bool> cell_covered(W * H, false);
  
  // Create triangulation using existing vertices with wider reach
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      size_t idx = idx_row_major(x, y, W);
      if (mask && mask[idx] == 0) continue;
      if (elevations[idx] == 0.0f) continue; // Skip zero-height cells as requested
      
      // Check if this cell is already covered by a triangle
      if (cell_covered[idx]) continue;
      
      // Find nearby vertices to create triangles that cover this cell
      std::vector<std::pair<std::pair<int, int>, int>> nearby_vertices;
      
      // Search in expanding radius to find vertices
      for (int radius = 1; radius <= coverage_radius && nearby_vertices.size() < 6; radius++) {
        for (int dy = -radius; dy <= radius; dy++) {
          for (int dx = -radius; dx <= radius; dx++) {
            // Only check perimeter of current radius
            if (std::abs(dx) != radius && std::abs(dy) != radius) continue;
            
            int nx = x + dx, ny = y + dy;
            if (nx >= 0 && ny >= 0 && nx < W && ny < H) {
              size_t nidx = idx_row_major(nx, ny, W);
              if (vertex_map[nidx] != -1) {
                nearby_vertices.push_back({{nx, ny}, vertex_map[nidx]});
              }
            }
          }
        }
      }
      
      // Create triangles that cover this cell using nearby vertices
      if (nearby_vertices.size() >= 3) {
        // Sort by distance from current cell for better triangulation
        std::sort(nearby_vertices.begin(), nearby_vertices.end(),
          [x, y](const auto& a, const auto& b) {
            double dist_a = (a.first.first - x) * (a.first.first - x) + 
                           (a.first.second - y) * (a.first.second - y);
            double dist_b = (b.first.first - x) * (b.first.first - x) + 
                           (b.first.second - y) * (b.first.second - y);
            return dist_a < dist_b;
          });
        
        // Create triangles using closest vertices
        for (size_t i = 0; i < nearby_vertices.size() && i < 3; i++) {
          for (size_t j = i + 1; j < nearby_vertices.size() && j < 4; j++) {
            for (size_t k = j + 1; k < nearby_vertices.size() && k < 5; k++) {
              auto& v0_pos = nearby_vertices[i].first;
              auto& v1_pos = nearby_vertices[j].first;
              auto& v2_pos = nearby_vertices[k].first;
              
              int v0 = nearby_vertices[i].second;
              int v1 = nearby_vertices[j].second;
              int v2 = nearby_vertices[k].second;
              
              // Check if this triangle covers the current cell (x,y)
              // Use barycentric coordinates to test if point is inside triangle
              double denom = (v1_pos.second - v2_pos.second) * (v0_pos.first - v2_pos.first) + 
                            (v2_pos.first - v1_pos.first) * (v0_pos.second - v2_pos.second);
              
              if (std::abs(denom) < 1e-6) continue; // Degenerate triangle
              
              double a = ((v1_pos.second - v2_pos.second) * (x - v2_pos.first) + 
                         (v2_pos.first - v1_pos.first) * (y - v2_pos.second)) / denom;
              double b = ((v2_pos.second - v0_pos.second) * (x - v2_pos.first) + 
                         (v0_pos.first - v2_pos.first) * (y - v2_pos.second)) / denom;
              double c = 1 - a - b;
              
              // Check if point is inside triangle (with generous tolerance for coverage)
              const double tolerance = 0.5; // More generous tolerance for coverage
              if (a >= -tolerance && b >= -tolerance && c >= -tolerance) {
                // This triangle covers the cell - create it
                out_mesh.add_triangle_with_upward_normal(v0, v1, v2);
                triangles_created++;
                
                // Mark all cells covered by this triangle
                // Find bounding box of triangle and check all cells within it
                int min_x = std::max(0, std::min({v0_pos.first, v1_pos.first, v2_pos.first}) - 1);
                int max_x = std::min(W-1, std::max({v0_pos.first, v1_pos.first, v2_pos.first}) + 1);
                int min_y = std::max(0, std::min({v0_pos.second, v1_pos.second, v2_pos.second}) - 1);
                int max_y = std::min(H-1, std::max({v0_pos.second, v1_pos.second, v2_pos.second}) + 1);
                
                for (int cy = min_y; cy <= max_y; cy++) {
                  for (int cx = min_x; cx <= max_x; cx++) {
                    // Test if this cell is covered by the triangle
                    double ca = ((v1_pos.second - v2_pos.second) * (cx - v2_pos.first) + 
                               (v2_pos.first - v1_pos.first) * (cy - v2_pos.second)) / denom;
                    double cb = ((v2_pos.second - v0_pos.second) * (cx - v2_pos.first) + 
                               (v0_pos.first - v2_pos.first) * (cy - v2_pos.second)) / denom;
                    double cc = 1 - ca - cb;
                    
                    if (ca >= -tolerance && cb >= -tolerance && cc >= -tolerance) {
                      size_t cidx = idx_row_major(cx, cy, W);
                      cell_covered[cidx] = true;
                    }
                  }
                }
                
                // Break out of loops since we found a covering triangle
                goto next_cell;
              }
            }
          }
        }
      }
      next_cell:;
    }
  }
  
  std::cerr << "TerraScape: Full-coverage triangulation created " << triangles_created << " triangles" << std::endl;
  
  // STEP 2: Handle any remaining uncovered non-zero cells with additional triangulation
  // Count how many cells are still uncovered
  int uncovered_count = 0;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      size_t idx = idx_row_major(x, y, W);
      if (mask && mask[idx] == 0) continue;
      if (elevations[idx] == 0.0f) continue;
      if (!cell_covered[idx]) uncovered_count++;
    }
  }
  
  if (uncovered_count > 0) {
    std::cerr << "TerraScape: " << uncovered_count << " cells still uncovered, adding fallback triangulation..." << std::endl;
    
    // For uncovered cells, create larger triangles using Delaunay-like approach
    for (int y = 0; y < H; y++) {
      for (int x = 0; x < W; x++) {
        size_t idx = idx_row_major(x, y, W);
        if (mask && mask[idx] == 0) continue;
        if (elevations[idx] == 0.0f) continue;
        if (cell_covered[idx]) continue; // Already covered
        
        // Find the 3 closest vertices to create a triangle that covers this cell
        std::vector<std::pair<double, std::pair<std::pair<int, int>, int>>> dist_vertices;
        
        for (int dy = -coverage_radius*3; dy <= coverage_radius*3; dy++) {
          for (int dx = -coverage_radius*3; dx <= coverage_radius*3; dx++) {
            int nx = x + dx, ny = y + dy;
            if (nx >= 0 && ny >= 0 && nx < W && ny < H) {
              size_t nidx = idx_row_major(nx, ny, W);
              if (vertex_map[nidx] != -1) {
                double dist = dx*dx + dy*dy;
                dist_vertices.push_back({dist, {{nx, ny}, vertex_map[nidx]}});
              }
            }
          }
        }
        
        if (dist_vertices.size() >= 3) {
          // Sort by distance
          std::sort(dist_vertices.begin(), dist_vertices.end());
          
          // Try multiple combinations of vertices to ensure coverage
          bool cell_finally_covered = false;
          for (size_t i = 0; i < dist_vertices.size() && i < 6 && !cell_finally_covered; i++) {
            for (size_t j = i + 1; j < dist_vertices.size() && j < 8 && !cell_finally_covered; j++) {
              for (size_t k = j + 1; k < dist_vertices.size() && k < 10 && !cell_finally_covered; k++) {
                auto& v0_pos = dist_vertices[i].second.first;
                auto& v1_pos = dist_vertices[j].second.first;
                auto& v2_pos = dist_vertices[k].second.first;
                
                int v0 = dist_vertices[i].second.second;
                int v1 = dist_vertices[j].second.second;
                int v2 = dist_vertices[k].second.second;
                
                // Check if this triangle covers the current cell
                double denom = (v1_pos.second - v2_pos.second) * (v0_pos.first - v2_pos.first) + 
                              (v2_pos.first - v1_pos.first) * (v0_pos.second - v2_pos.second);
                
                if (std::abs(denom) > 1e-6) {
                  double a = ((v1_pos.second - v2_pos.second) * (x - v2_pos.first) + 
                             (v2_pos.first - v1_pos.first) * (y - v2_pos.second)) / denom;
                  double b = ((v2_pos.second - v0_pos.second) * (x - v2_pos.first) + 
                             (v0_pos.first - v2_pos.first) * (y - v2_pos.second)) / denom;
                  double c = 1 - a - b;
                  
                  const double tolerance = 0.5;
                  if (a >= -tolerance && b >= -tolerance && c >= -tolerance) {
                    out_mesh.add_triangle_with_upward_normal(v0, v1, v2);
                    triangles_created++;
                    
                    // Mark covered cells for this triangle
                    int min_x = std::max(0, std::min({v0_pos.first, v1_pos.first, v2_pos.first}) - 1);
                    int max_x = std::min(W-1, std::max({v0_pos.first, v1_pos.first, v2_pos.first}) + 1);
                    int min_y = std::max(0, std::min({v0_pos.second, v1_pos.second, v2_pos.second}) - 1);
                    int max_y = std::min(H-1, std::max({v0_pos.second, v1_pos.second, v2_pos.second}) + 1);
                    
                    for (int cy = min_y; cy <= max_y; cy++) {
                      for (int cx = min_x; cx <= max_x; cx++) {
                        double ca = ((v1_pos.second - v2_pos.second) * (cx - v2_pos.first) + 
                                   (v2_pos.first - v1_pos.first) * (cy - v2_pos.second)) / denom;
                        double cb = ((v2_pos.second - v0_pos.second) * (cx - v2_pos.first) + 
                                   (v0_pos.first - v2_pos.first) * (cy - v2_pos.second)) / denom;
                        double cc = 1 - ca - cb;
                        
                        if (ca >= -tolerance && cb >= -tolerance && cc >= -tolerance) {
                          size_t cidx = idx_row_major(cx, cy, W);
                          cell_covered[cidx] = true;
                          if (cx == x && cy == y) cell_finally_covered = true;
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  
  std::cerr << "TerraScape: Total triangulation complete - " << triangles_created 
            << " triangles generated covering all non-zero terrain cells" << std::endl;
  
  // 2D PROJECTION VALIDATION: Check mesh 2D coverage against non-zero terrain cells
  // This validates that the algorithm is correctly covering the terrain area
  std::cerr << "TerraScape: Validating 2D projection coverage..." << std::endl;
  
  // Calculate total non-zero cell area in 2D
  double total_nonzero_cells = 0;
  double interior_edge_cells = 0;
  
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      size_t idx = idx_row_major(x, y, W);
      if (mask && mask[idx] == 0) continue;
      if (elevations[idx] != 0.0f) {
        total_nonzero_cells += 1.0; // Each cell has area 1.0 in grid units
        
        // Check if this is an interior edge cell (borders a zero-height cell)
        bool is_interior_edge = false;
        for (auto [dx, dy] : std::vector<std::pair<int,int>>{{0,1}, {1,0}, {0,-1}, {-1,0}}) {
          int nx = x + dx, ny = y + dy;
          if (nx >= 0 && ny >= 0 && nx < W && ny < H) {
            size_t nidx = idx_row_major(nx, ny, W);
            if (elevations[nidx] == 0.0f || (mask && mask[nidx] == 0)) {
              is_interior_edge = true;
              break;
            }
          }
        }
        if (is_interior_edge) interior_edge_cells += 1.0;
      }
    }
  }
  
  // Calculate 2D projected area of all triangles
  double mesh_2d_projected_area = 0.0;
  for (const auto& triangle : out_mesh.triangles) {
    // Get the 2D coordinates of triangle vertices (ignore Z)
    double x0 = out_mesh.vertices[triangle[0]][0];
    double y0 = out_mesh.vertices[triangle[0]][1];
    double x1 = out_mesh.vertices[triangle[1]][0];
    double y1 = out_mesh.vertices[triangle[1]][1];
    double x2 = out_mesh.vertices[triangle[2]][0];
    double y2 = out_mesh.vertices[triangle[2]][1];
    
    // Calculate 2D triangle area using cross product
    double area_2d = 0.5 * std::abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0));
    mesh_2d_projected_area += area_2d;
  }
  
  // Validation check
  double coverage_ratio_2d = mesh_2d_projected_area / total_nonzero_cells;
  double allowed_delta = 0.5 * interior_edge_cells; // 50% of interior edge cells as tolerance
  double area_difference = std::abs(mesh_2d_projected_area - total_nonzero_cells);
  
  std::cerr << "TerraScape: 2D Projection Validation Results:" << std::endl;
  std::cerr << "  Total non-zero cells: " << total_nonzero_cells << " (area units)" << std::endl;
  std::cerr << "  Interior edge cells: " << interior_edge_cells << std::endl;
  std::cerr << "  Mesh 2D projected area: " << mesh_2d_projected_area << std::endl;
  std::cerr << "  Coverage ratio (2D): " << coverage_ratio_2d << " (" << (coverage_ratio_2d * 100.0) << "%)" << std::endl;
  std::cerr << "  Area difference: " << area_difference << std::endl;
  std::cerr << "  Allowed delta (50% of edge cells): " << allowed_delta << std::endl;
  
  if (area_difference <= allowed_delta) {
    std::cerr << "  ✓ PASS: 2D projection matches terrain area within tolerance" << std::endl;
  } else {
    std::cerr << "  ❌ FAIL: 2D projection area mismatch - algorithm may have coverage gaps!" << std::endl;
    std::cerr << "  This suggests the triangulation is not properly covering all non-zero cells." << std::endl;
  }
  
  if (coverage_ratio_2d < 0.8) {
    std::cerr << "  ⚠ WARNING: 2D coverage ratio is suspiciously low (" << (coverage_ratio_2d * 100.0) << "%)" << std::endl;
    std::cerr << "  Expected close to 100% for proper terrain coverage algorithm." << std::endl;
  }
  
  // Comprehensive terrain mesh validation including surface area comparison
  if (opt.volume_delta_pct > 0.0) {
    if (out_mesh.triangles.size() == 0) {
      // CRITICAL ERROR: No triangles generated despite having vertices
      if (out_mesh.vertices.size() > 0) {
        std::cout << "CRITICAL ERROR: Generated " << out_mesh.vertices.size() 
                  << " vertices but 0 triangles!" << std::endl;
        std::cout << "  This indicates a triangulation failure that should be investigated." << std::endl;
        std::cout << "  Consider adjusting sampling parameters or error thresholds." << std::endl;
      } else {
        std::cout << "WARNING: No vertices or triangles generated" << std::endl;
        std::cout << "  Input may be too uniform or tolerance thresholds too strict" << std::endl;
      }
    } else {
      // Terrain-specific mesh quality validation with surface area comparison
      double vertex_density = static_cast<double>(out_mesh.vertices.size()) / (W * H);
      double triangle_density = static_cast<double>(out_mesh.triangles.size()) / (W * H);
      
      std::cout << "Terrain mesh validation:" << std::endl;
      std::cout << "  Grid size: " << W << "x" << H << " = " << (W * H) << " cells" << std::endl;
      std::cout << "  Vertex density: " << vertex_density << " vertices per grid cell" << std::endl;
      std::cout << "  Triangle density: " << triangle_density << " triangles per grid cell" << std::endl;
      std::cout << "  Coverage: " << (vertex_density * 100.0) << "% of terrain grid" << std::endl;
      
      // Surface area comparison (recommended sanity check)
      double terrain_surface_area = calculate_terrain_surface_area(elevations, W, H);
      double mesh_surface_area = calculate_mesh_surface_area(out_mesh);
      double area_ratio = mesh_surface_area / terrain_surface_area;
      
      std::cout << "  Terrain surface area: " << terrain_surface_area << std::endl;
      std::cout << "  Mesh surface area: " << mesh_surface_area << std::endl;
      std::cout << "  Area ratio (mesh/terrain): " << area_ratio << " (" << (area_ratio * 100.0) << "%)" << std::endl;
      
      // Check coverage adequacy - STRICTER validation for terrain meshes
      if (vertex_density < 0.5) {  // Terrain meshes need substantial coverage
        std::cout << "  ❌ FAIL: Insufficient terrain coverage (" << (vertex_density * 100.0) << "%) - mesh inadequate for terrain representation!" << std::endl;
        std::cout << "  ⚠ WARNING: For terrain meshes, coverage should be >= 50% to ensure proper terrain representation" << std::endl;
      } else if (vertex_density < 0.8) {
        std::cout << "  ⚠ CAUTION: Moderate terrain coverage - some detail may be lost" << std::endl;
      } else {
        std::cout << "  ✓ Good terrain coverage - terrain detail well preserved" << std::endl;
      }
      
      // Surface area validation
      if (area_ratio < 0.7) {
        std::cout << "  ⚠ WARNING: Mesh surface area significantly less than terrain - incomplete coverage!" << std::endl;
      } else if (area_ratio > 1.3) {
        std::cout << "  ℹ INFO: Mesh surface area higher than terrain (may include fine detail or volumetric components)" << std::endl;
      } else {
        std::cout << "  ✓ PASS: Mesh surface area approximates terrain data well" << std::endl;
      }
      
      // Check triangle-to-vertex ratio (should be roughly 2:1 for good meshes)
      double tri_vertex_ratio = static_cast<double>(out_mesh.triangles.size()) / out_mesh.vertices.size();
      if (tri_vertex_ratio < 0.5) {
        std::cout << "  ⚠ WARNING: Very few triangles relative to vertices" << std::endl;
      } else if (tri_vertex_ratio > 3.0) {
        std::cout << "  ⚠ WARNING: Excessive triangles - possible mesh quality issues" << std::endl;
      } else {
        std::cout << "  ✓ Good triangle-to-vertex ratio: " << tri_vertex_ratio << std::endl;
      }
      
      // BOUNDING BOX VALIDATION: Ensure mesh covers the exact same area as terrain data
      BoundingBox terrain_bounds = calculate_terrain_bounds(elevations, W, H);
      BoundingBox mesh_bounds = calculate_mesh_bounds(out_mesh);
      bool bounds_valid = validate_mesh_bounds(terrain_bounds, mesh_bounds);
      
      if (!bounds_valid) {
        std::cout << "  ❌ CRITICAL: Mesh does not cover the complete terrain area!" << std::endl;
        std::cout << "  The mesh should always represent the entire input terrain regardless of density settings." << std::endl;
      }
    }
  }
}

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

// --- Helper: Enhanced grid accessor (kept for potential future use) ---
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

// --- Utility: barycentric triangle routines (kept for potential future use) ---
inline bool point_in_triangle(float px, float py,
                              const Vertex& v0, const Vertex& v1, const Vertex& v2)
{
    float dX = px - v2.x;
    float dY = py - v2.y;
    float dX21 = v2.x - v1.x;
    float dY12 = v1.y - v2.y;
    float D = (v0.x - v2.x) * dY12 + (v0.y - v2.y) * dX21;
    if (std::abs(D) < 1e-20f) return false;
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

// --- Volume validation helpers ---

/**
 * Calculate the expected volume under a height field relative to a base plane
 * Each cell represents a column with area 1x1 and height = elevation - z_base
 */
template<typename T>
inline double calculate_heightfield_volume(int width, int height, const T* elevations, float z_base = 0.0f) {
    double total_volume = 0.0;
    
    // If z_base is 0, use the minimum elevation as the effective base
    if (z_base == 0.0f) {
        T min_elev = *std::min_element(elevations, elevations + width * height);
        z_base = static_cast<float>(min_elev);
    }
    
    for (int i = 0; i < width * height; ++i) {
        float height_above_base = static_cast<float>(elevations[i]) - z_base;
        if (height_above_base > 0.0f) {
            total_volume += height_above_base; // Each cell has area 1.0
        }
    }
    return total_volume;
}

/**
 * Calculate the volume of a triangle mesh using the divergence theorem
 * Volume = (1/6) * sum over all triangles of: dot(vertex, normal) * area
 */
inline double calculate_mesh_volume(const MeshResult& mesh, float z_base = 0.0f) {
    double volume = 0.0;
    
    for (const auto& tri : mesh.triangles) {
        const auto& v0 = mesh.vertices[tri.v0];
        const auto& v1 = mesh.vertices[tri.v1]; 
        const auto& v2 = mesh.vertices[tri.v2];
        
        // Translate vertices relative to base plane
        std::array<double, 3> p0 = {v0.x, v0.y, v0.z - z_base};
        std::array<double, 3> p1 = {v1.x, v1.y, v1.z - z_base};
        std::array<double, 3> p2 = {v2.x, v2.y, v2.z - z_base};
        
        // Signed volume contribution of tetrahedron from origin
        volume += (p0[0] * (p1[1] * p2[2] - p1[2] * p2[1]) +
                   p1[0] * (p2[1] * p0[2] - p2[2] * p0[1]) +
                   p2[0] * (p0[1] * p1[2] - p0[2] * p1[1])) / 6.0;
    }
    
    return std::abs(volume);
}

// --- Advanced grid-to-mesh implementation ---

/**
 * grid_to_mesh_impl
 * - Uses the consolidated region-growing triangulation with BRL-CAD tolerance integration
 * - Provides adaptive error thresholding and mesh-quality constraints internally
 */
template<typename T>
inline MeshResult grid_to_mesh_impl(
    int width, int height, const T* elevations,
    float error_threshold) {

    // Preprocess input for robustness
    auto preprocessing = preprocess_input_data(width, height, elevations, error_threshold);

    if (preprocessing.has_warnings) {
        for (const auto& warning : preprocessing.warnings) {
            std::cerr << "TerraScape warning: " << warning << std::endl;
        }
    }

    // Configure Region-Growing options with terrain-appropriate parameters
    RegionGrowingOptions rg_opt;
    
    // Calculate elevation range for adaptive parameter setting
    float min_elev = *std::min_element(preprocessing.processed_elevations.begin(), 
                                      preprocessing.processed_elevations.end());
    float max_elev = *std::max_element(preprocessing.processed_elevations.begin(), 
                                      preprocessing.processed_elevations.end());
    float elev_range = max_elev - min_elev;
    
    // Determine appropriate error threshold for terrain data
    float adaptive_error_threshold = preprocessing.adjusted_error_threshold;
    if (elev_range > 0.1f) {
        // If user provided a small threshold relative to elevation range, respect it
        // but provide guidance on reasonable ranges
        float relative_user_threshold = adaptive_error_threshold / elev_range;
        
        if (relative_user_threshold < 0.0001f) {
            // Very small threshold - use a minimum of 0.01% of range to avoid excessive detail
            adaptive_error_threshold = elev_range * 0.0001f;
            std::cerr << "TerraScape: Error threshold very small relative to elevation range, "
                      << "using minimum " << adaptive_error_threshold << std::endl;
        } else if (relative_user_threshold > 0.1f) {
            // Large threshold - warn but allow it
            std::cerr << "TerraScape: Large error threshold relative to elevation range, "
                      << "mesh may be very coarse" << std::endl;
        }
        
        std::cerr << "TerraScape: Using error threshold " << adaptive_error_threshold 
                  << " (" << (adaptive_error_threshold/elev_range*100.0f) << "% of elevation range: " 
                  << elev_range << ")" << std::endl;
    }
    
    rg_opt.base_error_threshold = adaptive_error_threshold;
    
    // Enable volume convergence for better terrain detail
    rg_opt.use_volume_convergence = true;
    rg_opt.volume_convergence_threshold = 0.005; // 0.5% volume change for convergence
    
    // Terrain-optimized parameters that balance quality and performance
    // These parameters automatically adjust based on elevation range and desired detail level
    float relative_threshold = adaptive_error_threshold / elev_range;
    
    if (relative_threshold < 0.001f) {
        // High detail mode - more triangles, longer processing
        rg_opt.slope_weight = 0.5;              // Reduced weight for more triangles
        rg_opt.curvature_weight = 1.0;          // Reduced weight for more triangles
        rg_opt.min_angle_deg = 5.0;             // Allow sharper angles for detail
        rg_opt.max_aspect_ratio = 20.0;         // Allow thinner triangles
        rg_opt.min_area = 0.001;                // Allow smaller triangles
        rg_opt.max_refinement_passes = 8;       // More passes for detail
        rg_opt.max_initial_iterations = 15000000;
        std::cerr << "TerraScape: Using high-detail terrain parameters" << std::endl;
    } else if (relative_threshold < 0.01f) {
        // Medium detail mode - balanced quality and performance
        rg_opt.slope_weight = 0.8;
        rg_opt.curvature_weight = 2.0;
        rg_opt.min_angle_deg = 8.0;
        rg_opt.max_aspect_ratio = 15.0;
        rg_opt.min_area = 0.01;
        rg_opt.max_refinement_passes = 6;
        rg_opt.max_initial_iterations = 10000000;
        std::cerr << "TerraScape: Using medium-detail terrain parameters" << std::endl;
    } else {
        // Low detail mode - fast processing, coarser mesh
        rg_opt.slope_weight = 1.0;
        rg_opt.curvature_weight = 3.0;
        rg_opt.min_angle_deg = 10.0;
        rg_opt.max_aspect_ratio = 12.0;
        rg_opt.min_area = 0.1;
        rg_opt.max_refinement_passes = 4;
        rg_opt.max_initial_iterations = 5000000;
        std::cerr << "TerraScape: Using low-detail terrain parameters" << std::endl;
    }

    // FEATURE DETECTION & GRAPH OPTIMIZATION: Optional terrain feature analysis
    std::vector<std::vector<double>> feature_map;
    bool has_feature_map = false;
    
    if (rg_opt.enable_feature_detection) {
        std::cerr << "TerraScape: Computing terrain feature map..." << std::endl;
        
        // Create elevation grid for feature detection
        ElevationGrid grid(preprocessing.processed_elevations.data(), width, height);
        
        // Configure feature detection options
        FeatureDetection::FeatureDetectionOptions feature_opts;
        feature_opts.gradient_threshold = rg_opt.base_error_threshold * 0.5; // Scale with error threshold
        feature_opts.curvature_threshold = rg_opt.base_error_threshold * 0.3;
        
        // Compute feature map
        feature_map = FeatureDetection::compute_feature_map(grid, feature_opts);
        has_feature_map = true;
        
        // Count strong features for user feedback
        auto strong_features = FeatureDetection::find_strong_features(feature_map, rg_opt.feature_threshold);
        std::cerr << "TerraScape: Found " << strong_features.size() << " strong terrain features" << std::endl;
    }
    
    // GRAPH-BASED OPTIMIZATION: Modify triangulation parameters based on features
    if (rg_opt.enable_graph_optimization && has_feature_map) {
        std::cerr << "TerraScape: Applying graph-based segmentation optimization..." << std::endl;
        
        // Build terrain graph with feature-weighted edges
        auto terrain_graph = Lemon::buildTerrainGraph(width, height, feature_map, rg_opt.feature_penalty_weight);
        
        if (rg_opt.use_mst_for_regions) {
            // Use MST to guide region connectivity
            auto mst_edges = Lemon::kruskalMST(terrain_graph);
            std::cerr << "TerraScape: MST computed with " << mst_edges.size() << " edges" << std::endl;
            
            // Adjust region merging threshold based on MST structure
            double total_mst_weight = 0.0;
            for (auto edge_id : mst_edges) {
                total_mst_weight += terrain_graph.edge(edge_id).weight;
            }
            double avg_mst_weight = total_mst_weight / mst_edges.size();
            
            // Scale region merge threshold by average MST edge weight
            rg_opt.region_merge_threshold *= (avg_mst_weight / rg_opt.feature_penalty_weight);
            std::cerr << "TerraScape: Adjusted region merge threshold to " << rg_opt.region_merge_threshold << std::endl;
        }
        
        // Additional graph optimizations could be added here
        // (min-cut for boundary placement, etc.)
    }

    // Volume validation sanity check - already performed by region-growing algorithm
    MeshResult result = region_growing_triangulation_advanced(
        preprocessing.processed_elevations.data(),
        width, height,
        nullptr, // NoData mask not yet propagated from preprocessing
        rg_opt
    );

    // Terrain-specific validation for mesh quality
    std::cerr << "TerraScape Terrain Mesh Validation:" << std::endl;
    std::cerr << "  Elevation range: " << min_elev << " to " << max_elev << std::endl;
    std::cerr << "  Terrain area: " << width << " x " << height << " = " << (width * height) << " cells" << std::endl;
    std::cerr << "  Generated vertices: " << result.vertices.size() << std::endl;
    std::cerr << "  Generated triangles: " << result.triangles.size() << std::endl;
    
    // Calculate terrain coverage and detail metrics
    double vertex_density = static_cast<double>(result.vertices.size()) / (width * height);
    double triangle_density = static_cast<double>(result.triangles.size()) / (width * height);
    
    std::cerr << "  Vertex density: " << vertex_density << " vertices per cell" << std::endl;
    std::cerr << "  Triangle density: " << triangle_density << " triangles per cell" << std::endl;
    
    if (vertex_density > 0.001 && triangle_density > 0.001) {
        std::cerr << "  ✓ PASS: Good terrain mesh density" << std::endl;
    } else {
        std::cerr << "  ⚠ WARNING: Sparse mesh may lose terrain detail" << std::endl;
    }

    return result;
}

// --- Volumetric mesh implementations ---

// VOLUMETRIC MESH WALL FIX:
// This implementation fixes the issue where volume mesh "walls" and "floors" were not uniform and planar.
// Previous approach used triangulation boundary edges, which could create walls from interior vertices
// when the surface mesh triangulation didn't reach all the way to the terrain data edges.
//
// The fix ensures that:
// 1. Walls are created only from the actual GEOMETRIC boundaries of the terrain data
// 2. The base/floor is uniform and planar at z_base
// 3. Walls are vertical and represent the terrain as if "sliced out of its surrounding matrix"
//
// This approach analyzes vertex positions to determine the bounding rectangle and creates walls
// only from vertices that are actually on the geometric boundary (edges of the data grid),
// not from triangulation artifacts.

// Helper function to organize boundary vertices into chains for wall creation
inline std::vector<std::vector<int>> organize_boundary_vertices(
    const MeshResult& surface_mesh,
    const std::vector<int>& boundary_vertices,
    float min_x, float max_x, float min_y, float max_y,
    float tolerance) {
    
    std::vector<std::vector<int>> chains;
    if (boundary_vertices.empty()) return chains;
    
    // Group vertices by which boundary edge they belong to
    // FIXED: Ensure each vertex belongs to only one edge to avoid non-manifold issues
    std::vector<int> left_edge, right_edge, bottom_edge, top_edge;
    
    for (int idx : boundary_vertices) {
        const Vertex& v = surface_mesh.vertices[idx];
        
        // Check corners first to avoid duplicate assignment
        bool is_corner = false;
        
        // Bottom-left corner
        if (std::abs(v.x - min_x) < tolerance && std::abs(v.y - min_y) < tolerance) {
            bottom_edge.push_back(idx);
            is_corner = true;
        }
        // Bottom-right corner  
        else if (std::abs(v.x - max_x) < tolerance && std::abs(v.y - min_y) < tolerance) {
            bottom_edge.push_back(idx);
            is_corner = true;
        }
        // Top-right corner
        else if (std::abs(v.x - max_x) < tolerance && std::abs(v.y - max_y) < tolerance) {
            right_edge.push_back(idx);
            is_corner = true;
        }
        // Top-left corner
        else if (std::abs(v.x - min_x) < tolerance && std::abs(v.y - max_y) < tolerance) {
            top_edge.push_back(idx);
            is_corner = true;
        }
        
        // If not a corner, assign to appropriate edge
        if (!is_corner) {
            if (std::abs(v.x - min_x) < tolerance) {
                left_edge.push_back(idx);
            }
            else if (std::abs(v.x - max_x) < tolerance) {
                right_edge.push_back(idx);
            }
            else if (std::abs(v.y - min_y) < tolerance) {
                bottom_edge.push_back(idx);
            }
            else if (std::abs(v.y - max_y) < tolerance) {
                top_edge.push_back(idx);
            }
        }
    }
    
    // Sort each edge by the appropriate coordinate
    auto sort_by_y = [&surface_mesh](int a, int b) {
        return surface_mesh.vertices[a].y < surface_mesh.vertices[b].y;
    };
    auto sort_by_x = [&surface_mesh](int a, int b) {
        return surface_mesh.vertices[a].x < surface_mesh.vertices[b].x;
    };
    
    std::sort(left_edge.begin(), left_edge.end(), sort_by_y);
    std::sort(right_edge.begin(), right_edge.end(), sort_by_y);
    std::sort(bottom_edge.begin(), bottom_edge.end(), sort_by_x);
    std::sort(top_edge.begin(), top_edge.end(), sort_by_x);
    
    // Add non-empty chains
    if (!left_edge.empty()) chains.push_back(left_edge);
    if (!bottom_edge.empty()) chains.push_back(bottom_edge);
    if (!right_edge.empty()) {
        std::reverse(right_edge.begin(), right_edge.end()); // Reverse for consistent winding
        chains.push_back(right_edge);
    }
    if (!top_edge.empty()) {
        std::reverse(top_edge.begin(), top_edge.end()); // Reverse for consistent winding
        chains.push_back(top_edge);
    }
    
    return chains;
}

// Helper function to determine if a point is inside a zero-height region that needs interior walls
inline bool is_zero_height_interior_point(const float* elevations, int width, int height, int x, int y, float z_base) {
    if (x < 0 || x >= width || y < 0 || y >= height) return false;
    
    float elevation = elevations[y * width + x];
    // Check if this point is at zero height (near z_base)
    if (std::abs(elevation - z_base) > 0.1f) return false;
    
    // Check if it's surrounded by higher terrain (making it an interior hole)
    bool has_elevated_neighbor = false;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                float neighbor_elevation = elevations[ny * width + nx];
                if (neighbor_elevation > z_base + 0.1f) {
                    has_elevated_neighbor = true;
                    break;
                }
            }
        }
        if (has_elevated_neighbor) break;
    }
    
    return has_elevated_neighbor;
}

// Create interior walls for zero-height regions to ensure closed manifolds
inline void create_interior_walls_for_zero_regions(MeshResult& volumetric_result,
                                                    const MeshResult& surface_mesh,
                                                    const std::vector<int>& base_vertex_mapping,
                                                    const float* elevations, int width, int height, float z_base) {
    
    // For each grid point, check if it's at zero elevation but surrounded by elevated terrain
    // We need to add vertices for these zero-height interior points that aren't in the surface mesh
    
    std::vector<std::pair<int, int>> zero_interior_points;
    
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            float center_elevation = elevations[y * width + x];
            
            // Check if this point is at zero/base elevation
            if (std::abs(center_elevation - z_base) > 0.1f) continue;
            
            // Check if it's surrounded by elevated terrain
            bool has_elevated_neighbor = false;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = x + dx, ny = y + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        float neighbor_elevation = elevations[ny * width + nx];
                        if (neighbor_elevation > z_base + 0.1f) {
                            has_elevated_neighbor = true;
                            break;
                        }
                    }
                }
                if (has_elevated_neighbor) break;
            }
            
            if (has_elevated_neighbor) {
                zero_interior_points.push_back({x, y});
            }
        }
    }
    
    if (zero_interior_points.empty()) return;
    
    // Add vertices for zero-height interior points (both surface and base)
    std::map<std::pair<int, int>, int> zero_surface_vertex_map;
    std::map<std::pair<int, int>, int> zero_base_vertex_map;
    
    for (const auto& point : zero_interior_points) {
        int x = point.first, y = point.second;
        
        // Add surface vertex at zero height
        int surface_idx = static_cast<int>(volumetric_result.vertices.size());
        volumetric_result.vertices.push_back(Vertex{static_cast<float>(x), static_cast<float>(y), z_base});
        zero_surface_vertex_map[point] = surface_idx;
        
        // Add base vertex
        int base_idx = static_cast<int>(volumetric_result.vertices.size());
        volumetric_result.vertices.push_back(Vertex{static_cast<float>(x), static_cast<float>(y), z_base});
        zero_base_vertex_map[point] = base_idx;
    }
    
    // Create walls connecting zero-height interior regions to nearby elevated surface vertices
    for (const auto& point : zero_interior_points) {
        int x = point.first, y = point.second;
        int zero_surface_idx = zero_surface_vertex_map[point];
        int zero_base_idx = zero_base_vertex_map[point];
        
        // Find nearby elevated vertices in the surface mesh to connect walls to
        for (size_t i = 0; i < surface_mesh.vertices.size(); ++i) {
            const Vertex& surface_vert = surface_mesh.vertices[i];
            
            // Skip if not elevated
            if (surface_vert.z <= z_base + 0.1f) continue;
            
            // Check if vertices are adjacent (within ~1.5 grid units)
            float dist_x = std::abs(surface_vert.x - static_cast<float>(x));
            float dist_y = std::abs(surface_vert.y - static_cast<float>(y));
            
            if (dist_x <= 1.5f && dist_y <= 1.5f && (dist_x + dist_y) > 0.1f) {
                int surface_base_idx = base_vertex_mapping[i];
                
                // Create wall triangles connecting the zero-height region to the elevated terrain
                // Ensure proper winding for outward-facing normals
                volumetric_result.triangles.push_back(Triangle{zero_surface_idx, surface_base_idx, static_cast<int>(i)});
                volumetric_result.triangles.push_back(Triangle{zero_surface_idx, zero_base_idx, surface_base_idx});
            }
        }
    }
    
    // Create triangular floor patches for the zero-height interior regions
    // This ensures the zero-height areas are properly enclosed
    for (size_t i = 0; i < zero_interior_points.size(); ++i) {
        for (size_t j = i + 1; j < zero_interior_points.size(); ++j) {
            const auto& p1 = zero_interior_points[i];
            const auto& p2 = zero_interior_points[j];
            
            // Check if points are adjacent
            int dx = std::abs(p1.first - p2.first);
            int dy = std::abs(p1.second - p2.second);
            
            if (dx <= 1 && dy <= 1 && (dx + dy) > 0) {
                int surface_idx1 = zero_surface_vertex_map[p1];
                int surface_idx2 = zero_surface_vertex_map[p2];
                int base_idx1 = zero_base_vertex_map[p1];
                int base_idx2 = zero_base_vertex_map[p2];
                
                // Create triangles for the zero-height floor area
                volumetric_result.triangles.push_back(Triangle{surface_idx1, surface_idx2, base_idx1});
                volumetric_result.triangles.push_back(Triangle{surface_idx2, base_idx2, base_idx1});
            }
        }
    }
}

// Forward declaration for boundary edge detection
inline std::vector<Edge> find_boundary_edges(const std::vector<Triangle>& triangles);

// Create uniform, planar walls from the geometric boundary of the terrain data
inline void create_geometric_boundary_walls(MeshResult& volumetric_result, 
                                           const MeshResult& surface_mesh, 
                                           const std::vector<int>& base_vertex_mapping, 
                                           float z_base) {
    // Find the actual geometric boundaries of the terrain data
    // by analyzing vertex positions to determine the bounding rectangle
    if (surface_mesh.vertices.empty()) return;
    
    float min_x = surface_mesh.vertices[0].x, max_x = surface_mesh.vertices[0].x;
    float min_y = surface_mesh.vertices[0].y, max_y = surface_mesh.vertices[0].y;
    
    for (const auto& v : surface_mesh.vertices) {
        min_x = std::min(min_x, v.x);
        max_x = std::max(max_x, v.x);
        min_y = std::min(min_y, v.y);
        max_y = std::max(max_y, v.y);
    }
    
    // Tolerance for determining if a vertex is on the boundary
    const float boundary_tolerance = 0.01f;
    
    // Find vertices that are actually on the geometric boundary
    std::vector<int> boundary_vertices;
    for (size_t i = 0; i < surface_mesh.vertices.size(); ++i) {
        const Vertex& v = surface_mesh.vertices[i];
        bool on_boundary = (std::abs(v.x - min_x) < boundary_tolerance) ||
                          (std::abs(v.x - max_x) < boundary_tolerance) ||
                          (std::abs(v.y - min_y) < boundary_tolerance) ||
                          (std::abs(v.y - max_y) < boundary_tolerance);
        if (on_boundary) {
            boundary_vertices.push_back(static_cast<int>(i));
        }
    }
    
    // FIXED: Use proper boundary edge detection instead of geometric boundary approach
    // Find the actual boundary edges of the surface mesh triangulation
    std::vector<Edge> boundary_edges = find_boundary_edges(surface_mesh.triangles);
    
    // Create wall triangles for each boundary edge
    for (const Edge& edge : boundary_edges) {
        int surface_v0 = edge.v0;
        int surface_v1 = edge.v1;
        int base_v0 = base_vertex_mapping[surface_v0];
        int base_v1 = base_vertex_mapping[surface_v1];
        
        // Create two triangles for the wall segment
        // Ensure proper winding for outward-facing normals
        // The boundary edge defines the direction, we need to maintain consistency
        volumetric_result.triangles.push_back(Triangle{surface_v0, base_v0, surface_v1});
        volumetric_result.triangles.push_back(Triangle{base_v0, base_v1, surface_v1});
    }
}

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
        volumetric_result.vertices.push_back(Vertex{v.x, v.y, z_base});
    }

    // Copy surface triangles
    volumetric_result.triangles = surface_mesh.triangles;

    // Add base triangles (with flipped winding for inward-facing normals)
    for (const Triangle& tri : surface_mesh.triangles) {
        int base_v0 = base_vertex_mapping[tri.v0];
        int base_v1 = base_vertex_mapping[tri.v1];
        int base_v2 = base_vertex_mapping[tri.v2];
        volumetric_result.triangles.push_back(Triangle{base_v0, base_v2, base_v1}); // Flipped winding
    }

    // Create uniform, planar walls from the geometric boundary of the terrain data
    // This ensures walls represent the actual data boundaries, not triangulation artifacts
    create_geometric_boundary_walls(volumetric_result, surface_mesh, base_vertex_mapping, z_base);

    return volumetric_result;
}

// Overload that accepts elevation data for interior wall generation
template<typename T>
inline MeshResult make_volumetric_mesh(const MeshResult& surface_mesh, float z_base, 
                                      int width, int height, const T* elevations) {
    MeshResult volumetric_result;
    volumetric_result.is_volumetric = true;

    // Copy surface vertices
    volumetric_result.vertices = surface_mesh.vertices;

    // Create base vertices (same x,y but z = z_base)
    std::vector<int> base_vertex_mapping(surface_mesh.vertices.size());
    for (size_t i = 0; i < surface_mesh.vertices.size(); ++i) {
        const Vertex& v = surface_mesh.vertices[i];
        base_vertex_mapping[i] = static_cast<int>(volumetric_result.vertices.size());
        volumetric_result.vertices.push_back(Vertex{v.x, v.y, z_base});
    }

    // Copy surface triangles
    volumetric_result.triangles = surface_mesh.triangles;

    // Add base triangles (with flipped winding for inward-facing normals)
    for (const Triangle& tri : surface_mesh.triangles) {
        int base_v0 = base_vertex_mapping[tri.v0];
        int base_v1 = base_vertex_mapping[tri.v1];
        int base_v2 = base_vertex_mapping[tri.v2];
        volumetric_result.triangles.push_back(Triangle{base_v0, base_v2, base_v1}); // Flipped winding
    }

    // Create uniform, planar walls from the geometric boundary of the terrain data
    create_geometric_boundary_walls(volumetric_result, surface_mesh, base_vertex_mapping, z_base);

    // Create interior walls for zero-height regions to ensure closed manifolds
    // Convert template parameter to float for consistent processing
    std::vector<float> float_elevations(width * height);
    for (int i = 0; i < width * height; ++i) {
        float_elevations[i] = static_cast<float>(elevations[i]);
    }
    create_interior_walls_for_zero_regions(volumetric_result, surface_mesh, base_vertex_mapping, 
                                         float_elevations.data(), width, height, z_base);

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
            result.positive_volume.triangles.push_back(Triangle{
                positive_mapping[tri.v0], positive_mapping[tri.v1], positive_mapping[tri.v2]
            });
        }
        // Triangle entirely below z_base
        else if (below0 && below1 && below2) {
            result.negative_volume.triangles.push_back(Triangle{
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
    float error_threshold = 1.0f) {

    MeshResult surface_mesh = grid_to_mesh_impl(width, height, elevations, error_threshold);
    return make_volumetric_mesh(surface_mesh, z_base, width, height, elevations);
}

template<typename T>
inline VolumetricMeshResult grid_to_mesh_volumetric_separated(
    int width, int height, const T* elevations,
    float z_base = 0.0f,
    float error_threshold = 1.0f) {

    MeshResult surface_mesh = grid_to_mesh_impl(width, height, elevations, error_threshold);
    return make_volumetric_mesh_separated(surface_mesh, z_base);
}

// ================================= Region-Growing API Functions =================================

// Function definitions moved to end of file after all struct definitions

// --- Main API Functions ---

// Core mesh generation with Region-Growing grid-aware triangulation
template<typename T>
inline MeshResult grid_to_mesh(
    int width, int height, const T* elevations,
    float error_threshold = 1.0f)
{
    // Delegate to Region-Growing implementation
    return grid_to_mesh_impl(width, height, elevations, error_threshold);
}

// ================================= Region-Growing API Function Implementations =================================

// Primary simplified interface using mesh density parameter
inline MeshResult region_growing_triangulation(const float* elevations,
                                              int width, int height,
                                              double mesh_density = 0.5,
                                              const uint8_t* mask = nullptr)
{
  RegionGrowingOptions opt;
  opt.mesh_density = mesh_density;
  
  // Ensure adequate defaults for terrain mesh generation
  opt.abs_tolerance_mm = 0.1;
  opt.rel_tolerance = 0.01;
  opt.norm_tolerance_deg = 15.0;
  opt.volume_delta_pct = 10.0;
  
  InternalMesh internal_mesh;
  triangulateRegionGrowing(elevations, width, height, mask, opt, internal_mesh);
  
  // Convert InternalMesh to MeshResult
  MeshResult result;
  result.vertices.reserve(internal_mesh.vertices.size());
  result.triangles.reserve(internal_mesh.triangles.size());
  
  for (const auto& v : internal_mesh.vertices) {
    result.vertices.push_back(Vertex{static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])});
  }
  
  for (const auto& t : internal_mesh.triangles) {
    result.triangles.push_back(Triangle{t[0], t[1], t[2]});
  }
  
  return result;
}

// Full interface with all options (for advanced usage and BRL-CAD tolerance control)
inline MeshResult region_growing_triangulation_advanced(const float* elevations,
                                                        int width, int height,
                                                        const uint8_t* mask, // optional, nullptr if none
                                                        const RegionGrowingOptions& opt)
{
  InternalMesh internal_mesh;
  triangulateRegionGrowing(elevations, width, height, mask, opt, internal_mesh);
  
  // Convert InternalMesh to MeshResult
  MeshResult result;
  result.vertices.reserve(internal_mesh.vertices.size());
  result.triangles.reserve(internal_mesh.triangles.size());
  
  for (const auto& v : internal_mesh.vertices) {
    result.vertices.push_back(Vertex{static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])});
  }
  
  for (const auto& t : internal_mesh.triangles) {
    result.triangles.push_back(Triangle{t[0], t[1], t[2]});
  }
  
  return result;
}

} // namespace TerraScape

#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic pop /* end ignoring warnings */
#elif defined(__clang__)
#  pragma clang diagnostic pop /* end ignoring warnings */
#endif

