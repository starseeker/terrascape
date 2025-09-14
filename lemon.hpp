#pragma once

/*
 * lemon.hpp - Minimal Graph Data Structures and Algorithms
 *
 * Header-only implementation of essential graph structures and algorithms
 * extracted from LEMON library. Provides undirected graphs, MST (Kruskal/Prim),
 * and min-cut/max-flow algorithms for terrain segmentation optimization.
 */

#include <vector>
#include <map>
#include <set>
#include <queue>
#include <algorithm>
#include <limits>
#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace TerraScape {
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
} // namespace TerraScape