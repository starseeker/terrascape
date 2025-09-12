#include "GridTriangulator.h"
#include "TerraScape.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <limits>

namespace TerraScape {

GridTriangulator::GridTriangulator() 
    : grid_width_(0), grid_height_(0)
    , min_x_(0.0f), min_y_(0.0f), max_x_(0.0f), max_y_(0.0f) {
}

GridTriangulator::~GridTriangulator() = default;

void GridTriangulator::initializeGrid(int width, int height, float min_x, float min_y, float max_x, float max_y) {
    clear();
    
    grid_width_ = width;
    grid_height_ = height;
    min_x_ = min_x;
    min_y_ = min_y;
    max_x_ = max_x;
    max_y_ = max_y;
    
    // Initialize grid mapping
    grid_to_vertex_.assign(width * height, UINT32_MAX);
}

void GridTriangulator::clear() {
    vertices_.clear();
    triangles_.clear();
    grid_to_vertex_.clear();
    
    // Clear advancing front
    while (!advancing_front_.empty()) {
        advancing_front_.pop();
    }
    processed_edges_.clear();
}

uint32_t GridTriangulator::addGridPoint(int grid_x, int grid_y, float z) {
    if (!isValidGridCoordinate(grid_x, grid_y)) {
        return UINT32_MAX;
    }
    
    float x = gridToWorldX(grid_x);
    float y = gridToWorldY(grid_y);
    
    uint32_t vertex_index = static_cast<uint32_t>(vertices_.size());
    vertices_.emplace_back(x, y, z, grid_x, grid_y, vertex_index);
    
    // Update grid mapping
    grid_to_vertex_[grid_y * grid_width_ + grid_x] = vertex_index;
    
    return vertex_index;
}

uint32_t GridTriangulator::addPoint(float x, float y, float z) {
    uint32_t vertex_index = static_cast<uint32_t>(vertices_.size());
    
    // Try to find closest grid coordinates
    auto grid_coords = worldToGrid(x, y);
    
    vertices_.emplace_back(x, y, z, grid_coords.first, grid_coords.second, vertex_index);
    
    return vertex_index;
}

bool GridTriangulator::triangulate() {
    if (vertices_.size() < 3) {
        std::cerr << "GridTriangulator: Not enough vertices for triangulation\n";
        return false;
    }
    
    // Step 1: Initialize boundary triangulation
    initializeBoundaryTriangulation();
    
    // Step 2: Advance front until complete
    while (!advancing_front_.empty()) {
        if (!advanceFrontStep()) {
            std::cerr << "GridTriangulator: Advancing front step failed\n";
            break;
        }
    }
    
    std::cout << "GridTriangulator: Created " << triangles_.size() << " triangles from " 
              << vertices_.size() << " vertices\n";
    
    return !triangles_.empty();
}

void GridTriangulator::initializeBoundaryTriangulation() {
    // Find boundary vertices (corners and edges)
    std::vector<uint32_t> boundary_vertices;
    
    // Add vertices in boundary order (counter-clockwise)
    for (const auto& vertex : vertices_) {
        bool is_boundary = (vertex.grid_x == 0 || vertex.grid_x == grid_width_ - 1 ||
                           vertex.grid_y == 0 || vertex.grid_y == grid_height_ - 1);
        if (is_boundary) {
            boundary_vertices.push_back(vertex.index);
        }
    }
    
    // If we have very few vertices, create a simple triangulation
    if (vertices_.size() <= 4) {
        if (vertices_.size() == 3) {
            addTriangle(0, 1, 2);
        } else if (vertices_.size() == 4) {
            // Check which diagonal gives better triangles
            float qual1 = calculateTriangleQuality(0, 1, 2) + calculateTriangleQuality(0, 2, 3);
            float qual2 = calculateTriangleQuality(0, 1, 3) + calculateTriangleQuality(1, 2, 3);
            
            if (qual1 >= qual2) {
                addTriangle(0, 1, 2);
                addTriangle(0, 2, 3);
            } else {
                addTriangle(0, 1, 3);
                addTriangle(1, 2, 3);
            }
        }
        return;
    }
    
    // Create initial boundary triangles and populate advancing front
    // For now, create a simple quad from corners and initialize front
    if (boundary_vertices.size() >= 4) {
        // Sort boundary vertices by position for consistent triangulation
        std::sort(boundary_vertices.begin(), boundary_vertices.end(), 
                 [this](uint32_t a, uint32_t b) {
                     const auto& va = vertices_[a];
                     const auto& vb = vertices_[b];
                     if (va.y != vb.y) return va.y < vb.y;
                     return va.x < vb.x;
                 });
        
        // Create initial boundary triangulation (simple approach)
        // Find corners
        uint32_t bottom_left = boundary_vertices[0];
        uint32_t bottom_right = boundary_vertices[0];
        uint32_t top_left = boundary_vertices[0];
        uint32_t top_right = boundary_vertices[0];
        
        for (uint32_t v_idx : boundary_vertices) {
            const auto& v = vertices_[v_idx];
            if (v.x <= vertices_[bottom_left].x && v.y <= vertices_[bottom_left].y) bottom_left = v_idx;
            if (v.x >= vertices_[bottom_right].x && v.y <= vertices_[bottom_right].y) bottom_right = v_idx;
            if (v.x <= vertices_[top_left].x && v.y >= vertices_[top_left].y) top_left = v_idx;
            if (v.x >= vertices_[top_right].x && v.y >= vertices_[top_right].y) top_right = v_idx;
        }
        
        // Create initial boundary quad
        addTriangle(bottom_left, bottom_right, top_right);
        addTriangle(bottom_left, top_right, top_left);
        
        // Initialize advancing front with boundary edges
        advancing_front_.push(AdvancingEdge(bottom_left, bottom_right, true));
        advancing_front_.push(AdvancingEdge(bottom_right, top_right, true));
        advancing_front_.push(AdvancingEdge(top_right, top_left, true));
        advancing_front_.push(AdvancingEdge(top_left, bottom_left, true));
    }
}

bool GridTriangulator::advanceFrontStep() {
    if (advancing_front_.empty()) {
        return false;
    }
    
    // Get the best edge to advance
    AdvancingEdge current_edge = advancing_front_.top();
    advancing_front_.pop();
    
    // Skip if edge already processed
    if (isEdgeProcessed(current_edge.v0, current_edge.v1)) {
        return true;
    }
    
    // Find best interior point to form triangle
    uint32_t best_point = findBestInteriorPoint(current_edge);
    
    if (best_point == UINT32_MAX) {
        // No suitable point found, mark edge as processed
        markEdgeProcessed(current_edge.v0, current_edge.v1);
        return true;
    }
    
    // Create triangle and update front
    addTriangle(current_edge.v0, current_edge.v1, best_point);
    updateAdvancingFront(current_edge.v0, current_edge.v1, best_point);
    
    // Mark original edge as processed
    markEdgeProcessed(current_edge.v0, current_edge.v1);
    
    return true;
}

uint32_t GridTriangulator::findBestInteriorPoint(const AdvancingEdge& edge) {
    uint32_t best_point = UINT32_MAX;
    float best_quality = -1.0f;
    
    const GridVertex& v0 = vertices_[edge.v0];
    const GridVertex& v1 = vertices_[edge.v1];
    
    // Search for interior points that could form good triangles
    for (uint32_t i = 0; i < vertices_.size(); ++i) {
        if (i == edge.v0 || i == edge.v1) continue;
        
        const GridVertex& candidate = vertices_[i];
        
        // Check if point is on the correct side of the edge (interior)
        if (!isLeftTurn(v0, v1, candidate)) continue;
        
        // Calculate triangle quality
        float quality = calculateTriangleQuality(edge.v0, edge.v1, i);
        
        // Check if this triangle would intersect with existing triangles
        // (simplified check - in full implementation would need proper intersection tests)
        
        if (quality > best_quality) {
            best_quality = quality;
            best_point = i;
        }
    }
    
    return best_point;
}

void GridTriangulator::addTriangle(uint32_t v0, uint32_t v1, uint32_t v2) {
    // Ensure counter-clockwise orientation
    const GridVertex& va = vertices_[v0];
    const GridVertex& vb = vertices_[v1];
    const GridVertex& vc = vertices_[v2];
    
    // Calculate signed area to determine orientation
    float signed_area = (vb.x - va.x) * (vc.y - va.y) - (vb.y - va.y) * (vc.x - va.x);
    
    if (signed_area > 0) {
        // Counter-clockwise, correct orientation
        triangles_.emplace_back(v0, v1, v2);
    } else {
        // Clockwise, flip to counter-clockwise
        triangles_.emplace_back(v0, v2, v1);
    }
    
    // Calculate and store triangle quality
    triangles_.back().quality = calculateTriangleQuality(v0, v1, v2);
}

void GridTriangulator::updateAdvancingFront(uint32_t v0, uint32_t v1, uint32_t v2) {
    // Add new edges to the advancing front (if not already processed)
    if (!isEdgeProcessed(v1, v2)) {
        AdvancingEdge edge1(v1, v2);
        edge1.quality_metric = calculateEdgeQuality(edge1);
        advancing_front_.push(edge1);
    }
    
    if (!isEdgeProcessed(v2, v0)) {
        AdvancingEdge edge2(v2, v0);
        edge2.quality_metric = calculateEdgeQuality(edge2);
        advancing_front_.push(edge2);
    }
}

float GridTriangulator::calculateTriangleQuality(uint32_t v0, uint32_t v1, uint32_t v2) const {
    const GridVertex& a = vertices_[v0];
    const GridVertex& b = vertices_[v1];
    const GridVertex& c = vertices_[v2];
    
    // Calculate edge lengths
    float ab = std::sqrt(distanceSquared(a, b));
    float bc = std::sqrt(distanceSquared(b, c));
    float ca = std::sqrt(distanceSquared(c, a));
    
    // Calculate area
    float area = std::abs(triangleArea(a, b, c));
    
    // Quality metric: ratio of area to perimeter squared (normalized)
    float perimeter = ab + bc + ca;
    if (perimeter < 1e-10f || area < 1e-10f) return 0.0f;
    
    // This gives 1.0 for equilateral triangles, lower for poor aspect ratios
    return (4.0f * std::sqrt(3.0f) * area) / (perimeter * perimeter);
}

float GridTriangulator::calculateEdgeQuality(const AdvancingEdge& edge) const {
    // Simple quality metric: prefer shorter edges
    const GridVertex& v0 = vertices_[edge.v0];
    const GridVertex& v1 = vertices_[edge.v1];
    
    float length = std::sqrt(distanceSquared(v0, v1));
    return -length; // Negative because priority queue is max-heap, we want min length
}

MeshResult GridTriangulator::toMeshResult() const {
    MeshResult result;
    
    // Convert vertices
    result.vertices.reserve(vertices_.size());
    for (const auto& vertex : vertices_) {
        result.vertices.emplace_back(Vertex{vertex.x, vertex.y, vertex.z});
    }
    
    // Convert triangles
    result.triangles.reserve(triangles_.size());
    for (const auto& triangle : triangles_) {
        result.triangles.emplace_back(Triangle{static_cast<int>(triangle.v0), static_cast<int>(triangle.v1), static_cast<int>(triangle.v2)});
    }
    
    return result;
}

void GridTriangulator::optimizeTriangleQuality() {
    // Implement edge swapping for triangle quality improvement
    // This is a simplified version - full implementation would be more sophisticated
    
    bool improved = true;
    int iterations = 0;
    const int MAX_ITERATIONS = 10;
    
    while (improved && iterations < MAX_ITERATIONS) {
        improved = false;
        iterations++;
        
        for (size_t i = 0; i < triangles_.size(); ++i) {
            for (size_t j = i + 1; j < triangles_.size(); ++j) {
                // Check if triangles share an edge
                const auto& tri1 = triangles_[i];
                const auto& tri2 = triangles_[j];
                
                // Find shared edge (simplified - should use proper edge matching)
                uint32_t shared_vertices = 0;
                uint32_t shared_v0 = UINT32_MAX, shared_v1 = UINT32_MAX;
                
                // This is a simplified shared edge detection
                std::vector<uint32_t> tri1_verts = {tri1.v0, tri1.v1, tri1.v2};
                std::vector<uint32_t> tri2_verts = {tri2.v0, tri2.v1, tri2.v2};
                
                for (uint32_t v1 : tri1_verts) {
                    for (uint32_t v2 : tri2_verts) {
                        if (v1 == v2) {
                            if (shared_vertices == 0) shared_v0 = v1;
                            else if (shared_vertices == 1) shared_v1 = v1;
                            shared_vertices++;
                        }
                    }
                }
                
                if (shared_vertices == 2) {
                    // Triangles share an edge, check if swapping improves quality
                    if (canSwapEdge(static_cast<uint32_t>(i), static_cast<uint32_t>(j), shared_v0, shared_v1)) {
                        swapEdge(static_cast<uint32_t>(i), static_cast<uint32_t>(j), shared_v0, shared_v1);
                        improved = true;
                    }
                }
            }
        }
    }
    
    std::cout << "GridTriangulator: Quality optimization completed in " << iterations << " iterations\n";
}

// Helper method implementations
bool GridTriangulator::isValidGridCoordinate(int x, int y) const {
    return x >= 0 && x < grid_width_ && y >= 0 && y < grid_height_;
}

float GridTriangulator::gridToWorldX(int grid_x) const {
    if (grid_width_ <= 1) return min_x_;
    return min_x_ + (static_cast<float>(grid_x) / (grid_width_ - 1)) * (max_x_ - min_x_);
}

float GridTriangulator::gridToWorldY(int grid_y) const {
    if (grid_height_ <= 1) return min_y_;
    return min_y_ + (static_cast<float>(grid_y) / (grid_height_ - 1)) * (max_y_ - min_y_);
}

std::pair<int, int> GridTriangulator::worldToGrid(float x, float y) const {
    int grid_x = static_cast<int>((x - min_x_) / (max_x_ - min_x_) * (grid_width_ - 1) + 0.5f);
    int grid_y = static_cast<int>((y - min_y_) / (max_y_ - min_y_) * (grid_height_ - 1) + 0.5f);
    
    grid_x = std::max(0, std::min(grid_width_ - 1, grid_x));
    grid_y = std::max(0, std::min(grid_height_ - 1, grid_y));
    
    return {grid_x, grid_y};
}

float GridTriangulator::distanceSquared(const GridVertex& a, const GridVertex& b) const {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return dx * dx + dy * dy;
}

float GridTriangulator::triangleArea(const GridVertex& a, const GridVertex& b, const GridVertex& c) const {
    return 0.5f * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
}

bool GridTriangulator::isLeftTurn(const GridVertex& a, const GridVertex& b, const GridVertex& c) const {
    return triangleArea(a, b, c) > 0.0f;
}

uint64_t GridTriangulator::encodeEdge(uint32_t v0, uint32_t v1) const {
    // Ensure consistent edge encoding (smaller index first)
    if (v0 > v1) std::swap(v0, v1);
    return (static_cast<uint64_t>(v0) << 32) | static_cast<uint64_t>(v1);
}

bool GridTriangulator::isEdgeProcessed(uint32_t v0, uint32_t v1) const {
    return processed_edges_.count(encodeEdge(v0, v1)) > 0;
}

void GridTriangulator::markEdgeProcessed(uint32_t v0, uint32_t v1) {
    processed_edges_.insert(encodeEdge(v0, v1));
}

bool GridTriangulator::canSwapEdge(uint32_t tri1, uint32_t tri2, uint32_t shared_v0, uint32_t shared_v1) {
    // Simplified edge swapping check
    // In full implementation, would check geometric validity and quality improvement
    return false; // Disable for now
}

void GridTriangulator::swapEdge(uint32_t tri1, uint32_t tri2, uint32_t shared_v0, uint32_t shared_v1) {
    // Simplified edge swapping implementation
    // In full implementation, would perform actual edge swap
}

std::vector<uint32_t> GridTriangulator::findTrianglesUsingEdge(uint32_t v0, uint32_t v1) {
    std::vector<uint32_t> result;
    
    for (size_t i = 0; i < triangles_.size(); ++i) {
        const auto& tri = triangles_[i];
        std::vector<uint32_t> tri_verts = {tri.v0, tri.v1, tri.v2};
        
        bool has_v0 = std::find(tri_verts.begin(), tri_verts.end(), v0) != tri_verts.end();
        bool has_v1 = std::find(tri_verts.begin(), tri_verts.end(), v1) != tri_verts.end();
        
        if (has_v0 && has_v1) {
            result.push_back(static_cast<uint32_t>(i));
        }
    }
    
    return result;
}

} // namespace TerraScape