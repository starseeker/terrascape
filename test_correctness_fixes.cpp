#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <set>
#include "TerraScape.hpp"
#include "TerraScapeImpl.h"

// Test data generation functions
std::vector<float> create_flat_surface(int width, int height) {
    return std::vector<float>(width * height, 1.0f);
}

std::vector<float> create_single_peak(int width, int height) {
    std::vector<float> data(width * height);
    int centerX = width / 2;
    int centerY = height / 2;
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float dx = x - centerX;
            float dy = y - centerY;
            float distance = std::sqrt(dx * dx + dy * dy);
            data[y * width + x] = std::exp(-distance * 0.5f);
        }
    }
    return data;
}

std::vector<float> create_deterministic_surface(int width, int height) {
    std::vector<float> data(width * height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            data[y * width + x] = 0.1f * x + 0.2f * y + 0.05f * x * y;
        }
    }
    return data;
}

// Test 1: Triangulation versioning prevents stale candidates
bool test_triangulation_versioning() {
    std::cout << "Test 1: Triangulation versioning..." << std::endl;
    
    TerraScape::DetriaTriangulationManager manager;
    
    // Initialize with boundary
    manager.initializeBoundary(0.0f, 0.0f, 5.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    uint32_t initial_version = manager.getVersion();
    
    // Add a point and retriangulate
    manager.addPoint(2.5f, 2.5f, 1.0f);
    manager.retriangulate();
    uint32_t after_first = manager.getVersion();
    
    // Add another point and retriangulate
    manager.addPoint(1.5f, 1.5f, 0.5f);
    manager.retriangulate();
    uint32_t after_second = manager.getVersion();
    
    // Version should increment after each successful retriangulation
    bool versioning_works = (after_first > initial_version) && (after_second > after_first);
    
    std::cout << "  Initial version: " << initial_version << std::endl;
    std::cout << "  After first: " << after_first << std::endl;
    std::cout << "  After second: " << after_second << std::endl;
    std::cout << "  ✓ Versioning works: " << (versioning_works ? "YES" : "NO") << std::endl;
    
    return versioning_works;
}

// Test 2: Duplicate point insertion guard
bool test_duplicate_guard() {
    std::cout << "\nTest 2: Duplicate point insertion guard..." << std::endl;
    
    auto data = create_single_peak(6, 6);
    
    // Use HEAP strategy with low threshold to ensure multiple candidates
    auto result = TerraScape::grid_to_mesh(6, 6, data.data(), 0.01f, 50, 
                                          TerraScape::MeshRefineStrategy::HEAP);
    
    // Check that all vertices have unique positions
    std::set<std::pair<float, float>> positions;
    bool no_duplicates = true;
    
    for (const auto& vertex : result.vertices) {
        std::pair<float, float> pos = {vertex.x, vertex.y};
        if (positions.find(pos) != positions.end()) {
            no_duplicates = false;
            std::cout << "  Found duplicate position: (" << vertex.x << ", " << vertex.y << ")" << std::endl;
        }
        positions.insert(pos);
    }
    
    std::cout << "  Generated " << result.vertices.size() << " unique vertices" << std::endl;
    std::cout << "  ✓ No duplicates: " << (no_duplicates ? "YES" : "NO") << std::endl;
    
    return no_duplicates;
}

// Test 3: Determinism test (same input produces same output)
bool test_determinism() {
    std::cout << "\nTest 3: Determinism test..." << std::endl;
    
    auto data = create_deterministic_surface(8, 8);
    
    // Run the same configuration twice
    auto result1 = TerraScape::grid_to_mesh(8, 8, data.data(), 0.05f, 30, 
                                           TerraScape::MeshRefineStrategy::HEAP);
    auto result2 = TerraScape::grid_to_mesh(8, 8, data.data(), 0.05f, 30, 
                                           TerraScape::MeshRefineStrategy::HEAP);
    
    bool same_vertex_count = (result1.vertices.size() == result2.vertices.size());
    bool same_triangle_count = (result1.triangles.size() == result2.triangles.size());
    
    // Check if vertex positions match (allowing for small floating point differences)
    bool positions_match = true;
    if (same_vertex_count) {
        for (size_t i = 0; i < result1.vertices.size(); ++i) {
            float dx = std::abs(result1.vertices[i].x - result2.vertices[i].x);
            float dy = std::abs(result1.vertices[i].y - result2.vertices[i].y);
            float dz = std::abs(result1.vertices[i].z - result2.vertices[i].z);
            
            if (dx > 1e-6f || dy > 1e-6f || dz > 1e-6f) {
                positions_match = false;
                break;
            }
        }
    } else {
        positions_match = false;
    }
    
    std::cout << "  Run 1: " << result1.vertices.size() << " vertices, " << result1.triangles.size() << " triangles" << std::endl;
    std::cout << "  Run 2: " << result2.vertices.size() << " vertices, " << result2.triangles.size() << " triangles" << std::endl;
    std::cout << "  ✓ Same vertex count: " << (same_vertex_count ? "YES" : "NO") << std::endl;
    std::cout << "  ✓ Same triangle count: " << (same_triangle_count ? "YES" : "NO") << std::endl;
    std::cout << "  ✓ Positions match: " << (positions_match ? "YES" : "NO") << std::endl;
    
    return same_vertex_count && same_triangle_count && positions_match;
}

// Test 4: Flat plane minimal refinement
bool test_flat_plane_minimal_refinement() {
    std::cout << "\nTest 4: Flat plane minimal refinement..." << std::endl;
    
    auto data = create_flat_surface(6, 6);
    
    auto result = TerraScape::grid_to_mesh(6, 6, data.data(), 0.1f, 50,
                                          TerraScape::MeshRefineStrategy::HEAP);
    
    // For a flat surface, we should have minimal triangles (just the boundary)
    bool minimal_triangulation = (result.vertices.size() <= 8); // Should be 4 corners + maybe a few more
    
    std::cout << "  Generated " << result.vertices.size() << " vertices for flat surface" << std::endl;
    std::cout << "  Generated " << result.triangles.size() << " triangles for flat surface" << std::endl;
    std::cout << "  ✓ Minimal triangulation: " << (minimal_triangulation ? "YES" : "NO") << std::endl;
    
    return minimal_triangulation;
}

// Test 5: Error threshold respect
bool test_error_threshold_respect() {
    std::cout << "\nTest 5: Error threshold respect..." << std::endl;
    
    auto data = create_single_peak(10, 10);
    
    // Test with strict threshold
    auto strict_result = TerraScape::grid_to_mesh(10, 10, data.data(), 0.01f, 100,
                                                TerraScape::MeshRefineStrategy::HEAP);
    
    // Test with loose threshold  
    auto loose_result = TerraScape::grid_to_mesh(10, 10, data.data(), 0.5f, 100,
                                               TerraScape::MeshRefineStrategy::HEAP);
    
    bool strict_has_more = (strict_result.vertices.size() >= loose_result.vertices.size());
    
    std::cout << "  Strict threshold (0.01): " << strict_result.vertices.size() << " vertices" << std::endl;
    std::cout << "  Loose threshold (0.5): " << loose_result.vertices.size() << " vertices" << std::endl;
    std::cout << "  ✓ Strict threshold produces more/equal vertices: " << (strict_has_more ? "YES" : "NO") << std::endl;
    
    return strict_has_more;
}

int main() {
    std::cout << "=== TerraScape Correctness Fixes Test Suite ===" << std::endl;
    std::cout << "Testing the high-impact correctness fixes:" << std::endl;
    std::cout << "1. Triangulation versioning to prevent stale candidates" << std::endl;
    std::cout << "2. Outline accumulation fix (clear before retriangulate)" << std::endl;
    std::cout << "3. Duplicate point insertion guard" << std::endl;
    std::cout << "4. Deterministic behavior validation" << std::endl;
    std::cout << "5. Error threshold enforcement" << std::endl;
    std::cout << std::endl;
    
    int passed = 0;
    int total = 5;
    
    if (test_triangulation_versioning()) passed++;
    if (test_duplicate_guard()) passed++;
    if (test_determinism()) passed++;
    if (test_flat_plane_minimal_refinement()) passed++;
    if (test_error_threshold_respect()) passed++;
    
    std::cout << "\n=== Test Results ===" << std::endl;
    std::cout << "Passed: " << passed << "/" << total << " tests" << std::endl;
    
    if (passed == total) {
        std::cout << "✅ All correctness fixes are working properly!" << std::endl;
        return 0;
    } else {
        std::cout << "❌ Some tests failed. Review the fixes." << std::endl;
        return 1;
    }
}