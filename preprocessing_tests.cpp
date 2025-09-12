#include "TerraScape.hpp"
#include "TerraScapeImpl.h"
#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <limits>

using namespace TerraScape;

// Test case: Completely flat grid (all elevations identical)
void test_flat_grid() {
    std::cout << "Testing flat grid preprocessing..." << std::endl;
    
    const int width = 4, height = 4;
    std::vector<float> flat_elevations(width * height, 100.0f);
    
    MeshResult result = grid_to_mesh(width, height, flat_elevations.data(), 1.0f, 1000);
    
    // Should produce a simple mesh without crashing
    assert(result.vertices.size() >= 4);
    assert(result.triangles.size() >= 2);
    
    std::cout << "  ✓ Flat grid handled correctly" << std::endl;
}

// Test case: Grid with zero error threshold
void test_zero_error_threshold() {
    std::cout << "Testing zero error threshold..." << std::endl;
    
    const int width = 3, height = 3;
    std::vector<float> elevations;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            elevations.push_back(static_cast<float>(x + y));
        }
    }
    
    // Use zero error threshold - should be clamped internally
    MeshResult result = grid_to_mesh(width, height, elevations.data(), 0.0f, 1000);
    
    // Should complete without crashing
    assert(result.vertices.size() > 0);
    assert(result.triangles.size() > 0);
    
    std::cout << "  ✓ Zero error threshold handled correctly" << std::endl;
}

// Test case: Grid with invalid values (NaN, infinity)
void test_invalid_values() {
    std::cout << "Testing invalid values (NaN, infinity)..." << std::endl;
    
    const int width = 3, height = 3;
    std::vector<float> elevations = {
        1.0f, 2.0f, 3.0f,
        4.0f, std::numeric_limits<float>::quiet_NaN(), 6.0f,
        7.0f, 8.0f, std::numeric_limits<float>::infinity()
    };
    
    MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
    
    // Should complete without crashing and replace invalid values
    assert(result.vertices.size() > 0);
    assert(result.triangles.size() > 0);
    
    std::cout << "  ✓ Invalid values handled correctly" << std::endl;
}

// Test case: Badly scaled coordinates (very large values)
void test_large_coordinates() {
    std::cout << "Testing large coordinate values..." << std::endl;
    
    const int width = 3, height = 3;
    std::vector<float> elevations;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            elevations.push_back(1e7f + static_cast<float>(x + y) * 1e6f);
        }
    }
    
    MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
    
    // Should complete without crashing
    assert(result.vertices.size() > 0);
    assert(result.triangles.size() > 0);
    
    std::cout << "  ✓ Large coordinate values handled correctly" << std::endl;
}

// Test case: Very small coordinate values
void test_small_coordinates() {
    std::cout << "Testing small coordinate values..." << std::endl;
    
    const int width = 3, height = 3;
    std::vector<float> elevations;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            elevations.push_back(1e-6f + static_cast<float>(x + y) * 1e-7f);
        }
    }
    
    MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
    
    // Should complete without crashing
    assert(result.vertices.size() > 0);
    assert(result.triangles.size() > 0);
    
    std::cout << "  ✓ Small coordinate values handled correctly" << std::endl;
}

// Test case: Coplanar grid (all points on same plane but not flat)
void test_coplanar_grid() {
    std::cout << "Testing coplanar grid..." << std::endl;
    
    const int width = 4, height = 4;
    std::vector<float> elevations;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            elevations.push_back(static_cast<float>(x + y)); // Linear plane
        }
    }
    
    MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
    
    // Should complete without crashing
    assert(result.vertices.size() > 0);
    assert(result.triangles.size() > 0);
    
    std::cout << "  ✓ Coplanar grid handled correctly" << std::endl;
}

// Test case: Mixed data types
void test_integer_input() {
    std::cout << "Testing integer input conversion..." << std::endl;
    
    const int width = 3, height = 3;
    std::vector<int> elevations;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            elevations.push_back(x + y * 10);
        }
    }
    
    MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
    
    // Should complete without crashing
    assert(result.vertices.size() > 0);
    assert(result.triangles.size() > 0);
    
    std::cout << "  ✓ Integer input conversion handled correctly" << std::endl;
}

// Test case: Edge case with single row/column
void test_degenerate_dimensions() {
    std::cout << "Testing degenerate dimensions..." << std::endl;
    
    // Test 1x4 grid (single row)
    const int width1 = 4, height1 = 1;
    std::vector<float> elevations1 = {1.0f, 2.0f, 3.0f, 4.0f};
    
    MeshResult result1 = grid_to_mesh(width1, height1, elevations1.data(), 1.0f, 1000);
    assert(result1.vertices.size() > 0);
    
    // Test 1x1 grid
    const int width2 = 1, height2 = 1;
    std::vector<float> elevations2 = {1.0f};
    
    MeshResult result2 = grid_to_mesh(width2, height2, elevations2.data(), 1.0f, 1000);
    assert(result2.vertices.size() > 0);
    
    std::cout << "  ✓ Degenerate dimensions handled correctly" << std::endl;
}

// Main test driver
int main() {
    std::cout << "=== TerraScape Input Preprocessing Tests ===" << std::endl;
    std::cout << "Testing robustness improvements for degenerate input data" << std::endl;
    std::cout << std::endl;
    
    try {
        test_flat_grid();
        test_zero_error_threshold();
        test_invalid_values();
        test_large_coordinates();
        test_small_coordinates();
        test_coplanar_grid();
        test_integer_input();
        test_degenerate_dimensions();
        
        std::cout << std::endl;
        std::cout << "=== All Preprocessing Tests Passed! ===" << std::endl;
        std::cout << "✓ TerraScape successfully handles degenerate input data" << std::endl;
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        return 1;
    }
}