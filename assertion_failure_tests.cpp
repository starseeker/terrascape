#include "TerraScape.hpp"
#include "TerraScapeImpl.h" 
#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <limits>
#include <random>

using namespace TerraScape;

// Test case designed to trigger potential detria assertion failures
void test_potential_assertion_triggers() {
    std::cout << "Testing potential detria assertion triggers..." << std::endl;
    
    // Test 1: Perfectly collinear points (all on same line)
    std::cout << "  Test 1a: Collinear points..." << std::endl;
    {
        const int width = 5, height = 1;
        std::vector<float> elevations = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
        assert(result.vertices.size() > 0);
        std::cout << "    ✓ Collinear points handled" << std::endl;
    }
    
    // Test 2: Duplicate/coincident points
    std::cout << "  Test 1b: Duplicate coordinates..." << std::endl;
    {
        const int width = 3, height = 3;
        std::vector<float> elevations;
        for (int i = 0; i < 9; i++) {
            elevations.push_back(5.0f); // All same height at different x,y
        }
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
        assert(result.vertices.size() > 0);
        std::cout << "    ✓ Duplicate coordinates handled" << std::endl;
    }
    
    // Test 3: Extremely close but not identical points
    std::cout << "  Test 1c: Nearly coincident points..." << std::endl;
    {
        const int width = 3, height = 3;
        std::vector<float> elevations;
        for (int i = 0; i < 9; i++) {
            elevations.push_back(5.0f + i * 1e-15f); // Tiny differences
        }
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
        assert(result.vertices.size() > 0);
        std::cout << "    ✓ Nearly coincident points handled" << std::endl;
    }
    
    std::cout << "  ✓ All potential assertion triggers handled correctly" << std::endl;
}

// Test edge cases that could cause numerical instability
void test_numerical_stability() {
    std::cout << "Testing numerical stability edge cases..." << std::endl;
    
    // Test 1: Very large coordinates
    std::cout << "  Test 2a: Very large coordinates..." << std::endl;
    {
        const int width = 3, height = 3;
        std::vector<double> elevations;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                elevations.push_back(1e12 + x + y); // Very large values
            }
        }
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
        assert(result.vertices.size() > 0);
        std::cout << "    ✓ Very large coordinates handled" << std::endl;
    }
    
    // Test 2: Very small coordinates  
    std::cout << "  Test 2b: Very small coordinates..." << std::endl;
    {
        const int width = 3, height = 3;
        std::vector<double> elevations;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                elevations.push_back(1e-12 + x * 1e-13 + y * 1e-13); // Very small values
            }
        }
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
        assert(result.vertices.size() > 0);
        std::cout << "    ✓ Very small coordinates handled" << std::endl;
    }
    
    // Test 3: Mixed large and small coordinates
    std::cout << "  Test 2c: Mixed scale coordinates..." << std::endl;
    {
        const int width = 3, height = 3;
        std::vector<float> elevations = {
            1e10f, 1e-10f, 1e5f,
            1e-5f, 0.0f, 1e8f,
            -1e9f, 1e-8f, 1e6f
        };
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
        assert(result.vertices.size() > 0);
        std::cout << "    ✓ Mixed scale coordinates handled" << std::endl;
    }
    
    std::cout << "  ✓ All numerical stability tests passed" << std::endl;
}

// Test extreme error thresholds that could cause issues
void test_extreme_error_thresholds() {
    std::cout << "Testing extreme error thresholds..." << std::endl;
    
    const int width = 4, height = 4;
    std::vector<float> elevations;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            elevations.push_back(static_cast<float>(x + y));
        }
    }
    
    // Test negative error threshold
    std::cout << "  Test 3a: Negative error threshold..." << std::endl;
    {
        MeshResult result = grid_to_mesh(width, height, elevations.data(), -1.0f, 1000);
        assert(result.vertices.size() > 0);
        std::cout << "    ✓ Negative error threshold handled" << std::endl;
    }
    
    // Test zero error threshold
    std::cout << "  Test 3b: Zero error threshold..." << std::endl;
    {
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 0.0f, 1000);
        assert(result.vertices.size() > 0);
        std::cout << "    ✓ Zero error threshold handled" << std::endl;
    }
    
    // Test extremely small error threshold
    std::cout << "  Test 3c: Extremely small error threshold..." << std::endl;
    {
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 1e-20f, 1000);
        assert(result.vertices.size() > 0);
        std::cout << "    ✓ Extremely small error threshold handled" << std::endl;
    }
    
    std::cout << "  ✓ All extreme error threshold tests passed" << std::endl;
}

// Test real-world-like problematic data patterns
void test_realworld_problematic_patterns() {
    std::cout << "Testing real-world problematic data patterns..." << std::endl;
    
    // Test 1: Data with "holes" (invalid regions)
    std::cout << "  Test 4a: Data with invalid regions..." << std::endl;
    {
        const int width = 5, height = 5;
        std::vector<float> elevations;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if ((x == 2 && y == 2) || (x == 1 && y == 3)) {
                    elevations.push_back(std::numeric_limits<float>::quiet_NaN()); // Holes
                } else {
                    elevations.push_back(static_cast<float>(x + y));
                }
            }
        }
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
        assert(result.vertices.size() > 0);
        std::cout << "    ✓ Data with invalid regions handled" << std::endl;
    }
    
    // Test 2: Noisy flat data (mostly flat with tiny variations)
    std::cout << "  Test 4b: Noisy flat data..." << std::endl;
    {
        const int width = 6, height = 6;
        std::vector<float> elevations;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> noise(-1e-12f, 1e-12f);
        
        for (int i = 0; i < width * height; ++i) {
            elevations.push_back(100.0f + noise(gen)); // Mostly flat with tiny noise
        }
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
        assert(result.vertices.size() > 0);
        std::cout << "    ✓ Noisy flat data handled" << std::endl;
    }
    
    // Test 3: Stepped terrain (sudden elevation changes)
    std::cout << "  Test 4c: Stepped terrain..." << std::endl;
    {
        const int width = 4, height = 4;
        std::vector<float> elevations;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // Create sharp steps
                float z = (x < width/2) ? 0.0f : 1000.0f;
                elevations.push_back(z);
            }
        }
        
        MeshResult result = grid_to_mesh(width, height, elevations.data(), 1.0f, 1000);
        assert(result.vertices.size() > 0);
        std::cout << "    ✓ Stepped terrain handled" << std::endl;
    }
    
    std::cout << "  ✓ All real-world problematic pattern tests passed" << std::endl;
}

// Main stress test driver
int main() {
    std::cout << "=== TerraScape Detria Assertion Failure Prevention Tests ===" << std::endl;
    std::cout << "Testing preprocessing to prevent assertion failures in detria triangulation" << std::endl;
    std::cout << std::endl;
    
    try {
        test_potential_assertion_triggers();
        test_numerical_stability();
        test_extreme_error_thresholds();
        test_realworld_problematic_patterns();
        
        std::cout << std::endl;
        std::cout << "=== All Assertion Failure Prevention Tests Passed! ===" << std::endl;
        std::cout << "✓ TerraScape preprocessing successfully prevents detria assertion failures" << std::endl;
        std::cout << "✓ Real-world problematic data patterns are handled robustly" << std::endl;
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        return 1;
    }
}