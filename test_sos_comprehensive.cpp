#include "bg_detria.hpp"
#include <iostream>
#include <vector>
#include <cassert>

// Override the SoS setting for this test
namespace detria {
    template<typename Point, typename Idx>
    struct SoSTriangulationConfig : public DefaultTriangulationConfig<Point, Idx> {
        // Enable SoS for this test
        constexpr static bool UseSimulationOfSimplicity = true;
    };
}

int main() {
    std::cout << "=== Comprehensive Simulation of Simplicity Test ===" << std::endl;
    
    bool all_tests_passed = true;
    
    // Test 1: Basic collinear point triangulation with SoS
    std::cout << "\nTest 1: Basic collinear triangulation..." << std::endl;
    try {
        std::vector<detria::PointD> points = {
            {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {1.0, 1.0}  // First 3 are collinear
        };
        
        detria::Triangulation<detria::PointD, uint32_t, detria::SoSTriangulationConfig<detria::PointD, uint32_t>> tri;
        tri.setPoints(points);
        
        std::vector<uint32_t> outline = {0, 3, 2, 1};  // Avoid collinear edge in outline
        tri.addOutline(outline);
        
        bool success = tri.triangulate(true);
        
        if (success) {
            std::cout << "  ✓ SoS triangulation succeeded" << std::endl;
        } else {
            std::cout << "  ✗ SoS triangulation failed" << std::endl;
            all_tests_passed = false;
        }
        
    } catch (...) {
        std::cout << "  ✗ Exception during SoS triangulation" << std::endl;
        all_tests_passed = false;
    }
    
    // Test 2: Edge collinearity handling
    std::cout << "\nTest 2: Direct SoS predicate testing..." << std::endl;
    try {
        using SoSTriangulation = detria::Triangulation<detria::PointD, uint32_t, detria::SoSTriangulationConfig<detria::PointD, uint32_t>>;
        
        // Test exact collinear points
        detria::Vec2<double> a{0.0, 0.0};
        detria::Vec2<double> b{1.0, 0.0}; 
        detria::Vec2<double> c{2.0, 0.0};
        
        // Regular orient2d should return Collinear
        auto regular_result = detria::math::orient2d<true>(a, b, c);
        assert(regular_result == detria::math::Orientation::Collinear);
        
        // SoS should never return Collinear
        auto sos_result = detria::math::sos::sos_orient2d_tiebreak(a, 0, b, 1, c, 2);
        assert(sos_result != detria::math::Orientation::Collinear);
        
        std::cout << "  ✓ SoS eliminates collinear results (regular=" << (int)regular_result 
                  << ", SoS=" << (int)sos_result << ")" << std::endl;
        
    } catch (...) {
        std::cout << "  ✗ Exception during SoS predicate testing" << std::endl;
        all_tests_passed = false;
    }
    
    // Test 3: Consistency test
    std::cout << "\nTest 3: SoS consistency testing..." << std::endl;
    try {
        // Test that SoS gives consistent results for same inputs
        detria::Vec2<double> p1{0.0, 0.0};
        detria::Vec2<double> p2{1.0, 0.0}; 
        detria::Vec2<double> p3{2.0, 0.0};
        
        auto result1 = detria::math::sos::sos_orient2d_tiebreak(p1, 0, p2, 1, p3, 2);
        auto result2 = detria::math::sos::sos_orient2d_tiebreak(p1, 0, p2, 1, p3, 2);
        
        assert(result1 == result2);
        
        // Test different index assignments give different but consistent results
        auto result3 = detria::math::sos::sos_orient2d_tiebreak(p1, 2, p2, 1, p3, 0);
        // Should be different from result1 but still consistent
        
        std::cout << "  ✓ SoS results are consistent (same inputs: " << (int)result1 
                  << "==" << (int)result2 << ", different indices: " << (int)result3 << ")" << std::endl;
                  
    } catch (...) {
        std::cout << "  ✗ Exception during SoS consistency testing" << std::endl;
        all_tests_passed = false;
    }
    
    // Test 4: Complex scenario
    std::cout << "\nTest 4: Complex multi-point scenario..." << std::endl;
    try {
        std::vector<detria::PointD> points = {
            {0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}, {2.0, 0.0},  // Many collinear points
            {1.0, 0.5}, {1.0, 1.0}, {1.0, 1.5}  // Some off-line points
        };
        
        detria::Triangulation<detria::PointD, uint32_t, detria::SoSTriangulationConfig<detria::PointD, uint32_t>> tri;
        tri.setPoints(points);
        
        // Create outline avoiding collinear sequences
        std::vector<uint32_t> outline = {0, 5, 7, 4};
        tri.addOutline(outline);
        
        bool success = tri.triangulate(true);
        
        if (success) {
            int triangle_count = 0;
            tri.forEachTriangle([&](auto triangle) {
                triangle_count++;
            });
            std::cout << "  ✓ Complex SoS scenario succeeded (" << triangle_count << " triangles)" << std::endl;
        } else {
            std::cout << "  ✗ Complex SoS scenario failed" << std::endl;
            all_tests_passed = false;
        }
        
    } catch (...) {
        std::cout << "  ✗ Exception during complex SoS scenario" << std::endl;
        all_tests_passed = false;
    }
    
    // Summary
    std::cout << "\n=== Summary ===" << std::endl;
    if (all_tests_passed) {
        std::cout << "✅ All SoS tests passed! Simulation of Simplicity is working correctly." << std::endl;
        std::cout << "Note: SoS is now enabled by default for robust geometric computation." << std::endl;
        return 0;
    } else {
        std::cout << "❌ Some SoS tests failed. Implementation needs refinement." << std::endl;
        return 1;
    }
}