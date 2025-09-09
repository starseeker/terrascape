#include "bg_detria.hpp"
#include <iostream>
#include <cassert>

void test_lexicographic_less_2d() {
    std::cout << "Testing lexicographic_less_2d..." << std::endl;
    
    // Test basic ordering
    assert(detria::math::sos::lexicographic_less_2d(0.0, 0.0, 0, 1.0, 0.0, 1) == true);  // (0,0,0) < (1,0,1)
    assert(detria::math::sos::lexicographic_less_2d(1.0, 0.0, 1, 0.0, 0.0, 0) == false); // (1,0,1) > (0,0,0)
    
    // Test tie-breaking by index when positions are same
    assert(detria::math::sos::lexicographic_less_2d(1.0, 1.0, 0, 1.0, 1.0, 1) == true);  // same pos, index 0 < 1
    assert(detria::math::sos::lexicographic_less_2d(1.0, 1.0, 1, 1.0, 1.0, 0) == false); // same pos, index 1 > 0
    
    std::cout << "✓ lexicographic_less_2d tests passed" << std::endl;
}

void test_sos_orient2d_tiebreak() {
    std::cout << "Testing sos_orient2d_tiebreak..." << std::endl;
    
    // Test collinear points (0,0), (1,0), (2,0) with indices 0, 1, 2
    detria::PointD a{0.0, 0.0};
    detria::PointD b{1.0, 0.0}; 
    detria::PointD c{2.0, 0.0}; // Exactly collinear
    
    auto result = detria::math::sos::sos_orient2d_tiebreak(a, 0, b, 1, c, 2);
    std::cout << "SoS result for (0,0,0), (1,0,1), (2,0,2): " << (int)result << std::endl;
    
    // The result should be deterministic and not Collinear
    assert(result != detria::math::Orientation::Collinear);
    
    // Test with different index orderings - should give consistent results
    auto result2 = detria::math::sos::sos_orient2d_tiebreak(a, 2, b, 1, c, 0);  // Different indices
    std::cout << "SoS result for (0,0,2), (1,0,1), (2,0,0): " << (int)result2 << std::endl;
    
    std::cout << "✓ sos_orient2d_tiebreak tests passed" << std::endl;
}

void test_regular_vs_sos() {
    std::cout << "Testing regular vs SoS orient2d..." << std::endl;
    
    // Create exactly collinear points
    detria::PointD a{0.0, 0.0};
    detria::PointD b{1.0, 0.0}; 
    detria::PointD c{2.0, 0.0};
    
    // Regular orient2d should return Collinear
    auto regular = detria::math::orient2d<true>(a, b, c);
    std::cout << "Regular orient2d: " << (int)regular << " (should be 2 = Collinear)" << std::endl;
    assert(regular == detria::math::Orientation::Collinear);
    
    // SoS should return something definite (not Collinear)
    auto sos = detria::math::sos::sos_orient2d_tiebreak(a, 0, b, 1, c, 2);
    std::cout << "SoS orient2d: " << (int)sos << " (should be 0 or 1, not 2)" << std::endl;
    assert(sos != detria::math::Orientation::Collinear);
    
    std::cout << "✓ Regular vs SoS tests passed" << std::endl;
}

int main() {
    std::cout << "=== SoS Implementation Tests ===" << std::endl;
    
    try {
        test_lexicographic_less_2d();
        test_sos_orient2d_tiebreak();
        test_regular_vs_sos();
        
        std::cout << "✅ All SoS tests passed!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cout << "❌ Test failed: " << e.what() << std::endl;
        return 1;
    }
}