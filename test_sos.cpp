#include "bg_detria.hpp"
#include <iostream>

int main() {
    std::cout << "Testing Simulation of Simplicity..." << std::endl;
    
    // Create three collinear points
    detria::PointD a{0.0, 0.0};
    detria::PointD b{1.0, 0.0}; 
    detria::PointD c{2.0, 0.0}; // Exactly collinear
    
    using TriangulationType = detria::Triangulation<detria::PointD, uint32_t>;
    
    // Test regular orient2d - should return Collinear
    auto regular_result = detria::math::orient2d<true>(a, b, c);
    std::cout << "Regular orient2d result: " << (int)regular_result << " (0=CW, 1=CCW, 2=Collinear)" << std::endl;
    
    // Test SoS orient2d - should never return Collinear (2)
    auto sos_result = TriangulationType::orient2d_sos(
        {a.x, a.y}, 0,  // point a with index 0
        {b.x, b.y}, 1,  // point b with index 1  
        {c.x, c.y}, 2   // point c with index 2
    );
    std::cout << "SoS orient2d result: " << (int)sos_result << " (0=CW, 1=CCW, 2=Collinear)" << std::endl;
    
    // Check if SoS is enabled
    std::cout << "UseSimulationOfSimplicity: " << TriangulationType::Config::UseSimulationOfSimplicity << std::endl;
    
    if (sos_result == detria::math::Orientation::Collinear) {
        std::cout << "ERROR: SoS should never return Collinear!" << std::endl;
        return 1;
    } else {
        std::cout << "SUCCESS: SoS returned a definite orientation" << std::endl;
        return 0;
    }
}