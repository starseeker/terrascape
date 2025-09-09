#include "bg_detria.hpp"
#include <iostream>

// Configuration with SoS enabled
template<typename Point, typename Idx>
struct SoSTriangulationConfig : public detria::DefaultTriangulationConfig<Point, Idx> {
    constexpr static bool UseSimulationOfSimplicity = true;
};

int main() {
    std::cout << "=== Debug SoS Crash Test ===" << std::endl;
    
    try {
        // Simpler test case to isolate the crash
        std::vector<detria::PointD> points = {
            {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {1.0, 1.0}  // Three collinear + one off
        };
        
        std::cout << "Creating triangulation..." << std::endl;
        detria::Triangulation<detria::PointD, uint32_t, SoSTriangulationConfig<detria::PointD, uint32_t>> tri;
        
        std::cout << "Setting points..." << std::endl;
        tri.setPoints(points);
        
        std::cout << "Adding outline..." << std::endl;
        std::vector<uint32_t> outline = {0, 1, 3, 2};  // Simplified outline
        tri.addOutline(outline);
        
        std::cout << "Starting triangulation..." << std::endl;
        bool success = tri.triangulate(true);
        
        if (success) {
            std::cout << "  ✓ Simple case succeeded" << std::endl;
        } else {
            std::cout << "  ✗ Simple case failed" << std::endl;
        }
        
        std::cout << "Now trying the complex case..." << std::endl;
        
        // Now try the complex case
        std::vector<detria::PointD> complex_points = {
            {0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}, {2.0, 0.0},  // Many collinear points
            {1.0, 0.5}, {1.0, 1.0}, {1.0, 1.5}  // Some off-line points
        };
        
        std::cout << "Creating complex triangulation..." << std::endl;
        detria::Triangulation<detria::PointD, uint32_t, SoSTriangulationConfig<detria::PointD, uint32_t>> tri2;
        
        std::cout << "Setting complex points..." << std::endl;
        tri2.setPoints(complex_points);
        
        std::cout << "Adding complex outline..." << std::endl;
        std::vector<uint32_t> complex_outline = {0, 5, 7, 4};
        tri2.addOutline(complex_outline);
        
        std::cout << "Starting complex triangulation..." << std::endl;
        bool complex_success = tri2.triangulate(true);
        
        if (complex_success) {
            std::cout << "  ✓ Complex case succeeded" << std::endl;
        } else {
            std::cout << "  ✗ Complex case failed" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "Exception caught: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cout << "Unknown exception caught" << std::endl;
        return 1;
    }
    
    return 0;
}