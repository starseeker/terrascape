#include "bg_detria.hpp"
#include <iostream>
#include <vector>

// Test SoS in the context of actual detria triangulation
int main() {
    std::cout << "=== Direct Detria SoS Test ===" << std::endl;
    
    try {
        // Create a simple triangulation that might trigger SoS
        std::vector<detria::PointD> points = {
            {0.0, 0.0},   // 0
            {1.0, 0.0},   // 1  
            {2.0, 0.0},   // 2 - this makes 0,1,2 collinear!
            {0.5, 1.0}    // 3
        };
        
        detria::Triangulation<detria::PointD, uint32_t> tri;
        tri.setPoints(points);
        
        // Add an outline that includes the collinear points
        std::vector<uint32_t> outline = {0, 1, 2, 3};
        tri.addOutline(outline);
        
        std::cout << "Attempting triangulation with collinear points..." << std::endl;
        bool success = tri.triangulate(true);  // Delaunay = true
        
        if (success) {
            std::cout << "SUCCESS: Triangulation completed" << std::endl;
            
            // Count triangles
            int triangle_count = 0;
            tri.forEachTriangle([&](auto triangle) {
                triangle_count++;
                std::cout << "Triangle: " << triangle.x << "," << triangle.y << "," << triangle.z << std::endl;
            });
            
            std::cout << "Generated " << triangle_count << " triangles" << std::endl;
        } else {
            std::cout << "FAILED: Triangulation failed" << std::endl;
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cout << "EXCEPTION: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cout << "CRASH: Unknown exception" << std::endl;
        return 1;
    }
    
    return 0;
}