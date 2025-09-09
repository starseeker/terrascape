#pragma once

#include <string>
#include <vector>
#include <iostream>

namespace TerrainDataUtils {
    
    // Structure to hold terrain data info
    struct TerrainInfo {
        int width = 0;
        int height = 0;
        double min_elevation = 0.0;
        double max_elevation = 0.0;
        std::string projection;
        double pixel_size_x = 0.0;
        double pixel_size_y = 0.0;
    };
    
    // Convert BIL file to PGM format using GDAL (when available)
    bool convertBilToPgm(const std::string& bil_file, const std::string& pgm_file, TerrainInfo* info = nullptr);
    
    // Download terrain data from Hawaii dataset (when GDAL available)
    bool downloadHawaiiTerrainData(const std::string& output_dir);
    
    // Validate mesh against original terrain data (when GDAL available)
    bool validateMeshAgainstTerrain(const std::string& terrain_file, 
                                   const std::vector<float>& mesh_vertices,
                                   const std::vector<int>& mesh_triangles,
                                   double tolerance = 1.0);
    
    // Check if GDAL support is available
    bool isGdalAvailable();
    
    // Create a sample terrain dataset for testing
    bool createSampleTerrainData(const std::string& output_dir);
}