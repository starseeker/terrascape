#pragma once

#include <vector>
#include <string>
#include <cstdint>

namespace DSPReader {
    
    // Structure to hold DSP terrain data information
    struct DSPTerrainInfo {
        int width = 0;
        int height = 0;
        double min_elevation = 0.0;
        double max_elevation = 0.0;
        std::string data_type; // "uint8", "uint16", "float", etc.
        size_t data_size = 0;  // Total size of data in bytes
    };
    
    // DSP file header structure (simplified BRL-CAD-like format)
    struct DSPHeader {
        uint32_t magic;        // Magic number to identify DSP format
        uint32_t width;        // Width of elevation grid
        uint32_t height;       // Height of elevation grid  
        uint32_t data_type;    // Data type (1=uint8, 2=uint16, 3=float, etc.)
        uint32_t reserved[4];  // Reserved for future use
    };
    
    // Read a BRL-CAD style DSP binary file
    bool readDSPFile(const std::string& filename, 
                     int& width, int& height, 
                     std::vector<float>& elevations,
                     DSPTerrainInfo* info = nullptr);
    
    // Create a synthetic terra.bin file for testing
    bool createSyntheticTerraBin(const std::string& filename,
                                 int width, int height,
                                 const std::vector<float>& elevations);
    
    // Create various test patterns that might cause triangulation issues
    bool createProblematicTerraBin(const std::string& filename,
                                   const std::string& pattern_type);
    
    // Try to read terra.bin file with various format assumptions
    bool readTerraBinFile(const std::string& filename,
                          int& width, int& height,
                          std::vector<float>& elevations,
                          DSPTerrainInfo* info = nullptr);
    
    // Validate binary terrain data for potential triangulation issues
    bool validateTerrainData(const std::vector<float>& elevations,
                            int width, int height,
                            std::vector<std::string>& warnings);
}