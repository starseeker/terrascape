#include "dsp_reader.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace DSPReader {

bool readDSPFile(const std::string& filename, 
                 int& width, int& height, 
                 std::vector<float>& elevations,
                 DSPTerrainInfo* info) {
    
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Cannot open DSP file " << filename << std::endl;
        return false;
    }
    
    // Try to read DSP header
    DSPHeader header;
    file.read(reinterpret_cast<char*>(&header), sizeof(header));
    
    if (!file.good()) {
        std::cerr << "Error: Failed to read DSP header from " << filename << std::endl;
        return false;
    }
    
    // Check for valid magic number (simplified approach)
    // Different DSP variants might have different magic numbers
    const uint32_t DSP_MAGIC_1 = 0x44535020; // "DSP "
    const uint32_t DSP_MAGIC_2 = 0x20505344; // " PSD" (reversed)
    
    bool has_header = (header.magic == DSP_MAGIC_1 || header.magic == DSP_MAGIC_2);
    
    if (has_header) {
        width = header.width;
        height = header.height;
        
        if (info) {
            info->width = width;
            info->height = height;
            info->data_type = (header.data_type == 1) ? "uint8" : 
                             (header.data_type == 2) ? "uint16" : 
                             (header.data_type == 3) ? "float" : "unknown";
        }
        
        std::cout << "Found DSP header: " << width << "x" << height 
                  << ", data_type=" << header.data_type << std::endl;
    } else {
        // No header found, rewind and try to guess format
        file.seekg(0, std::ios::beg);
        
        // Get file size to guess dimensions
        file.seekg(0, std::ios::end);
        size_t file_size = file.tellg();
        file.seekg(0, std::ios::beg);
        
        if (info) {
            info->data_size = file_size;
        }
        
        std::cout << "No DSP header found, file size: " << file_size << " bytes" << std::endl;
        
        // Try common square dimensions first
        std::vector<int> common_sizes = {256, 512, 1024, 128, 64, 32, 100, 200, 300, 400, 500};
        
        bool found_dimensions = false;
        for (int size : common_sizes) {
            // Try square dimensions
            if (size * size * sizeof(float) == file_size) {
                width = height = size;
                found_dimensions = true;
                std::cout << "Guessed square dimensions: " << width << "x" << height << " (float)" << std::endl;
                break;
            }
            if (size * size * sizeof(uint16_t) == file_size) {
                width = height = size;
                found_dimensions = true;
                std::cout << "Guessed square dimensions: " << width << "x" << height << " (uint16)" << std::endl;
                break;
            }
            if (size * size * sizeof(uint8_t) == file_size) {
                width = height = size;
                found_dimensions = true;
                std::cout << "Guessed square dimensions: " << width << "x" << height << " (uint8)" << std::endl;
                break;
            }
        }
        
        if (!found_dimensions) {
            // Try some rectangular dimensions
            for (int w = 100; w <= 1000; w += 50) {
                for (int h = 100; h <= 1000; h += 50) {
                    if (w * h * sizeof(float) == file_size) {
                        width = w;
                        height = h;
                        found_dimensions = true;
                        std::cout << "Guessed rectangular dimensions: " << width << "x" << height << " (float)" << std::endl;
                        break;
                    }
                }
                if (found_dimensions) break;
            }
        }
        
        if (!found_dimensions) {
            std::cerr << "Error: Could not determine dimensions for file size " << file_size << std::endl;
            return false;
        }
    }
    
    // Read elevation data
    size_t data_points = width * height;
    elevations.resize(data_points);
    
    // Determine data format and read accordingly
    if (has_header) {
        switch (header.data_type) {
            case 1: { // uint8
                std::vector<uint8_t> raw_data(data_points);
                file.read(reinterpret_cast<char*>(raw_data.data()), data_points);
                for (size_t i = 0; i < data_points; ++i) {
                    elevations[i] = static_cast<float>(raw_data[i]);
                }
                break;
            }
            case 2: { // uint16
                std::vector<uint16_t> raw_data(data_points);
                file.read(reinterpret_cast<char*>(raw_data.data()), data_points * sizeof(uint16_t));
                for (size_t i = 0; i < data_points; ++i) {
                    elevations[i] = static_cast<float>(raw_data[i]);
                }
                break;
            }
            case 3: { // float
                file.read(reinterpret_cast<char*>(elevations.data()), data_points * sizeof(float));
                break;
            }
            default:
                std::cerr << "Error: Unsupported data type " << header.data_type << std::endl;
                return false;
        }
    } else {
        // Try float first, then fallback to other types
        file.read(reinterpret_cast<char*>(elevations.data()), data_points * sizeof(float));
    }
    
    if (!file.good() && !file.eof()) {
        std::cerr << "Error: Failed to read elevation data" << std::endl;
        return false;
    }
    
    // Calculate min/max for info
    if (!elevations.empty()) {
        auto minmax = std::minmax_element(elevations.begin(), elevations.end());
        if (info) {
            info->min_elevation = *minmax.first;
            info->max_elevation = *minmax.second;
        }
        
        std::cout << "Elevation range: " << *minmax.first << " to " << *minmax.second << std::endl;
    }
    
    return true;
}

bool createSyntheticTerraBin(const std::string& filename,
                             int width, int height,
                             const std::vector<float>& elevations) {
    
    if (elevations.size() != static_cast<size_t>(width * height)) {
        std::cerr << "Error: Elevation data size doesn't match dimensions" << std::endl;
        return false;
    }
    
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Cannot create " << filename << std::endl;
        return false;
    }
    
    // Write simple binary format (just raw float data, no header)
    file.write(reinterpret_cast<const char*>(elevations.data()), 
               elevations.size() * sizeof(float));
    
    if (!file.good()) {
        std::cerr << "Error: Failed to write terrain data" << std::endl;
        return false;
    }
    
    std::cout << "Created synthetic terra.bin: " << width << "x" << height 
              << " (" << elevations.size() * sizeof(float) << " bytes)" << std::endl;
    
    return true;
}

bool createProblematicTerraBin(const std::string& filename,
                               const std::string& pattern_type) {
    
    int width = 256, height = 256;
    std::vector<float> elevations(width * height);
    
    if (pattern_type == "all_zeros") {
        // All zeros - perfectly flat, should trigger flat data handling
        std::fill(elevations.begin(), elevations.end(), 0.0f);
        
    } else if (pattern_type == "all_same") {
        // All same non-zero value - also perfectly flat
        std::fill(elevations.begin(), elevations.end(), 100.0f);
        
    } else if (pattern_type == "regular_grid") {
        // Regular checkerboard pattern - might cause triangulation degeneracy
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float value = ((x + y) % 2 == 0) ? 0.0f : 1.0f;
                elevations[y * width + x] = value;
            }
        }
        
    } else if (pattern_type == "collinear_rows") {
        // Each row has linear progression - collinear points
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                elevations[y * width + x] = static_cast<float>(x);
            }
        }
        
    } else if (pattern_type == "infinities") {
        // Pattern with infinite/NaN values
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if ((x + y) % 10 == 0) {
                    elevations[y * width + x] = std::numeric_limits<float>::infinity();
                } else if ((x + y) % 13 == 0) {
                    elevations[y * width + x] = std::numeric_limits<float>::quiet_NaN();
                } else {
                    elevations[y * width + x] = static_cast<float>(x + y);
                }
            }
        }
        
    } else if (pattern_type == "extreme_values") {
        // Mix of very large and very small values
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if ((x + y) % 2 == 0) {
                    elevations[y * width + x] = 1e20f;  // Very large
                } else {
                    elevations[y * width + x] = -1e20f; // Very small
                }
            }
        }
        
    } else {
        // Default: simple hill pattern
        float center_x = width / 2.0f;
        float center_y = height / 2.0f;
        float max_radius = std::min(width, height) / 2.0f;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float dx = x - center_x;
                float dy = y - center_y;
                float distance = std::sqrt(dx * dx + dy * dy);
                float normalized_distance = distance / max_radius;
                elevations[y * width + x] = 100.0f * std::exp(-normalized_distance * normalized_distance);
            }
        }
    }
    
    return createSyntheticTerraBin(filename, width, height, elevations);
}

bool readTerraBinFile(const std::string& filename,
                      int& width, int& height,
                      std::vector<float>& elevations,
                      DSPTerrainInfo* info) {
    
    std::cout << "Attempting to read terra.bin file: " << filename << std::endl;
    
    // First try as DSP format
    if (readDSPFile(filename, width, height, elevations, info)) {
        std::cout << "Successfully read as DSP format" << std::endl;
        return true;
    }
    
    std::cout << "DSP format failed, trying raw binary formats..." << std::endl;
    
    // If DSP format fails, try various raw binary formats
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Cannot open " << filename << std::endl;
        return false;
    }
    
    // Get file size
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    std::cout << "File size: " << file_size << " bytes" << std::endl;
    
    // Try common terra.bin dimensions (based on typical BRL-CAD DSP usage)
    std::vector<std::pair<int, int>> common_dims = {
        {512, 512}, {256, 256}, {1024, 1024}, {128, 128},
        {400, 400}, {300, 300}, {200, 200}, {100, 100},
        {512, 256}, {256, 512}, {1024, 512}, {512, 1024}
    };
    
    for (auto& dims : common_dims) {
        int w = dims.first;
        int h = dims.second;
        
        // Try different data types
        if (w * h * sizeof(float) == file_size) {
            width = w;
            height = h;
            elevations.resize(w * h);
            file.seekg(0, std::ios::beg);
            file.read(reinterpret_cast<char*>(elevations.data()), file_size);
            
            if (file.good()) {
                std::cout << "Successfully read as " << w << "x" << h << " float array" << std::endl;
                
                auto minmax = std::minmax_element(elevations.begin(), elevations.end());
                if (info) {
                    info->width = width;
                    info->height = height;
                    info->min_elevation = *minmax.first;
                    info->max_elevation = *minmax.second;
                    info->data_type = "float";
                    info->data_size = file_size;
                }
                
                std::cout << "Elevation range: " << *minmax.first << " to " << *minmax.second << std::endl;
                return true;
            }
        }
        
        if (w * h * sizeof(uint16_t) == file_size) {
            width = w;
            height = h;
            std::vector<uint16_t> raw_data(w * h);
            file.seekg(0, std::ios::beg);
            file.read(reinterpret_cast<char*>(raw_data.data()), file_size);
            
            if (file.good()) {
                elevations.resize(w * h);
                for (size_t i = 0; i < raw_data.size(); ++i) {
                    elevations[i] = static_cast<float>(raw_data[i]);
                }
                
                std::cout << "Successfully read as " << w << "x" << h << " uint16 array" << std::endl;
                
                auto minmax = std::minmax_element(elevations.begin(), elevations.end());
                if (info) {
                    info->width = width;
                    info->height = height;
                    info->min_elevation = *minmax.first;
                    info->max_elevation = *minmax.second;
                    info->data_type = "uint16";
                    info->data_size = file_size;
                }
                
                std::cout << "Elevation range: " << *minmax.first << " to " << *minmax.second << std::endl;
                return true;
            }
        }
    }
    
    std::cerr << "Error: Could not determine format for terra.bin file" << std::endl;
    return false;
}

bool validateTerrainData(const std::vector<float>& elevations,
                        int width, int height,
                        std::vector<std::string>& warnings) {
    
    warnings.clear();
    
    if (elevations.empty()) {
        warnings.push_back("Empty elevation data");
        return false;
    }
    
    if (elevations.size() != static_cast<size_t>(width * height)) {
        warnings.push_back("Elevation data size doesn't match dimensions");
        return false;
    }
    
    // Check for invalid values
    int invalid_count = 0;
    int zero_count = 0;
    int identical_count = 0;
    
    float first_value = elevations[0];
    bool all_identical = true;
    
    for (const float& val : elevations) {
        if (!std::isfinite(val)) {
            invalid_count++;
        }
        if (val == 0.0f) {
            zero_count++;
        }
        if (val != first_value) {
            all_identical = false;
        }
    }
    
    if (all_identical) {
        identical_count = elevations.size();
    }
    
    // Calculate statistics
    auto minmax = std::minmax_element(elevations.begin(), elevations.end());
    float min_val = *minmax.first;
    float max_val = *minmax.second;
    float range = max_val - min_val;
    
    // Generate warnings
    if (invalid_count > 0) {
        warnings.push_back("Found " + std::to_string(invalid_count) + " invalid values (inf/NaN)");
    }
    
    if (range == 0.0f) {
        warnings.push_back("Data is perfectly flat (elevation range: 0)");
    } else if (range < 1e-10f) {
        warnings.push_back("Data is nearly flat (elevation range: " + std::to_string(range) + ")");
    }
    
    if (zero_count == static_cast<int>(elevations.size())) {
        warnings.push_back("All elevation values are zero");
    } else if (zero_count > static_cast<int>(elevations.size()) * 0.8) {
        warnings.push_back("More than 80% of elevation values are zero");
    }
    
    if (identical_count == static_cast<int>(elevations.size())) {
        warnings.push_back("All elevation values are identical");
    }
    
    // Check for regular patterns
    bool has_checkerboard = true;
    for (int y = 0; y < height - 1 && has_checkerboard; ++y) {
        for (int x = 0; x < width - 1 && has_checkerboard; ++x) {
            float val1 = elevations[y * width + x];
            float val2 = elevations[y * width + (x + 1)];
            float val3 = elevations[(y + 1) * width + x];
            
            if ((val1 == val2) && (val1 == val3)) {
                has_checkerboard = false;
            }
        }
    }
    
    if (has_checkerboard && range < 10.0f) {
        warnings.push_back("Potential checkerboard pattern detected");
    }
    
    return warnings.empty();
}

} // namespace DSPReader