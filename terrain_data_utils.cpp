#include "terrain_data_utils.hpp"
#include <fstream>
#include <cmath>
#include <algorithm>

#ifdef HAVE_GDAL
#include "gdal.h"
#include "gdal_priv.h"
#include "cpl_conv.h"
#include "cpl_string.h"
#include <curl/curl.h>
#endif

namespace TerrainDataUtils {

bool isGdalAvailable() {
#ifdef HAVE_GDAL
    return true;
#else
    return false;
#endif
}

#ifdef HAVE_GDAL
// GDAL-enabled implementation
bool convertBilToPgm(const std::string& bil_file, const std::string& pgm_file, TerrainInfo* info) {
    // Initialize GDAL
    GDALAllRegister();
    
    // Open the BIL file
    GDALDataset* dataset = (GDALDataset*)GDALOpen(bil_file.c_str(), GA_ReadOnly);
    if (dataset == nullptr) {
        std::cerr << "Error: Could not open " << bil_file << std::endl;
        return false;
    }
    
    // Get dataset information
    int width = dataset->GetRasterXSize();
    int height = dataset->GetRasterYSize();
    int band_count = dataset->GetRasterCount();
    
    if (band_count == 0) {
        std::cerr << "Error: No raster bands found in " << bil_file << std::endl;
        GDALClose(dataset);
        return false;
    }
    
    // Get the first band (elevation data)
    GDALRasterBand* band = dataset->GetRasterBand(1);
    if (band == nullptr) {
        std::cerr << "Error: Could not get raster band from " << bil_file << std::endl;
        GDALClose(dataset);
        return false;
    }
    
    // Read elevation data
    std::vector<float> elevations(width * height);
    CPLErr err = band->RasterIO(GF_Read, 0, 0, width, height, 
                               elevations.data(), width, height, GDT_Float32, 0, 0);
    
    if (err != CE_None) {
        std::cerr << "Error: Could not read raster data from " << bil_file << std::endl;
        GDALClose(dataset);
        return false;
    }
    
    // Find min/max values for normalization
    auto minmax = std::minmax_element(elevations.begin(), elevations.end());
    float min_elev = *minmax.first;
    float max_elev = *minmax.second;
    
    // Fill in terrain info if requested
    if (info != nullptr) {
        info->width = width;
        info->height = height;
        info->min_elevation = min_elev;
        info->max_elevation = max_elev;
        
        // Get geotransform for pixel size
        double geotransform[6];
        if (dataset->GetGeoTransform(geotransform) == CE_None) {
            info->pixel_size_x = std::abs(geotransform[1]);
            info->pixel_size_y = std::abs(geotransform[5]);
        }
        
        // Get projection if available
        const char* proj = dataset->GetProjectionRef();
        if (proj != nullptr) {
            info->projection = proj;
        }
    }
    
    // Close GDAL dataset
    GDALClose(dataset);
    
    // Write PGM file
    std::ofstream pgm(pgm_file);
    if (!pgm) {
        std::cerr << "Error: Could not create " << pgm_file << std::endl;
        return false;
    }
    
    // Write PGM header
    pgm << "P2\n";
    pgm << width << " " << height << "\n";
    pgm << "65535\n";  // Use 16-bit precision for better elevation representation
    
    // Normalize and write elevation data
    float range = max_elev - min_elev;
    if (range <= 0) range = 1.0f;  // Avoid division by zero
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float normalized = (elevations[y * width + x] - min_elev) / range;
            int pgm_value = static_cast<int>(normalized * 65535.0f);
            pgm_value = std::max(0, std::min(65535, pgm_value));
            pgm << pgm_value;
            if (x < width - 1) pgm << " ";
        }
        pgm << "\n";
    }
    
    std::cout << "Successfully converted " << bil_file << " to " << pgm_file << std::endl;
    std::cout << "Dimensions: " << width << "x" << height << std::endl;
    std::cout << "Elevation range: " << min_elev << " to " << max_elev << std::endl;
    
    return true;
}

// Helper function for curl downloads
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

bool downloadHawaiiTerrainData(const std::string& output_dir) {
    // List of available Hawaii terrain files
    std::vector<std::string> hawaii_files = {
        "bigisland.zip",
        "kauai.zip", 
        "maui.zip",
        "molokai.zip",
        "oahu.zip"
    };
    
    const std::string base_url = "http://gis.ess.washington.edu/data/raster/tenmeter/hawaii/";
    
    // Initialize curl
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Error: Could not initialize curl" << std::endl;
        return false;
    }
    
    bool success = true;
    
    for (const auto& filename : hawaii_files) {
        std::string url = base_url + filename;
        std::string output_file = output_dir + "/" + filename;
        
        std::cout << "Downloading " << url << " to " << output_file << std::endl;
        
        // Open output file
        FILE* fp = fopen(output_file.c_str(), "wb");
        if (!fp) {
            std::cerr << "Error: Could not create " << output_file << std::endl;
            success = false;
            continue;
        }
        
        // Set curl options
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 120L);
        
        // Perform download
        CURLcode res = curl_easy_perform(curl);
        fclose(fp);
        
        if (res != CURLE_OK) {
            std::cerr << "Download failed: " << curl_easy_strerror(res) << std::endl;
            success = false;
        } else {
            std::cout << "Successfully downloaded " << filename << std::endl;
        }
    }
    
    curl_easy_cleanup(curl);
    return success;
}

bool validateMeshAgainstTerrain(const std::string& terrain_file, 
                               const std::vector<float>& mesh_vertices,
                               const std::vector<int>& mesh_triangles,
                               double tolerance) {
    // Initialize GDAL
    GDALAllRegister();
    
    // Open terrain file
    GDALDataset* dataset = (GDALDataset*)GDALOpen(terrain_file.c_str(), GA_ReadOnly);
    if (dataset == nullptr) {
        std::cerr << "Error: Could not open terrain file " << terrain_file << std::endl;
        return false;
    }
    
    // Get terrain properties
    int width = dataset->GetRasterXSize();
    int height = dataset->GetRasterYSize();
    GDALRasterBand* band = dataset->GetRasterBand(1);
    
    if (band == nullptr) {
        std::cerr << "Error: Could not get raster band" << std::endl;
        GDALClose(dataset);
        return false;
    }
    
    // Get geotransform for coordinate conversion
    double geotransform[6];
    if (dataset->GetGeoTransform(geotransform) != CE_None) {
        std::cerr << "Warning: No geotransform available, using pixel coordinates" << std::endl;
        // Set default transform (identity)
        geotransform[0] = 0.0;  // top-left x
        geotransform[1] = 1.0;  // pixel width
        geotransform[2] = 0.0;  // rotation
        geotransform[3] = 0.0;  // top-left y
        geotransform[4] = 0.0;  // rotation
        geotransform[5] = 1.0;  // pixel height
    }
    
    // Validate mesh vertices against terrain
    int valid_vertices = 0;
    int total_vertices = mesh_vertices.size() / 3;
    
    for (int i = 0; i < total_vertices; i++) {
        float x = mesh_vertices[i * 3];
        float y = mesh_vertices[i * 3 + 1];
        float z = mesh_vertices[i * 3 + 2];
        
        // Convert world coordinates to pixel coordinates
        double pixel_x = (x - geotransform[0]) / geotransform[1];
        double pixel_y = (y - geotransform[3]) / geotransform[5];
        
        // Check if within bounds
        if (pixel_x >= 0 && pixel_x < width && pixel_y >= 0 && pixel_y < height) {
            // Sample terrain elevation at this point
            float terrain_elev;
            CPLErr err = band->RasterIO(GF_Read, 
                                      static_cast<int>(pixel_x), static_cast<int>(pixel_y), 
                                      1, 1, &terrain_elev, 1, 1, GDT_Float32, 0, 0);
            
            if (err == CE_None) {
                double elevation_diff = std::abs(z - terrain_elev);
                if (elevation_diff <= tolerance) {
                    valid_vertices++;
                }
            }
        }
    }
    
    GDALClose(dataset);
    
    double validation_ratio = static_cast<double>(valid_vertices) / total_vertices;
    std::cout << "Mesh validation: " << valid_vertices << "/" << total_vertices 
              << " vertices within tolerance (" << (validation_ratio * 100.0) << "%)" << std::endl;
    
    return validation_ratio > 0.8;  // Consider valid if >80% of vertices are within tolerance
}

#else
// Non-GDAL implementation (fallback)
bool convertBilToPgm(const std::string& bil_file, const std::string& pgm_file, TerrainInfo* info) {
    std::cerr << "Error: GDAL support not available. Cannot convert BIL to PGM." << std::endl;
    return false;
}

bool downloadHawaiiTerrainData(const std::string& output_dir) {
    std::cerr << "Error: GDAL support not available. Cannot download terrain data." << std::endl;
    return false;
}

bool validateMeshAgainstTerrain(const std::string& terrain_file, 
                               const std::vector<float>& mesh_vertices,
                               const std::vector<int>& mesh_triangles,
                               double tolerance) {
    std::cerr << "Error: GDAL support not available. Cannot validate mesh." << std::endl;
    return false;
}
#endif

bool createSampleTerrainData(const std::string& output_dir) {
    // Create a synthetic hill terrain for testing when real data is not available
    const int width = 100;
    const int height = 100;
    const std::string filename = output_dir + "/sample_hill.pgm";
    
    std::ofstream pgm(filename);
    if (!pgm) {
        std::cerr << "Error: Could not create " << filename << std::endl;
        return false;
    }
    
    // Write PGM header
    pgm << "P2\n";
    pgm << width << " " << height << "\n";
    pgm << "255\n";
    
    // Generate a simple hill pattern
    double center_x = width / 2.0;
    double center_y = height / 2.0;
    double max_radius = std::min(width, height) / 2.0;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double dx = x - center_x;
            double dy = y - center_y;
            double distance = std::sqrt(dx * dx + dy * dy);
            
            // Create a smooth hill with Gaussian-like falloff
            double normalized_distance = distance / max_radius;
            double elevation = 255.0 * std::exp(-normalized_distance * normalized_distance * 2.0);
            int pgm_value = static_cast<int>(std::max(0.0, std::min(255.0, elevation)));
            
            pgm << pgm_value;
            if (x < width - 1) pgm << " ";
        }
        pgm << "\n";
    }
    
    std::cout << "Created sample terrain data: " << filename << std::endl;
    std::cout << "Dimensions: " << width << "x" << height << std::endl;
    
    return true;
}

} // namespace TerrainDataUtils