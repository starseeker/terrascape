#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cstring>
#include <filesystem>

#include "terrain_data_utils.hpp"
#include "TerraScape.hpp"

namespace fs = std::filesystem;

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]\n";
    std::cout << "Options:\n";
    std::cout << "  --download-only    Download terrain data only, don't process\n";
    std::cout << "  --process-only     Process existing terrain data only\n";
    std::cout << "  --validate         Validate generated meshes against terrain data\n";
    std::cout << "  --help             Show this help message\n";
    std::cout << "\n";
    std::cout << "This tool downloads real terrain data from Hawaii and processes it\n";
    std::cout << "with TerraScape to demonstrate mesh generation from real-world data.\n";
}

bool processTerrainFile(const std::string& bil_file, const std::string& output_dir, bool validate = false) {
    if (!TerrainDataUtils::isGdalAvailable()) {
        std::cout << "GDAL not available - creating sample terrain data instead\n";
        return TerrainDataUtils::createSampleTerrainData(output_dir);
    }
    
    // Extract base filename for PGM and OBJ outputs
    fs::path bil_path(bil_file);
    std::string base_name = bil_path.stem().string();
    
    std::string pgm_file = output_dir + "/" + base_name + ".pgm";
    std::string obj_file = output_dir + "/" + base_name + "_mesh.obj";
    
    std::cout << "\n=== Processing " << bil_file << " ===\n";
    
    // Convert BIL to PGM
    TerrainDataUtils::TerrainInfo terrain_info;
    if (!TerrainDataUtils::convertBilToPgm(bil_file, pgm_file, &terrain_info)) {
        std::cerr << "Failed to convert " << bil_file << " to PGM format\n";
        return false;
    }
    
    // Read the PGM file into elevation data
    std::ifstream pgm_stream(pgm_file);
    if (!pgm_stream) {
        std::cerr << "Could not read generated PGM file: " << pgm_file << "\n";
        return false;
    }
    
    std::string line;
    std::getline(pgm_stream, line); // Magic number
    std::getline(pgm_stream, line); // Dimensions
    std::getline(pgm_stream, line); // Max value
    
    std::vector<float> elevations(terrain_info.width * terrain_info.height);
    for (int i = 0; i < terrain_info.width * terrain_info.height; i++) {
        float val;
        pgm_stream >> val;
        elevations[i] = val;
    }
    
    // Generate mesh using TerraScape
    std::cout << "Generating mesh from " << terrain_info.width << "x" << terrain_info.height << " terrain data...\n";
    
    // Use reasonable parameters for real terrain data
    float error_threshold = static_cast<float>((terrain_info.max_elevation - terrain_info.min_elevation) * 0.01); // 1% of elevation range
    int point_limit = std::min(5000, terrain_info.width * terrain_info.height / 10); // At most 10% of original points
    
    auto mesh = TerraScape::grid_to_mesh(terrain_info.width, terrain_info.height, 
                                        elevations.data(), error_threshold);
    
    std::cout << "Generated mesh: " << mesh.vertices.size() << " vertices, " 
              << mesh.triangles.size() << " triangles\n";
    
    // Write mesh to OBJ file
    std::ofstream obj_stream(obj_file);
    if (!obj_stream) {
        std::cerr << "Could not create OBJ file: " << obj_file << "\n";
        return false;
    }
    
    // Write vertices
    for (const auto& vertex : mesh.vertices) {
        obj_stream << "v " << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
    }
    
    // Write triangles (OBJ uses 1-based indexing)
    for (const auto& triangle : mesh.triangles) {
        obj_stream << "f " << (triangle.v0 + 1) << " " << (triangle.v1 + 1) << " " << (triangle.v2 + 1) << "\n";
    }
    
    std::cout << "Mesh saved to: " << obj_file << "\n";
    
    // Validate mesh if requested
    if (validate && TerrainDataUtils::isGdalAvailable()) {
        std::cout << "Validating mesh against original terrain data...\n";
        
        // Convert mesh to flat vertex array for validation
        std::vector<float> mesh_vertices_flat;
        for (const auto& vertex : mesh.vertices) {
            mesh_vertices_flat.push_back(vertex.x);
            mesh_vertices_flat.push_back(vertex.y);
            mesh_vertices_flat.push_back(vertex.z);
        }
        
        // Convert triangles to flat index array
        std::vector<int> mesh_triangles_flat;
        for (const auto& triangle : mesh.triangles) {
            mesh_triangles_flat.push_back(triangle.v0);
            mesh_triangles_flat.push_back(triangle.v1);
            mesh_triangles_flat.push_back(triangle.v2);
        }
        
        double tolerance = (terrain_info.max_elevation - terrain_info.min_elevation) * 0.05; // 5% tolerance
        bool validation_result = TerrainDataUtils::validateMeshAgainstTerrain(
            bil_file, mesh_vertices_flat, mesh_triangles_flat, tolerance);
        
        if (validation_result) {
            std::cout << "✓ Mesh validation PASSED\n";
        } else {
            std::cout << "✗ Mesh validation FAILED\n";
        }
    }
    
    return true;
}

int main(int argc, char* argv[]) {
    std::cout << "TerraScape Terrain Data Processor\n";
    std::cout << "Processing real-world terrain data with GDAL support\n\n";
    
    if (argc > 1 && strcmp(argv[1], "--help") == 0) {
        printUsage(argv[0]);
        return 0;
    }
    
    bool download_only = false;
    bool process_only = false;
    bool validate = false;
    
    // Parse command line options
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--download-only") == 0) {
            download_only = true;
        } else if (strcmp(argv[i], "--process-only") == 0) {
            process_only = true;
        } else if (strcmp(argv[i], "--validate") == 0) {
            validate = true;
        }
    }
    
    std::string terrain_data_dir = "terrain_data";
    
    // Create terrain data directory
    if (!fs::exists(terrain_data_dir)) {
        fs::create_directories(terrain_data_dir);
    }
    
    // Check GDAL availability
    if (!TerrainDataUtils::isGdalAvailable()) {
        std::cout << "GDAL not available - creating sample terrain data instead\n";
        bool success = TerrainDataUtils::createSampleTerrainData(terrain_data_dir);
        if (success) {
            std::cout << "Sample terrain data created successfully\n";
            
            // Process the sample data
            std::string sample_file = terrain_data_dir + "/sample_hill.pgm";
            std::ifstream test_stream(sample_file);
            if (test_stream) {
                std::string line;
                std::getline(test_stream, line); // Magic
                int width, height, maxval;
                test_stream >> width >> height >> maxval;
                
                std::vector<float> elevations(width * height);
                for (int i = 0; i < width * height; i++) {
                    float val;
                    test_stream >> val;
                    elevations[i] = val;
                }
                
                std::cout << "Processing sample terrain (" << width << "x" << height << ")...\n";
                auto mesh = TerraScape::grid_to_mesh(width, height, elevations.data(), 10.0f);
                std::cout << "Generated mesh: " << mesh.vertices.size() << " vertices, " 
                          << mesh.triangles.size() << " triangles\n";
                
                // Save mesh
                std::string obj_file = terrain_data_dir + "/sample_hill_mesh.obj";
                std::ofstream obj_stream(obj_file);
                for (const auto& vertex : mesh.vertices) {
                    obj_stream << "v " << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
                }
                for (const auto& triangle : mesh.triangles) {
                    obj_stream << "f " << (triangle.v0 + 1) << " " << (triangle.v1 + 1) << " " << (triangle.v2 + 1) << "\n";
                }
                std::cout << "Sample mesh saved to: " << obj_file << "\n";
            }
        }
        return success ? 0 : 1;
    }
    
    // Download terrain data (if not process-only)
    if (!process_only) {
        std::cout << "Downloading Hawaii terrain data...\n";
        bool download_success = TerrainDataUtils::downloadHawaiiTerrainData(terrain_data_dir);
        
        if (!download_success) {
            std::cerr << "Warning: Failed to download some or all terrain data files\n";
            std::cout << "Creating sample terrain data instead...\n";
            TerrainDataUtils::createSampleTerrainData(terrain_data_dir);
        }
        
        if (download_only) {
            std::cout << "Download complete. Use --process-only to process the data.\n";
            return download_success ? 0 : 1;
        }
    }
    
    // Process terrain data
    if (!download_only) {
        std::cout << "\nProcessing terrain data...\n";
        
        // Look for BIL files in the terrain data directory
        std::vector<std::string> bil_files;
        
        for (const auto& entry : fs::directory_iterator(terrain_data_dir)) {
            if (entry.path().extension() == ".bil") {
                bil_files.push_back(entry.path().string());
            }
        }
        
        if (bil_files.empty()) {
            std::cout << "No BIL files found. Processing sample terrain data...\n";
            TerrainDataUtils::createSampleTerrainData(terrain_data_dir);
            
            // Process the sample data
            std::string sample_file = terrain_data_dir + "/sample_hill.pgm";
            std::ifstream test_stream(sample_file);
            if (test_stream) {
                std::string line;
                std::getline(test_stream, line); // Magic
                int width, height, maxval;
                test_stream >> width >> height >> maxval;
                
                std::vector<float> elevations(width * height);
                for (int i = 0; i < width * height; i++) {
                    float val;
                    test_stream >> val;
                    elevations[i] = val;
                }
                
                std::cout << "Processing sample terrain (" << width << "x" << height << ")...\n";
                auto mesh = TerraScape::grid_to_mesh(width, height, elevations.data(), 10.0f);
                std::cout << "Generated mesh: " << mesh.vertices.size() << " vertices, " 
                          << mesh.triangles.size() << " triangles\n";
            }
        } else {
            // Process each BIL file
            int successful = 0;
            for (const auto& bil_file : bil_files) {
                if (processTerrainFile(bil_file, terrain_data_dir, validate)) {
                    successful++;
                }
            }
            
            std::cout << "\nProcessing complete: " << successful << "/" << bil_files.size() 
                      << " files processed successfully\n";
        }
    }
    
    std::cout << "\nTerraScape terrain data processing complete!\n";
    return 0;
}