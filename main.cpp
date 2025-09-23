#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include "TerraScape.hpp"
#include "cxxopts.hpp"
#ifdef HAVE_GDAL
#include <gdal_priv.h>
#include <gdal.h>
#endif

int main(int argc, char* argv[])
{
    cxxopts::Options options("terrascape_demo", "Terrain Triangle Mesh Generation Demo");
    
    options.add_options()
        ("i,input", "Input terrain file", cxxopts::value<std::string>()->default_value("crater.pgm"))
        ("o,output", "Output OBJ file", cxxopts::value<std::string>()->default_value("terrain.obj"))
        ("h,help", "Print usage");
    
    auto result = options.parse(argc, argv);
    
    if (result.count("help"))
    {
        std::cout << options.help() << std::endl;
        return 0;
    }
    
    std::string input_file = result["input"].as<std::string>();
    std::string output_file = result["output"].as<std::string>();
    
    std::cout << "TerraScape Terrain Triangulation Demo" << std::endl;
    std::cout << "Input: " << input_file << std::endl;
    std::cout << "Output: " << output_file << std::endl;
   
    try {
#ifdef HAVE_GDAL 
        // Initialize GDAL
        GDALAllRegister();
#endif
        // Read terrain data
        TerraScape::TerrainData terrain;
        if (!TerraScape::readTerrainFile(input_file, terrain)) {
            std::cerr << "Error: Failed to read terrain file: " << input_file << std::endl;
            return 1;
        }
        
        std::cout << "Loaded terrain: " << terrain.width << "x" << terrain.height << " cells" << std::endl;
        std::cout << "Height range: " << terrain.min_height << " to " << terrain.max_height << std::endl;
        
        // Generate triangle mesh
        TerraScape::TerrainMesh mesh;
        TerraScape::triangulateTerrainVolume(terrain, mesh);
        
        std::cout << "Generated mesh: " << mesh.vertices.size() << " vertices, " 
                  << mesh.triangles.size() << " triangles" << std::endl;
        
        // Validate mesh properties
        TerraScape::MeshStats stats = TerraScape::validateMesh(mesh, terrain);
        std::cout << "Mesh validation:" << std::endl;
        std::cout << "  Volume: " << stats.volume << " (expected: " << stats.expected_volume << ")" << std::endl;
        std::cout << "  Surface area: " << stats.surface_area << " (expected: " << stats.expected_surface_area << ")" << std::endl;
        std::cout << "  Is manifold: " << (stats.is_manifold ? "yes" : "no") << std::endl;
        std::cout << "  CCW oriented: " << (stats.is_ccw_oriented ? "yes" : "no") << std::endl;
        
        // Write OBJ file
        if (!TerraScape::writeObjFile(output_file, mesh)) {
            std::cerr << "Error: Failed to write OBJ file: " << output_file << std::endl;
            return 1;
        }
        
        std::cout << "Successfully wrote mesh to: " << output_file << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
