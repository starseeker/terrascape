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
        ("s,simplified", "Use Terra/Scape simplified triangulation")
        ("surface-only", "Generate surface-only mesh (no volume)")
        ("components", "Handle terrain islands and holes separately (default)")
        ("legacy", "Use legacy single-mesh approach (may connect disjoint islands)")
        ("brlcad-test", "Test BRL-CAD DSP integration")
        ("dsp-test", "Test specific DSP binary file (requires --width and --height)")
        ("width", "Width for DSP binary file", cxxopts::value<int>()->default_value("256"))
        ("height", "Height for DSP binary file", cxxopts::value<int>()->default_value("256"))
        ("cell-size", "Physical cell size for DSP", cxxopts::value<double>()->default_value("1.0"))
        ("height-scale", "Height scaling factor for DSP (default: auto-scale to 0-100 range)", cxxopts::value<double>()->default_value("0.0"))
        ("manifold-validation", "Validate mesh using @elalish/manifold library")
        ("e,error", "Error threshold for simplification", cxxopts::value<double>()->default_value("0.1"))
        ("r,reduction", "Minimum triangle reduction percentage", cxxopts::value<int>()->default_value("70"))
        ("h,help", "Print usage");
    
    auto result = options.parse(argc, argv);
    
    if (result.count("help"))
    {
        std::cout << options.help() << std::endl;
        return 0;
    }
    
    std::string input_file = result["input"].as<std::string>();
    std::string output_file = result["output"].as<std::string>();
    bool use_simplified = result.count("simplified") > 0;
    bool surface_only = result.count("surface-only") > 0;
    bool use_components = result.count("components") > 0;
    bool use_legacy = result.count("legacy") > 0;
    bool brlcad_test = result.count("brlcad-test") > 0;
    bool dsp_test = result.count("dsp-test") > 0;
    bool manifold_validation = result.count("manifold-validation") > 0;
    int dsp_width = result["width"].as<int>();
    int dsp_height = result["height"].as<int>();
    double dsp_cell_size = result["cell-size"].as<double>();
    double height_scale = result["height-scale"].as<double>();
    double error_threshold = result.count("error") ? result["error"].as<double>() : 0.1;
    int reduction_percent = result.count("reduction") ? result["reduction"].as<int>() : 70;
    
    std::cout << "TerraScape Terrain Triangulation Demo" << std::endl;
    std::cout << "Input: " << input_file << std::endl;
    std::cout << "Output: " << output_file << std::endl;
    
    // BRL-CAD DSP integration test
    if (brlcad_test) {
        std::cout << "Mode: BRL-CAD DSP Integration Test" << std::endl;
        
        // Load terrain data first
        TerraScape::TerrainData terrain;
        if (!TerraScape::readTerrainFile(input_file, terrain)) {
            std::cerr << "Error: Failed to read terrain file: " << input_file << std::endl;
            return 1;
        }
        
        std::cout << "Loaded terrain: " << terrain.width << "x" << terrain.height << " cells" << std::endl;
        
        // Convert to DSP format
        TerraScape::DSPData dsp;
        if (!TerraScape::convertTerrainToDSP(terrain, dsp)) {
            std::cerr << "Error: Failed to convert terrain to DSP format" << std::endl;
            return 1;
        }
        
        // Test BRL-CAD integration
        TerraScape::NMGTriangleData nmg_data;
        if (!TerraScape::triangulateTerrainForBRLCAD(dsp, nmg_data)) {
            std::cerr << "Error: BRL-CAD triangulation failed" << std::endl;
            return 1;
        }
        
        std::cout << "BRL-CAD Integration Results:" << std::endl;
        std::cout << "  NMG triangles: " << nmg_data.triangles.size() << std::endl;
        std::cout << "  Surface triangles: " << nmg_data.surface_triangle_count << std::endl;
        std::cout << "  Unique vertices: " << nmg_data.unique_vertices.size() << std::endl;
        
        // Convert back to regular mesh for validation and output
        TerraScape::TerrainMesh mesh;
        mesh.vertices = nmg_data.unique_vertices;
        mesh.surface_triangle_count = nmg_data.surface_triangle_count;
        
        for (const auto& nmg_tri : nmg_data.triangles) {
            TerraScape::Triangle tri;
            tri.vertices[0] = nmg_tri.vertices[0].original_index;
            tri.vertices[1] = nmg_tri.vertices[1].original_index;
            tri.vertices[2] = nmg_tri.vertices[2].original_index;
            tri.normal = nmg_tri.normal;
            mesh.triangles.push_back(tri);
        }
        
        // Validate the mesh
        TerraScape::MeshStats stats = TerraScape::validateMesh(mesh, terrain);
        std::cout << "Mesh validation:" << std::endl;
        std::cout << "  Volume: " << stats.volume << " (expected: " << stats.expected_volume << ")" << std::endl;
        std::cout << "  Surface area: " << stats.surface_area << " (expected: " << stats.expected_surface_area << ")" << std::endl;
        std::cout << "  Is manifold: " << (stats.is_manifold ? "yes" : "no") << std::endl;
        std::cout << "  CCW oriented: " << (stats.is_ccw_oriented ? "yes" : "no") << std::endl;
        
        // Perform manifold validation using @elalish/manifold library
        if (manifold_validation) {
            std::cout << "\nManifold Library Validation:" << std::endl;
            TerraScape::ManifoldValidationInfo manifold_info = TerraScape::validateMeshWithManifold(mesh);
            std::cout << "  Vertices: " << manifold_info.num_vertices << std::endl;
            std::cout << "  Triangles: " << manifold_info.num_triangles << std::endl;
            std::cout << "  Is manifold: " << (manifold_info.is_manifold ? "yes" : "no") << std::endl;
            std::cout << "  Status: " << manifold_info.error_message << std::endl;
            
            // If there's a validation issue, report it
            if (!manifold_info.is_manifold) {
                std::cerr << "Warning: Mesh failed manifold library validation!" << std::endl;
                std::cerr << "  Issue: " << manifold_info.error_message << std::endl;
            }
        }
        
        if (TerraScape::writeObjFile(output_file, mesh)) {
            std::cout << "Successfully wrote BRL-CAD compatible mesh to: " << output_file << std::endl;
        } else {
            std::cerr << "Failed to write output file: " << output_file << std::endl;
        }
        
        return 0;
    }
    
    // DSP Binary File Test
    if (dsp_test) {
        std::cout << "Mode: DSP Binary File Test" << std::endl;
        std::cout << "DSP dimensions: " << dsp_width << "x" << dsp_height << std::endl;
        std::cout << "Cell size: " << dsp_cell_size << std::endl;
        
        // Test 1: Read as TerrainData
        TerraScape::TerrainData terrain;
        bool success;
        if (height_scale > 0.0) {
            success = TerraScape::readDSPBinaryFile(input_file, terrain, dsp_width, dsp_height, dsp_cell_size, height_scale);
            std::cout << "Using custom height scale: " << height_scale << std::endl;
        } else {
            success = TerraScape::readDSPBinaryFile(input_file, terrain, dsp_width, dsp_height, dsp_cell_size, height_scale);
            std::cout << "Using auto-scaling (0-100 range)" << std::endl;
        }
        
        if (!success) {
            std::cerr << "Error: Failed to read DSP binary file: " << input_file << std::endl;
            return 1;
        }
        
        std::cout << "Successfully loaded DSP binary file:" << std::endl;
        std::cout << "  Dimensions: " << terrain.width << "x" << terrain.height << " cells" << std::endl;
        std::cout << "  Height range: " << terrain.min_height << " to " << terrain.max_height << std::endl;
        std::cout << "  Cell size: " << terrain.cell_size << std::endl;
        
        // Test 2: Read directly as DSPData and convert
        TerraScape::DSPData dsp;
        if (!TerraScape::readDSPBinaryFile(input_file, dsp, dsp_width, dsp_height)) {
            std::cerr << "Error: Failed to read DSP file directly to DSPData" << std::endl;
            return 1;
        }
        
        std::cout << "DSP Data verification:" << std::endl;
        std::cout << "  First few height values: ";
        for (int i = 0; i < std::min(10, static_cast<int>(dsp.dsp_xcnt)); i++) {
            std::cout << dsp.getHeight(i, 0) << " ";
        }
        std::cout << std::endl;
        
        // Test 3: Full BRL-CAD processing pipeline
        TerraScape::NMGTriangleData nmg_data;
        if (!TerraScape::triangulateTerrainForBRLCAD(dsp, nmg_data)) {
            std::cerr << "Error: BRL-CAD DSP triangulation failed" << std::endl;
            return 1;
        }
        
        std::cout << "BRL-CAD DSP Processing Results:" << std::endl;
        std::cout << "  NMG triangles: " << nmg_data.triangles.size() << std::endl;
        std::cout << "  Surface triangles: " << nmg_data.surface_triangle_count << std::endl;
        std::cout << "  Unique vertices: " << nmg_data.unique_vertices.size() << std::endl;
        
        // Convert back to regular mesh for detailed analysis
        TerraScape::TerrainMesh mesh;
        mesh.vertices = nmg_data.unique_vertices;
        mesh.surface_triangle_count = nmg_data.surface_triangle_count;
        
        for (const auto& nmg_tri : nmg_data.triangles) {
            TerraScape::Triangle tri;
            tri.vertices[0] = nmg_tri.vertices[0].original_index;
            tri.vertices[1] = nmg_tri.vertices[1].original_index;
            tri.vertices[2] = nmg_tri.vertices[2].original_index;
            tri.normal = nmg_tri.normal;
            mesh.triangles.push_back(tri);
        }
        
        // Detailed mesh validation
        TerraScape::MeshStats stats = TerraScape::validateMesh(mesh, terrain);
        std::cout << "Detailed Mesh Analysis:" << std::endl;
        std::cout << "  Volume: " << stats.volume << " (expected: " << stats.expected_volume << ")" << std::endl;
        std::cout << "  Surface area: " << stats.surface_area << " (expected: " << stats.expected_surface_area << ")" << std::endl;
        std::cout << "  Is manifold: " << (stats.is_manifold ? "YES" : "NO") << std::endl;
        std::cout << "  CCW oriented: " << (stats.is_ccw_oriented ? "YES" : "NO") << std::endl;
        std::cout << "  Non-manifold edges: " << stats.non_manifold_edges << std::endl;
        
        // Perform manifold validation with @elalish/manifold library
        std::cout << "\nManifold Library Validation:" << std::endl;
        TerraScape::ManifoldValidationInfo manifold_info = TerraScape::validateMeshWithManifold(mesh);
        std::cout << "  Result: ";
        switch (manifold_info.result) {
            case TerraScape::ManifoldValidationResult::Valid:
                std::cout << "VALID"; break;
            case TerraScape::ManifoldValidationResult::NonFiniteVertex:
                std::cout << "NON-FINITE VERTEX"; break;
            case TerraScape::ManifoldValidationResult::NotManifold:
                std::cout << "NOT MANIFOLD"; break;
            case TerraScape::ManifoldValidationResult::VertexOutOfBounds:
                std::cout << "VERTEX OUT OF BOUNDS"; break;
            case TerraScape::ManifoldValidationResult::OtherError:
            default:
                std::cout << "OTHER ERROR"; break;
        }
        std::cout << std::endl;
        std::cout << "  Is manifold: " << (manifold_info.is_manifold ? "YES" : "NO") << std::endl;
        std::cout << "  Error message: " << manifold_info.error_message << std::endl;
        
        // Write output file
        if (TerraScape::writeObjFile(output_file, mesh)) {
            std::cout << "Successfully wrote DSP mesh to: " << output_file << std::endl;
        } else {
            std::cerr << "Failed to write output file: " << output_file << std::endl;
        }
        
        return 0;
    }
    
    if (use_components) {
        std::cout << "Mode: Components (separate islands and holes)" << std::endl;
    } else if (use_legacy) {
        std::cout << "Mode: Legacy (single connected mesh)" << std::endl;
    } else if (surface_only) {
        std::cout << "Mode: Surface-only (Terra/Scape)" << std::endl;
    } else if (use_simplified) {
        std::cout << "Mode: Simplified (Terra/Scape)" << std::endl;
    } else {
        std::cout << "Mode: Dense (with component analysis)" << std::endl;
    }
    if (use_simplified || surface_only) {
        std::cout << "Error threshold: " << error_threshold << std::endl;
        std::cout << "Target reduction: " << reduction_percent << "%" << std::endl;
    }
   
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
        if (use_components) {
            TerraScape::triangulateTerrainVolumeWithComponents(terrain, mesh);
        } else if (use_legacy) {
            TerraScape::triangulateTerrainVolumeLegacy(terrain, mesh);
        } else if (surface_only) {
            TerraScape::SimplificationParams params;
            params.error_threshold = error_threshold;
            params.min_triangle_reduction = reduction_percent;
            TerraScape::triangulateTerrainSurfaceOnly(terrain, mesh, params);
        } else if (use_simplified) {
            TerraScape::SimplificationParams params;
            params.error_threshold = error_threshold;
            params.min_triangle_reduction = reduction_percent;
            TerraScape::triangulateTerrainVolumeSimplified(terrain, mesh, params);
        } else {
            // Default to component-based approach
            TerraScape::triangulateTerrainVolume(terrain, mesh);
        }
        
        std::cout << "Generated mesh: " << mesh.vertices.size() << " vertices, " 
                  << mesh.triangles.size() << " triangles" << std::endl;
        
        // Validate mesh properties
        TerraScape::MeshStats stats = TerraScape::validateMesh(mesh, terrain);
        std::cout << "Mesh validation:" << std::endl;
        std::cout << "  Volume: " << stats.volume << " (expected: " << stats.expected_volume << ")" << std::endl;
        std::cout << "  Surface area: " << stats.surface_area << " (expected: " << stats.expected_surface_area << ")" << std::endl;
        std::cout << "  Is manifold: " << (stats.is_manifold ? "yes" : "no") << std::endl;
        std::cout << "  CCW oriented: " << (stats.is_ccw_oriented ? "yes" : "no") << std::endl;
        
        // Perform manifold validation using @elalish/manifold library if requested
        if (manifold_validation) {
            std::cout << "\nManifold Library Validation:" << std::endl;
            TerraScape::ManifoldValidationInfo manifold_info = TerraScape::validateMeshWithManifold(mesh);
            std::cout << "  Vertices: " << manifold_info.num_vertices << std::endl;
            std::cout << "  Triangles: " << manifold_info.num_triangles << std::endl;
            std::cout << "  Is manifold: " << (manifold_info.is_manifold ? "yes" : "no") << std::endl;
            std::cout << "  Status: " << manifold_info.error_message << std::endl;
            
            // If there's a validation issue, report it
            if (!manifold_info.is_manifold) {
                std::cerr << "Warning: Mesh failed manifold library validation!" << std::endl;
                std::cerr << "  Issue: " << manifold_info.error_message << std::endl;
            }
        }
        
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
