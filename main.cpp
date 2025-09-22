#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <cfloat>  // For HUGE_VAL
#include <cmath>   // For log10
#include <vector>
#include <filesystem>

// Define compatibility with old-style headers
using namespace std;

#include "version.h"
#include "TerraScape.hpp"  // Use new consolidated TerraScape header

// Simple PGM reader that extracts elevation data into a float array
bool readPGMToFloatArray(const char* filename, int& width, int& height, vector<float>& elevations) {
    ifstream file(filename);
    if (!file) {
        cerr << "Error: Cannot open " << filename << endl;
        return false;
    }
    
    char magicP, magicNum;
    int maxval;
    
    file >> magicP >> magicNum >> width >> height >> maxval;
    
    if (magicP != 'P' || (magicNum != '2' && magicNum != '5')) {
        cerr << "Error: Not a valid PGM file" << endl;
        return false;
    }
    
    elevations.resize(width * height);
    
    if (magicNum == '2') {
        // Textual PGM
        for (int i = 0; i < width * height; ++i) {
            float val;
            file >> val;
            elevations[i] = val;
        }
    } else {
        // Binary PGM - for simplicity, convert to textual read
        char newline;
        file.get(newline); // consume the newline after maxval
        for (int i = 0; i < width * height; ++i) {
            unsigned char val;
            file.read(reinterpret_cast<char*>(&val), 1);
            elevations[i] = float(val);
        }
    }
    
    cout << "Successfully read PGM: " << width << "x" << height << " pixels, maxval=" << maxval << endl;
    
    // Find min/max for info
    float min_val = elevations[0], max_val = elevations[0];
    for (float val : elevations) {
        if (val < min_val) min_val = val;
        if (val > max_val) max_val = val;
    }
    cout << "Elevation range: " << min_val << " to " << max_val << endl;
    
    return true;
}

// Detect file type based on extension and read accordingly
bool readElevationFile(const char* filename, int& width, int& height, vector<float>& elevations) {
    string fname(filename);
    string extension;
    
    // Extract file extension
    size_t dot_pos = fname.find_last_of('.');
    if (dot_pos != string::npos) {
        extension = fname.substr(dot_pos);
        // Convert to lowercase for comparison
        for (char& c : extension) {
            c = tolower(c);
        }
    }
    
    if (extension == ".pgm") {
        cout << "Detected PGM file format" << endl;
        return readPGMToFloatArray(filename, width, height, elevations);
    } else {
        // Try to auto-detect based on file content or fallback to PGM
        cout << "Unknown file extension '" << extension << "', trying PGM format..." << endl;
        return readPGMToFloatArray(filename, width, height, elevations);
    }
}

// Write mesh to OBJ format
bool writeMeshToOBJ(const char* filename, const TerraScape::MeshResult& mesh) {
    ofstream file(filename);
    if (!file) {
        cerr << "Error: Cannot create " << filename << endl;
        return false;
    }
    
    // Write vertices
    for (const auto& vertex : mesh.vertices) {
        file << "v " << vertex.x << " " << vertex.y << " " << vertex.z << endl;
    }
    
    // Write triangles (OBJ uses 1-based indexing)
    for (const auto& triangle : mesh.triangles) {
        file << "f " << (triangle.v0 + 1) << " " << (triangle.v1 + 1) << " " << (triangle.v2 + 1) << endl;
    }
    
    cout << "OBJ file written: " << mesh.vertices.size() << " vertices, " << mesh.triangles.size() << " triangles" << endl;
    return true;
}

int main(int argc, char **argv)
{
    cout << "Terrascape Demo v" << terra_version_string << " (using bg_grid_mesh API)" << endl;
    
    // Initialize with default values
    const char* input_file = nullptr;
    const char* output_file = "terrain_mesh.obj";
    float error_threshold = 1.0f;     // Lower default for more detail
    bool volumetric = true;           // Generate volumetric mesh by default
    float z_base = 0.0f;             // Base level for volumetric mesh
    
    // BRL-CAD tolerance parameters
    double abs_tolerance_mm = 0.1;
    double rel_tolerance = 0.01;
    double norm_tolerance_deg = 15.0;
    double volume_delta_pct = 10.0;
    
    // Mesh density control (can be set directly or calculated from tolerance)
    double mesh_density = -1.0;  // -1 means calculate from rel_tolerance
    
    // New feature detection and graph optimization options
    bool enable_feature_detection = false;
    bool enable_graph_optimization = false;
    double feature_penalty_weight = 10.0;
    double feature_threshold = 0.5;
    bool use_mst_for_regions = false;
    bool use_mincut_for_boundaries = false;
    
    // Simple command line parsing
    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        cout << "Usage: " << argv[0] << " [options] [input] [output.obj] [error_threshold]" << endl;
        cout << "Options:" << endl;
        cout << "  -s, --surface: Generate surface mesh instead of volumetric (closed manifold) mesh" << endl;
        cout << "  --base <z>: Set base level for volumetric mesh (default: 0.0)" << endl;
        cout << "  --abs-tolerance <mm>: Absolute tolerance in millimeters (default: 0.1)" << endl;
        cout << "  --rel-tolerance <frac>: Relative tolerance as fraction (default: 0.01)" << endl;
        cout << "                          Lower values create denser meshes, higher values create coarser meshes" << endl;
        cout << "  --mesh-density <val>: Direct mesh density control 0.0-1.0 (overrides rel-tolerance if set)" << endl;
        cout << "  --norm-tolerance <deg>: Normal angle tolerance in degrees (default: 15.0)" << endl;
        cout << "  --volume-delta <pct>: Max volume delta percentage (default: 10.0)" << endl;
        cout << "  --enable-features: Enable terrain feature detection" << endl;
        cout << "  --enable-graph-opt: Enable graph-based optimization" << endl;
        cout << "  --feature-penalty <weight>: Edge weight penalty for crossing features (default: 10.0)" << endl;
        cout << "  --feature-threshold <thresh>: Threshold for strong features (default: 0.5)" << endl;
        cout << "  --use-mst: Use MST for region connectivity optimization" << endl;
        cout << "  --use-mincut: Use min-cut for boundary placement optimization" << endl;
        cout << "  -h, --help: Show this help message" << endl;
        cout << "Arguments:" << endl;
        cout << "  input: Input heightfield file (.pgm format, default: auto-detect)" << endl;
        cout << "  output.obj: Output OBJ mesh file (default: terrain_mesh.obj)" << endl;
        cout << "  error_threshold: Maximum error threshold (default: 1.0)" << endl;
        cout << "Supported file formats:" << endl;
        cout << "  - PGM (.pgm): Portable Graymap format heightfields" << endl;
        cout << "Default behavior: Volumetric meshing with adaptive localized error metrics" << endl;
        return 0;
    }
    
    // Parse command line arguments
    int arg_idx = 1;
    while (arg_idx < argc) {
        if (strcmp(argv[arg_idx], "-s") == 0 || strcmp(argv[arg_idx], "--surface") == 0) {
            volumetric = false;
            if (strcmp(output_file, "terrain_mesh.obj") == 0) {
                output_file = "terrain_surface.obj"; // Change default output for surface
            }
        } else if (strcmp(argv[arg_idx], "--base") == 0 && arg_idx + 1 < argc) {
            z_base = atof(argv[++arg_idx]);
        } else if (strcmp(argv[arg_idx], "--abs-tolerance") == 0 && arg_idx + 1 < argc) {
            abs_tolerance_mm = atof(argv[++arg_idx]);
        } else if (strcmp(argv[arg_idx], "--rel-tolerance") == 0 && arg_idx + 1 < argc) {
            rel_tolerance = atof(argv[++arg_idx]);
        } else if (strcmp(argv[arg_idx], "--mesh-density") == 0 && arg_idx + 1 < argc) {
            mesh_density = atof(argv[++arg_idx]);
            if (mesh_density < 0.0 || mesh_density > 1.0) {
                cerr << "Error: mesh-density must be between 0.0 and 1.0" << endl;
                return 1;
            }
        } else if (strcmp(argv[arg_idx], "--norm-tolerance") == 0 && arg_idx + 1 < argc) {
            norm_tolerance_deg = atof(argv[++arg_idx]);
        } else if (strcmp(argv[arg_idx], "--volume-delta") == 0 && arg_idx + 1 < argc) {
            volume_delta_pct = atof(argv[++arg_idx]);
        } else if (strcmp(argv[arg_idx], "--enable-features") == 0) {
            enable_feature_detection = true;
        } else if (strcmp(argv[arg_idx], "--enable-graph-opt") == 0) {
            enable_graph_optimization = true;
        } else if (strcmp(argv[arg_idx], "--feature-penalty") == 0 && arg_idx + 1 < argc) {
            feature_penalty_weight = atof(argv[++arg_idx]);
        } else if (strcmp(argv[arg_idx], "--feature-threshold") == 0 && arg_idx + 1 < argc) {
            feature_threshold = atof(argv[++arg_idx]);
        } else if (strcmp(argv[arg_idx], "--use-mst") == 0) {
            use_mst_for_regions = true;
        } else if (strcmp(argv[arg_idx], "--use-mincut") == 0) {
            use_mincut_for_boundaries = true;
        } else {
            // Non-option argument - handle positionally
            static int pos_arg = 0;
            if (pos_arg == 0) {
                input_file = argv[arg_idx];
            } else if (pos_arg == 1) {
                output_file = argv[arg_idx];
            } else if (pos_arg == 2) {
                error_threshold = atof(argv[arg_idx]);
            }
            pos_arg++;
        }
        arg_idx++;
    }
    
    // If no input file was specified, auto-detect available input files
    if (input_file == nullptr) {
        if (filesystem::exists("crater.pgm")) {
            input_file = "crater.pgm";
            cout << "Found crater.pgm file - will use for processing" << endl;
        } else {
            input_file = "crater.pgm"; // Fallback default
            cout << "No input files found, will try crater.pgm" << endl;
        }
    }
    
    cout << "Input: " << input_file << endl;
    cout << "Output: " << output_file << endl;
    cout << "Error threshold: " << error_threshold << endl;
    cout << "Mesh type: " << (volumetric ? "Volumetric (manifold) with adaptive refinement" : "Surface only") << endl;
    if (volumetric) {
        cout << "Base level: " << z_base << endl;
    }
    
    // Display BRL-CAD tolerance settings (currently for reference - direct TerraScape API provides full control)
    cout << "\n=== BRL-CAD Tolerance Settings (Reference) ===" << endl;
    cout << "Absolute tolerance: " << abs_tolerance_mm << " mm" << endl;
    cout << "Relative tolerance: " << rel_tolerance << " (fraction)" << endl;
    cout << "Normal tolerance: " << norm_tolerance_deg << " degrees" << endl;
    cout << "Volume delta tolerance: " << volume_delta_pct << "%" << endl;
    cout << "Note: For full tolerance control, use TerraScape::grid_to_mesh API directly" << endl;
    cout << "===============================================" << endl;
    cout << endl;
    
    // Read elevation file (supports PGM format)
    int width, height;
    vector<float> elevations;
    if (!readElevationFile(input_file, width, height, elevations)) {
        return 1;
    }

    // Calculate mesh density from relative tolerance if not explicitly set
    if (mesh_density < 0.0) {
        // Map relative tolerance to mesh density with terrain-appropriate defaults
        // rel_tolerance: 0.001 (very fine) -> 0.1 (very coarse)
        // mesh_density: 1.0 (finest) -> 0.0 (coarsest)
        
        // Use logarithmic mapping to provide good sensitivity across the range
        double log_rel_tol = log10(rel_tolerance);
        double log_min = log10(0.001);  // -3.0
        double log_max = log10(0.1);    // -1.0
        
        // Clamp to reasonable range
        log_rel_tol = std::max(log_min, std::min(log_max, log_rel_tol));
        
        // Map to mesh density (inverted: lower tolerance = higher density)
        double base_density = 1.0 - (log_rel_tol - log_min) / (log_max - log_min);
        
        // For terrain data, ensure minimum density for adequate coverage
        // Boost density to ensure at least 70% terrain coverage
        mesh_density = std::max(0.7, base_density);
        
        cout << "Calculated mesh density: " << mesh_density << " from rel_tolerance: " << rel_tolerance << endl;
        if (mesh_density > base_density) {
            cout << "  (boosted from " << base_density << " to ensure adequate terrain coverage)" << endl;
        }
    } else {
        cout << "Using explicit mesh density: " << mesh_density << endl;
    }
    
    // Generate mesh using the TerraScape API
    cout << "Generating " << (volumetric ? "volumetric" : "surface") 
         << " mesh using adaptive error-driven refinement with localized error metrics..." << endl;
    
    TerraScape::MeshResult mesh;
    
    // Use advanced API if any of the new features are enabled
    if (enable_feature_detection || enable_graph_optimization || use_mst_for_regions || use_mincut_for_boundaries) {
        cout << "Using advanced TerraScape with feature detection and graph optimization..." << endl;
        
        // Configure options for advanced triangulation
        TerraScape::RegionGrowingOptions opts;
        opts.base_error_threshold = error_threshold;  // Set the error threshold
        opts.abs_tolerance_mm = abs_tolerance_mm;
        opts.rel_tolerance = rel_tolerance;
        opts.norm_tolerance_deg = norm_tolerance_deg;
        opts.volume_delta_pct = volume_delta_pct;
        opts.mesh_density = mesh_density;  // Apply calculated mesh density
        
        // Set feature detection and graph optimization options
        opts.enable_feature_detection = enable_feature_detection;
        opts.enable_graph_optimization = enable_graph_optimization;
        opts.feature_penalty_weight = feature_penalty_weight;
        opts.feature_threshold = feature_threshold;
        opts.use_mst_for_regions = use_mst_for_regions;
        opts.use_mincut_for_boundaries = use_mincut_for_boundaries;
        
        // Generate surface mesh with advanced features first
        TerraScape::MeshResult surface_mesh = TerraScape::region_growing_triangulation_advanced(
            elevations.data(), width, height, nullptr, opts);
        
        if (volumetric) {
            cout << "Converting surface mesh to volumetric mesh with base level: " << z_base << endl;
            mesh = TerraScape::make_volumetric_mesh(surface_mesh, z_base);
        } else {
            mesh = surface_mesh;
        }
    } else {
        // Use region-growing API for better mesh density control
        TerraScape::MeshResult surface_mesh = TerraScape::region_growing_triangulation(
            elevations.data(), width, height, mesh_density, nullptr);
        
        if (volumetric) {
            cout << "Converting surface mesh to volumetric mesh with base level: " << z_base << endl;
            mesh = TerraScape::make_volumetric_mesh(surface_mesh, z_base);
        } else {
            mesh = surface_mesh;
        }
    }
    
    cout << "Mesh generation complete!" << endl;
    cout << "Final mesh: " << mesh.vertices.size() << " vertices, " << mesh.triangles.size() << " triangles" << endl;
    cout << "Mesh type: " << (mesh.is_volumetric ? "Volumetric (closed manifold)" : "Surface mesh") << endl;
    
    // Write the resulting mesh to OBJ format
    if (!writeMeshToOBJ(output_file, mesh)) {
        return 1;
    }
    
    cout << "Success! Mesh written to " << output_file << endl;
    return 0;
}
