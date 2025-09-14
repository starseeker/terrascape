#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <cfloat>  // For HUGE_VAL
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
    int point_limit = 10000;          // Higher default for better quality
    bool volumetric = true;           // Generate volumetric mesh by default
    float z_base = 0.0f;             // Base level for volumetric mesh
    
    // BRL-CAD tolerance parameters
    double abs_tolerance_mm = 0.1;
    double rel_tolerance = 0.01;
    double norm_tolerance_deg = 15.0;
    double volume_delta_pct = 10.0;
    
    // Simple command line parsing
    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        cout << "Usage: " << argv[0] << " [options] [input] [output.obj] [error_threshold] [point_limit]" << endl;
        cout << "Options:" << endl;
        cout << "  -s, --surface: Generate surface mesh instead of volumetric (closed manifold) mesh" << endl;
        cout << "  --base <z>: Set base level for volumetric mesh (default: 0.0)" << endl;
        cout << "  --abs-tolerance <mm>: Absolute tolerance in millimeters (default: 0.1)" << endl;
        cout << "  --rel-tolerance <frac>: Relative tolerance as fraction (default: 0.01)" << endl;
        cout << "  --norm-tolerance <deg>: Normal angle tolerance in degrees (default: 15.0)" << endl;
        cout << "  --volume-delta <pct>: Max volume delta percentage (default: 10.0)" << endl;
        cout << "  -h, --help: Show this help message" << endl;
        cout << "Arguments:" << endl;
        cout << "  input: Input heightfield file (.pgm format, default: auto-detect)" << endl;
        cout << "  output.obj: Output OBJ mesh file (default: terrain_mesh.obj)" << endl;
        cout << "  error_threshold: Maximum error threshold (default: 1.0)" << endl;
        cout << "  point_limit: Maximum number of vertices (default: 10000)" << endl;
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
        } else if (strcmp(argv[arg_idx], "--norm-tolerance") == 0 && arg_idx + 1 < argc) {
            norm_tolerance_deg = atof(argv[++arg_idx]);
        } else if (strcmp(argv[arg_idx], "--volume-delta") == 0 && arg_idx + 1 < argc) {
            volume_delta_pct = atof(argv[++arg_idx]);
        } else {
            // Non-option argument - handle positionally
            static int pos_arg = 0;
            if (pos_arg == 0) {
                input_file = argv[arg_idx];
            } else if (pos_arg == 1) {
                output_file = argv[arg_idx];
            } else if (pos_arg == 2) {
                error_threshold = atof(argv[arg_idx]);
            } else if (pos_arg == 3) {
                point_limit = atoi(argv[arg_idx]);
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
    cout << "Point limit: " << point_limit << endl;
    cout << "Mesh type: " << (volumetric ? "Volumetric (manifold) with adaptive refinement" : "Surface only") << endl;
    if (volumetric) {
        cout << "Base level: " << z_base << endl;
    }
    
    // Display BRL-CAD tolerance settings (currently for reference - direct greedy_cuts API needed for full control)
    cout << "\n=== BRL-CAD Tolerance Settings (Reference) ===" << endl;
    cout << "Absolute tolerance: " << abs_tolerance_mm << " mm" << endl;
    cout << "Relative tolerance: " << rel_tolerance << " (fraction)" << endl;
    cout << "Normal tolerance: " << norm_tolerance_deg << " degrees" << endl;
    cout << "Volume delta tolerance: " << volume_delta_pct << "%" << endl;
    cout << "Note: For full tolerance control, use greedy_cuts.hpp API directly" << endl;
    cout << "===============================================" << endl;
    cout << endl;
    
    // Read elevation file (supports PGM format)
    int width, height;
    vector<float> elevations;
    if (!readElevationFile(input_file, width, height, elevations)) {
        return 1;
    }
    
    // Generate mesh using the TerraScape API
    cout << "Generating " << (volumetric ? "volumetric" : "surface") 
         << " mesh using adaptive greedy refinement with localized error metrics..." << endl;
    
    TerraScape::MeshResult mesh;
    if (volumetric) {
        mesh = TerraScape::grid_to_mesh_volumetric(width, height, elevations.data(), 
                                                  z_base, error_threshold, point_limit);
    } else {
        mesh = TerraScape::grid_to_mesh(width, height, elevations.data(), 
                                       error_threshold, point_limit);
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