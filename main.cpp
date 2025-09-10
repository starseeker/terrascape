#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <cfloat>  // For HUGE_VAL
#include <vector>

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
    cout << "Processing crater.pgm and generating mesh..." << endl;
    
    const char* input_file = "crater.pgm";
    const char* output_file = "crater_mesh.obj";
    float error_threshold = 40.0f;  // Reasonable error threshold
    int point_limit = 1000;         // Reasonable point limit for demo
    bool volumetric = false;        // Generate volumetric mesh?
    float z_base = 0.0f;           // Base level for volumetric mesh
    
    // Simple command line parsing
    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        cout << "Usage: " << argv[0] << " [options] [input.pgm] [output.obj] [error_threshold] [point_limit]" << endl;
        cout << "Options:" << endl;
        cout << "  -v, --volumetric: Generate volumetric (closed manifold) mesh instead of surface mesh" << endl;
        cout << "  --base <z>: Set base level for volumetric mesh (default: 0.0)" << endl;
        cout << "  -h, --help: Show this help message" << endl;
        cout << "Arguments:" << endl;
        cout << "  input.pgm: Input PGM heightfield file (default: crater.pgm)" << endl;
        cout << "  output.obj: Output OBJ mesh file (default: crater_mesh.obj)" << endl;
        cout << "  error_threshold: Maximum error threshold (default: 40.0)" << endl;
        cout << "  point_limit: Maximum number of vertices (default: 1000)" << endl;
        return 0;
    }
    
    // Parse command line arguments
    int arg_idx = 1;
    while (arg_idx < argc) {
        if (strcmp(argv[arg_idx], "-v") == 0 || strcmp(argv[arg_idx], "--volumetric") == 0) {
            volumetric = true;
            if (strcmp(output_file, "crater_mesh.obj") == 0) {
                output_file = "crater_volumetric.obj"; // Change default output for volumetric
            }
        } else if (strcmp(argv[arg_idx], "--base") == 0 && arg_idx + 1 < argc) {
            z_base = atof(argv[++arg_idx]);
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
    
    cout << "Input: " << input_file << endl;
    cout << "Output: " << output_file << endl;
    cout << "Error threshold: " << error_threshold << endl;
    cout << "Point limit: " << point_limit << endl;
    cout << "Mesh type: " << (volumetric ? "Volumetric (manifold)" : "Surface only") << endl;
    if (volumetric) {
        cout << "Base level: " << z_base << endl;
    }
    cout << endl;
    
    // Read PGM file to get elevation data
    int width, height;
    vector<float> elevations;
    if (!readPGMToFloatArray(input_file, width, height, elevations)) {
        return 1;
    }
    
    // Generate mesh using the TerraScape API
    cout << "Generating " << (volumetric ? "volumetric" : "surface") 
         << " mesh using greedy refinement algorithm..." << endl;
    
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