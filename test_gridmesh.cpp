#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include "bg_grid_mesh.h"

using namespace std;

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
        // Binary PGM - skip for now, focus on textual
        cerr << "Error: Binary PGM not supported yet" << endl;
        return false;
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
bool writeMeshToOBJ(const char* filename, const bg::MeshResult& mesh) {
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

int main(int argc, char** argv) {
    cout << "Grid to Mesh Test Application" << endl;
    
    const char* input_file = "crater.pgm";
    const char* output_file = "output_mesh.obj";
    float error_threshold = 10.0f;  // Reasonable error threshold
    int point_limit = 1000;         // Reasonable point limit
    
    if (argc > 1) {
        input_file = argv[1];
    }
    if (argc > 2) {
        output_file = argv[2];
    }
    if (argc > 3) {
        error_threshold = atof(argv[3]);
    }
    if (argc > 4) {
        point_limit = atoi(argv[4]);
    }
    
    cout << "Input: " << input_file << endl;
    cout << "Output: " << output_file << endl;
    cout << "Error threshold: " << error_threshold << endl;
    cout << "Point limit: " << point_limit << endl;
    cout << endl;
    
    // Read PGM file
    int width, height;
    vector<float> elevations;
    if (!readPGMToFloatArray(input_file, width, height, elevations)) {
        return 1;
    }
    
    // Generate mesh using bg_grid_mesh
    cout << "Generating mesh..." << endl;
    auto mesh = bg::grid_to_mesh(width, height, elevations.data(), error_threshold, point_limit);
    
    cout << "Mesh generation complete!" << endl;
    cout << "Final mesh: " << mesh.vertices.size() << " vertices, " << mesh.triangles.size() << " triangles" << endl;
    
    // Write to OBJ
    if (!writeMeshToOBJ(output_file, mesh)) {
        return 1;
    }
    
    cout << "Success!" << endl;
    return 0;
}