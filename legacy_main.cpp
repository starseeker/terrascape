#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <cfloat>  // For HUGE_VAL

// Define compatibility with old-style headers
using namespace std;

#include "version.h"
#include "terra.h"

// Define the global variables that are declared as extern in terra.h
GreedySubdivision *mesh = nullptr;
Map *DEM = nullptr;
ImportMask *MASK = nullptr;

real error_threshold = 0.0;
int point_limit = 1000;
real height_scale = 1.0;
FileFormat output_format = OBJfile;
char *output_filename = nullptr;
char *script_filename = nullptr;

// Simple implementation of process_cmdline for demo purposes
void process_cmdline(int argc, char **argv)
{
    // Set default values for demo
    error_threshold = 40.0;  // Reasonable error threshold
    point_limit = 1000;      // Reasonable point limit
    height_scale = 1.0;      // No scaling
    
    // For now, just use default settings
    // In a full implementation, this would parse command line arguments
}

int goal_not_met()
{
    return mesh->maxError() > error_threshold &&
           mesh->pointCount() < point_limit;
}

int main(int argc, char **argv)
{
    cout << "Terra Terrascape Demo v" << terra_version_string << endl;
    cout << "Processing crater.pgm and generating mesh..." << endl;
    
    // Process command line (currently just sets defaults)
    process_cmdline(argc, argv);
    
    // Open and read the PGM file
    ifstream dem_file("crater.pgm");
    if (!dem_file) {
        cerr << "Error: Cannot open crater.pgm" << endl;
        return 1;
    }
    
    // Read the DEM data
    DEM = readPGM(dem_file);
    dem_file.close();
    
    if (!DEM) {
        cerr << "Error: Failed to read crater.pgm" << endl;
        return 1;
    }
    
    cout << "DEM loaded: " << DEM->width << "x" << DEM->height << " pixels" << endl;
    cout << "Height range: " << DEM->min << " to " << DEM->max << endl;
    
    // Create a simple mask for the greedy algorithm
    MASK = new ImportMask();
    MASK->width = DEM->width;
    MASK->height = DEM->height;
    
    // Test the optimized greedy mesh generation
    cout << "Creating GreedySubdivision mesh..." << endl;
    mesh = new GreedySubdivision(DEM);
    
    cout << "Initial mesh: " << mesh->pointCount() << " points, max error: " << mesh->maxError() << endl;
    
    // Run a few iterations of greedy insertion for testing
    cout << "Running greedy insertion (limited iterations for demo)..." << endl;
    int iterations = 0;
    while(goal_not_met() && iterations < 20) {  // More iterations to see it work
        if (!mesh->greedyInsert()) {
            cout << "Greedy insertion failed at iteration " << iterations << endl;
            break;
        }
        iterations++;
        if (iterations % 5 == 0) {  // Print every 5 iterations
            cout << "Iteration " << iterations << ": points=" << mesh->pointCount() 
                 << ", error=" << mesh->maxError() << endl;
        }
    }
    
    cout << "Greedy mesh generation completed!" << endl;
    cout << "Final: " << mesh->pointCount() << " points, max error: " << mesh->maxError() << endl;
    
    // Generate a simple OBJ output directly from the DEM data
    const char* obj_filename = "crater_mesh.obj";
    ofstream obj_file(obj_filename);
    if (!obj_file) {
        cerr << "Error: Cannot create output file " << obj_filename << endl;
        return 1;
    }
    
    // Output vertices for a simple grid
    int step = 10; // Sample every 10th point to keep output manageable
    int vertex_count = 0;
    for(int y = 0; y < DEM->height; y += step) {
        for(int x = 0; x < DEM->width; x += step) {
            real height = DEM->eval(x, y);
            obj_file << "v " << x << " " << y << " " << height << endl;
            vertex_count++;
        }
    }
    
    // Output simple triangle faces
    int width_verts = (DEM->width + step - 1) / step;
    for(int y = 0; y < (DEM->height / step) - 1; y++) {
        for(int x = 0; x < width_verts - 1; x++) {
            int v1 = y * width_verts + x + 1;        // OBJ uses 1-based indexing
            int v2 = v1 + 1;
            int v3 = (y + 1) * width_verts + x + 1;
            int v4 = v3 + 1;
            
            // Two triangles per quad
            obj_file << "f " << v1 << " " << v2 << " " << v3 << endl;
            obj_file << "f " << v2 << " " << v4 << " " << v3 << endl;
        }
    }
    
    obj_file.close();
    
    cout << "Simple OBJ file written successfully with " << vertex_count << " vertices." << endl;
    cout << "This is a basic grid representation of the terrain data." << endl;
    
    // Clean up the mesh
    delete mesh;
    delete MASK;
    delete DEM;
    
    return 0;
}