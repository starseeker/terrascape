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
    
    // Create the greedy subdivision mesh
    cout << "Creating GreedySubdivision..." << endl;
    mesh = new GreedySubdivision(DEM);
    cout << "GreedySubdivision created successfully." << endl;
    
    // Create a default mask (no masking)
    MASK = new ImportMask();
    
    // Run the greedy insertion algorithm
    cout << "Running greedy mesh generation..." << endl;
    greedy_insertion();
    
    cout << "Mesh generation complete." << endl;
    cout << "Final mesh has " << mesh->pointCount() << " points" << endl;
    cout << "Maximum error: " << mesh->maxError() << endl;
    
    // Generate OBJ output
    const char* obj_filename = "crater_mesh.obj";
    cout << "Generating OBJ output: " << obj_filename << endl;
    
    ofstream obj_file(obj_filename);
    if (!obj_file) {
        cerr << "Error: Cannot create output file " << obj_filename << endl;
        return 1;
    }
    
    output_obj(obj_file);
    obj_file.close();
    
    cout << "OBJ file written successfully." << endl;
    
    // Clean up
    delete mesh;
    delete DEM;
    delete MASK;
    
    return 0;
}