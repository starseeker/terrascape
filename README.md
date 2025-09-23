# TerraScape - Terrain Mesh Generation Library

TerraScape is a modern C++ library for converting elevation grids to triangle meshes.

## Features

- **Header-only implementation**: Include TerraScape.hpp and you're ready to go
- **C99 Compatible Interface**: C interface for integration with existing C codebases
- **BRL-CAD DSP Integration**: Direct support for BRL-CAD's Displacement Map primitive
- **Volumetric mesh generation**: Creates complete 3D volume, not just surface
- **Manifold guarantee**: All edges shared by exactly 2 triangles
- **CCW orientation**: Consistent triangle winding for outward-facing normals
- **Multiple input formats**: PGM files and GDAL-supported formats
- **Validation tools**: Volume and surface area verification

## Usage

### C++ API
```cpp
#include "TerraScape.hpp"

// Read terrain data
TerraScape::TerrainData terrain;
TerraScape::readTerrainFile("terrain.pgm", terrain);

// Generate triangulated mesh
TerraScape::TerrainMesh mesh;
TerraScape::triangulateTerrainVolume(terrain, mesh);

// Validate and write output
TerraScape::MeshStats stats = TerraScape::validateMesh(mesh, terrain);
TerraScape::writeObjFile("output.obj", mesh);
```

### C API (for BRL-CAD integration)
```c
#include "terrascape_c.h"

// Create DSP data structure
terrascape_dsp_data_t dsp_data;
dsp_data.height_data = your_height_array;
dsp_data.width = width;
dsp_data.height = height;

// Generate mesh
terrascape_mesh_t *mesh = terrascape_mesh_create();
terrascape_triangulate_dsp_surface(&dsp_data, mesh, NULL);

// Use mesh data...
terrascape_mesh_free(mesh);
```

## BRL-CAD Integration

TerraScape provides a C99-compatible interface specifically designed for integration with BRL-CAD's DSP (Displacement Map) primitive. See [BRL-CAD_INTEGRATION.md](BRL-CAD_INTEGRATION.md) for detailed integration instructions.

## Building

### C++ Library (Default)
Requires C++17 and optionally GDAL:

```bash
mkdir build && cd build
cmake ..
make
```

### With BRL-CAD Integration
```bash
mkdir build && cd build
cmake -DBUILD_C_INTERFACE=ON ..
make

# Test the integration
./bin/test_c_interface
./bin/brlcad_integration_example
```

```bash
mkdir build && cd build
cmake ..
make
```

## Demo Application

The included demo reads terrain data and generates an OBJ file:

```bash
./terrascape_demo --input crater.pgm --output terrain.obj
```

