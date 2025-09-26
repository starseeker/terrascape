# TerraScape - Terrain Mesh Generation Library

TerraScape is a modern C++ library for converting elevation grids to triangle meshes.

## Features

- **Header-only implementation**: Include TerraScape.hpp and you're ready to go
- **Volumetric mesh generation**: Creates complete 3D volume, not just surface
- **Manifold guarantee**: All edges shared by exactly 2 triangles
- **CCW orientation**: Consistent triangle winding for outward-facing normals
- **Multiple input formats**: PGM files and GDAL-supported formats
- **Validation tools**: Volume and surface area verification

## Usage

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

## Building

Requires C++17 and GDAL:

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

