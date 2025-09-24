# TerraScape - Terrain Mesh Generation Library

TerraScape is a modern C++ library for converting elevation grids to triangle meshes.

## Features

- **Header-only implementation**: Include TerraScape.hpp and you're ready to go
- **Volumetric mesh generation**: Creates complete 3D volume, not just surface
- **Manifold guarantee**: All edges shared by exactly 2 triangles
- **CCW orientation**: Consistent triangle winding for outward-facing normals
- **Multiple input formats**: PGM files and GDAL-supported formats
- **Validation tools**: Volume and surface area verification
- **Manifold validation**: Integration with @elalish/manifold library for strict manifold validation
- **Steiner point triangulation**: Enhanced triangulation with detria for improved triangle quality

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

Requires C++17 and CMake >= 3.18. The manifold library is included as a submodule:

```bash
git clone --recursive https://github.com/starseeker/terrascape.git
mkdir build && cd build
cmake ..
make
```

Optional: GDAL for additional terrain file format support.

## Demo Application

The included demo reads terrain data and generates an OBJ file:

```bash
# Basic usage
./terrascape_demo --input crater.pgm --output terrain.obj

# With manifold validation using @elalish/manifold library
./terrascape_demo --input crater.pgm --manifold-validation

# BRL-CAD integration test with manifold validation
./terrascape_demo --brlcad-test --manifold-validation
```

## Manifold Validation

TerraScape includes integration with the @elalish/manifold library to validate mesh manifoldness according to strict geometric criteria. Use the `--manifold-validation` flag to enable this validation. This is particularly useful when integrating with BRL-CAD or other systems that require geometrically valid manifold meshes.

