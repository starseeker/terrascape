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

## Coplanar Patch Surface Mesh Optimization

TerraScape now includes an experimental coplanar patch-based surface mesh optimization approach:

```bash
./terrascape_demo --planar-patches --coplanar-tolerance 0.1 --input crater.pgm --output optimized.obj
```

This new approach:
- Identifies coplanar regions of 9+ cells during surface mesh generation
- Grows patches until they hit boundaries or non-coplanar areas
- Uses high-quality Delaunay triangulation (detria) for large patches (25+ cells)
- Falls back to standard grid triangulation for smaller regions
- Can achieve ~50% reduction in triangle count while maintaining surface quality

Parameters:
- `--planar-patches`: Enable the new approach
- `--coplanar-tolerance`: Height tolerance for coplanarity detection (default: 0.01)

## Demo Application

The included demo reads terrain data and generates an OBJ file:

```bash
./terrascape_demo --input crater.pgm --output terrain.obj
```

Additional options:
- `--components`: Handle terrain islands separately (default)
- `--legacy`: Single connected mesh approach
- `--simplified`: Use Terra/Scape simplification
- `--surface-only`: Generate surface-only mesh
- `--planar-patches`: Use coplanar patch optimization

