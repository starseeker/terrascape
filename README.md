# TerraScape - Terrain Mesh Generation Library

TerraScape is a modern C++ library for converting elevation grids to optimized triangle meshes using region-growing triangulation with error-driven refinement. It builds upon the algorithmic foundations from the original Terra and Scape terrain simplification software by Michael Garland and Paul Heckbert.

## Key Features

- **Region-Growing Triangulation**: Grid-aware triangulation optimized for terrain data
- **Error-Driven Refinement**: Adaptive point selection based on approximation error
- **BRL-CAD Tolerance Integration**: Precision control with absolute, relative, and volume tolerance parameters  
- **Template-Based API**: Works with float, double, or any numeric elevation type
- **Robust Preprocessing**: Handles degenerate cases and numerical precision issues
- **Modern C++17**: Clean implementation with STL containers
- **Cross-Platform**: Builds on Windows, Linux, and macOS

## Quick Start

### Building

```bash
mkdir build
cd build
cmake ..
make -j4
```

### Requirements

- CMake 3.12+
- C++17 compatible compiler
- Standard C++ library

### Basic Usage

```cpp
#include "TerraScape.hpp"
#include <vector>

// Load elevation data (width x height grid)
std::vector<float> elevations = loadHeightField("terrain.pgm");
int width = 512, height = 512;

// Generate mesh with error threshold
auto mesh = TerraScape::grid_to_mesh(
    width, height, elevations.data(),
    1.0f    // error threshold
);

// mesh.vertices contains 3D points
// mesh.triangles contains triangle indices
```

### Demo Applications

```bash
# Main demo application
./build/bin/terrascape_demo input.pgm output.obj

# Comprehensive test suite  
./build/bin/unified_tests
```

## API Reference

### Core Data Structures

```cpp
namespace TerraScape {
    struct Vertex { float x, y, z; };
    struct Triangle { int v0, v1, v2; };
    struct MeshResult {
        std::vector<Vertex> vertices;
        std::vector<Triangle> triangles;
    };
}
```

### Main Function

```cpp
template<typename T>
MeshResult grid_to_mesh(
    int width, int height, const T* elevations,
    float error_threshold = 1.0f
);
```

**Parameters:**
- `width, height`: Grid dimensions
- `elevations`: Row-major elevation array (elevation[y * width + x])
- `error_threshold`: Maximum allowed approximation error

### Advanced API with Tolerance Control

```cpp
// For BRL-CAD integration and advanced control
TerraScape::RegionGrowingOptions opts;
opts.abs_tolerance_mm = 0.1;      // 0.1mm absolute tolerance
opts.rel_tolerance = 0.01;        // 1% relative tolerance
opts.mesh_density = 0.5;          // 0.0 = coarsest, 1.0 = finest

auto mesh = TerraScape::region_growing_triangulation_advanced(
    elevations.data(), width, height, nullptr, opts
);
```

## Algorithm Overview

TerraScape uses a region-growing approach that:

1. **Feature Detection**: Identifies terrain features and selects important vertices
2. **Grid Triangulation**: Creates triangles using systematic grid-based patterns
3. **Error-Driven Refinement**: Adds vertices where they most reduce approximation error
4. **Quality Control**: Ensures triangle quality through area and aspect ratio constraints

The implementation preserves the core algorithmic insights from the original papers while providing robust geometric computation for modern applications.

## Tolerance Integration

The library includes precision control compatible with BRL-CAD standards:

- **Absolute Tolerance**: Minimum geometric difference in millimeters
- **Relative Tolerance**: Minimum relative difference as fraction  
- **Volume Validation**: Compares mesh volume against theoretical cell volume
- **Normal Tolerance**: Maximum normal angle variation in degrees

## File Format Support

### Input: PGM (Portable Gray Map)
```
P2
width height
maxval
elevation1 elevation2 ... elevationN
```

### Output: OBJ (Wavefront)
```
v x1 y1 z1
v x2 y2 z2
...
f v1 v2 v3
f v4 v5 v6
...
```

## Testing

Run the comprehensive test suite:

```bash
cd build
./bin/unified_tests
```

The tests validate correctness, performance, and robustness across various terrain types and edge cases.

## Core Concepts from Original Papers

### From Scape (1995)
- **Point Insertion**: Select points that most reduce approximation error  
- **Triangular Mesh Representation**: Efficient storage and processing of terrain
- **Error Metrics**: Quantitative evaluation of terrain approximation quality

### From Terra (1996)
- **Data-Dependent Triangulation**: Adapt mesh structure to terrain features
- **Incremental Algorithm**: Build mesh progressively rather than all-at-once

TerraScape preserves these algorithmic foundations while adding modern robustness, tolerance controls, and geometric guarantees through region-growing triangulation.

## Performance Characteristics

- **Memory Usage**: O(N) for grid size N, with adaptive batching for large datasets
- **Processing Time**: Scales approximately linearly with input size
- **Quality**: Adaptive error thresholds maintain terrain detail while controlling mesh complexity

Example performance on different grid sizes:
- 50x50 grid (2.5K points): ~1ms
- 100x100 grid (10K points): ~10ms  
- 200x200 grid (40K points): ~100ms

## Version History

- **v0.7**: Consolidated API with region-growing triangulation and BRL-CAD tolerance integration
- **v0.6**: Robust geometric predicates and numerical stability improvements
- **v0.5**: Incremental error-driven mesh refinement
- **v0.4**: Original Terra/Scape algorithm foundation

## License

This software is in the **public domain** and is provided **AS IS**.
Use it at **YOUR OWN RISK**.

## References

- Garland, M. and Heckbert, P. "Fast Polygonal Approximation of Terrains and Height Fields" (CMU-CS-95-181, 1995)
- Original Terra software: http://www.cs.cmu.edu/~garland/scape/

## Contributing

This is research software primarily intended for algorithm development and evaluation. Contributions that improve robustness, performance, or add new features are welcome.