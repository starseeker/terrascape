# TerraScape - Advanced Terrain Mesh Generation Library

TerraScape is a modern C++ library for converting elevation grids to optimized triangle meshes using advanced Delaunay triangulation and greedy refinement algorithms. It is the evolution of the original Terra and Scape terrain simplification software by Michael Garland and Paul Heckbert.

## Overview

TerraScape provides efficient grid-to-mesh conversion with robust geometric algorithms that preserve terrain features while generating optimal triangle quality. The library uses incremental Delaunay triangulation with error-driven point insertion to create high-quality terrain approximations.

## Key Features

- **Advanced Delaunay Triangulation**: Uses robust geometric predicates and half-edge topology management
- **Greedy Error-Driven Refinement**: Iteratively adds points where they reduce approximation error most
- **Multiple Refinement Strategies**: AUTO, HEAP, HYBRID, and SPARSE modes for different use cases
- **Template-Based API**: Works with float, double, or any numeric elevation type
- **Modern C++**: Clean C++17 implementation with STL containers
- **Cross-Platform**: Builds on Windows, Linux, and macOS
- **PGM Format Support**: Standard portable graymap format for height field input

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

// Generate mesh with error threshold and point limit
auto mesh = TerraScape::grid_to_mesh(
    width, height, elevations.data(),
    1.0f,    // error threshold
    10000    // maximum points
);

// mesh.vertices contains 3D points
// mesh.triangles contains triangle indices
```

### Demo Applications

```bash
# Basic grid-to-mesh conversion
./build/bin/terrascape_demo

# Test different refinement strategies
./build/bin/test_detria_integration

# Demonstrate incremental insertion
./build/bin/test_incremental_demo

# Basic functionality test
./build/bin/test_gridmesh
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
    float error_threshold = 1.0f,
    int point_limit = 10000,
    MeshRefineStrategy strategy = MeshRefineStrategy::AUTO
);
```

**Parameters:**
- `width, height`: Grid dimensions
- `elevations`: Row-major elevation array (height[y * width + x])
- `error_threshold`: Maximum allowed approximation error
- `point_limit`: Maximum vertices in output mesh
- `strategy`: Refinement strategy (AUTO, HEAP, HYBRID, SPARSE)

### Refinement Strategies

- **AUTO**: Automatically selects best strategy based on grid size and available memory
- **HEAP**: Error-driven with priority queue (optimal quality, high memory usage)
- **HYBRID**: Balances quality and performance for medium-sized grids
- **SPARSE**: Fast regular sampling (lowest quality, fastest execution)

## Architecture

TerraScape uses a two-layer architecture:

### Triangulation Layer
- **Delaunay Triangulation**: Robust geometric predicates using bg_detria.hpp
- **Half-Edge Topology**: Efficient edge operations and triangle navigation
- **Geometric Robustness**: Handles degenerate cases and numerical precision issues

### Refinement Layer
- **Error-Driven Selection**: Candidates prioritized by approximation error
- **Incremental Insertion**: Points added one at a time with localized updates
- **Progressive Fallback**: Graceful handling of triangulation limits
- **Memory Management**: Automatic strategy selection based on available RAM

## Core Concepts from Original Papers

### From Scape (1995)
- **Greedy Point Insertion**: Select points that most reduce approximation error
- **Triangular Mesh Representation**: Efficient storage and processing of terrain
- **Error Metrics**: Quantitative evaluation of terrain approximation quality

### From Terra (1996)  
- **Data-Dependent Triangulation**: Adapt mesh structure to terrain features
- **Multiple Input Formats**: Support for standard height field formats
- **Incremental Algorithm**: Build mesh progressively rather than all-at-once

### New Concepts in TerraScape
- **Delaunay Constraint**: Ensure optimal triangle quality and avoid skinny triangles
- **Half-Edge Topology**: Modern data structure for efficient mesh operations
- **Robust Geometric Predicates**: Handle numerical precision and degenerate cases
- **Strategy Auto-Selection**: Automatically choose best algorithm based on problem size
- **Progressive Fallback**: Graceful degradation when hitting algorithmic limits
- **Template-Based Design**: Generic implementation supporting multiple data types

## Performance Characteristics

- **Small Grids** (< 500K points): HEAP strategy provides optimal results
- **Medium Grids** (500K - 5M points): HYBRID strategy balances quality and speed
- **Large Grids** (> 5M points): SPARSE strategy prevents memory exhaustion
- **Memory Usage**: Automatically estimated and factored into strategy selection

Example performance on 336x459 grid (154K points):
- 1000 vertices: ~6 seconds
- 2000 vertices: ~20 seconds  
- 431 vertices (error-limited): ~10 seconds

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

## Version History

- **v0.7**: TerraScape refactoring with consolidated API and improved naming
- **v0.6**: Detria integration with robust Delaunay triangulation
- **v0.5**: Incremental insertion implementation
- **v0.4**: Original Terra/Scape algorithms

## License

This software is in the **public domain** and is provided **AS IS**.
Use it at **YOUR OWN RISK**.

## Historical Context

TerraScape builds upon decades of research in terrain simplification:

- **Original Scape (1994)**: Fast algorithms for terrain approximation
- **Terra (1996)**: Improved structure and usability over Scape  
- **TerraScape (2024)**: Modern C++ with robust geometric algorithms

The core insight from the original papers remains valid: greedy insertion based on approximation error produces high-quality terrain meshes efficiently. TerraScape preserves this algorithmic foundation while adding modern robustness and geometric guarantees.

## References

- Garland, M. and Heckbert, P. "Fast Polygonal Approximation of Terrains and Height Fields" (CMU-CS-95-181, 1995)
- Original Terra software: http://www.cs.cmu.edu/~garland/scape/

## Contributing

This is research software primarily intended for algorithm development and evaluation. However, contributions that improve robustness, performance, or add new features are welcome.