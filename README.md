# TerraScape - Advanced Terrain Mesh Generation Library

TerraScape is a modern C++ library for converting elevation grids to optimized triangle meshes using advanced Delaunay triangulation and greedy refinement algorithms. It is the evolution of the original Terra and Scape terrain simplification software by Michael Garland and Paul Heckbert.

## Overview

TerraScape provides efficient grid-to-mesh conversion with robust geometric algorithms that preserve terrain features while generating optimal triangle quality. The library uses incremental Delaunay triangulation with error-driven point insertion to create high-quality terrain approximations.

## Key Features

- **Advanced Delaunay Triangulation**: Uses robust geometric predicates and half-edge topology management
- **Greedy Error-Driven Refinement**: Iteratively adds points where they reduce approximation error most
- **BRL-CAD Tolerance Integration**: Precision control with absolute, relative, and volume tolerance parameters
- **Multiple Refinement Strategies**: AUTO, HEAP, HYBRID, and SPARSE modes for different use cases
- **Simulation of Simplicity (SoS)**: Robust handling of geometric degeneracies with production-ready implementation
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

### Optional Dependencies

- **GDAL (Geospatial Data Abstraction Library)**: For processing real-world terrain data
  - Install: `sudo apt install libgdal-dev gdal-bin` (Ubuntu/Debian)
  - When available, enables:
    - Download and processing of real terrain data from Hawaii dataset
    - Conversion of BIL (Band Interleaved by Line) files to PGM format
    - Mesh validation against original terrain data
    - Advanced terrain data processing tools

- **libcurl**: For downloading terrain datasets (required when GDAL is enabled)
  - Install: `sudo apt install libcurl4-openssl-dev` (Ubuntu/Debian)

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
# Main demo application
./build/bin/terrascape_demo

# Comprehensive test suite
./build/bin/terrascape_tests

# Real terrain data processor (requires GDAL)
./build/bin/terrain_data_processor --help
```

### Working with Real Terrain Data

When GDAL is available, TerraScape can process real-world terrain data:

```bash
# Download Hawaii terrain data and process with TerraScape
./build/bin/terrain_data_processor

# Download terrain data only
./build/bin/terrain_data_processor --download-only

# Process existing terrain data with validation
./build/bin/terrain_data_processor --process-only --validate

# Run terrain data tests
make download_terrain_data  # Download Hawaii terrain files
ctest -R RealTerrainDataTest  # Test with real terrain data
```

The terrain data processor can:
- Download BIL files from the Hawaii 10-meter DEM dataset
- Convert BIL elevation data to PGM format using GDAL
- Generate optimized meshes from real terrain data
- Validate generated meshes against original elevation data
- Create sample terrain data when real data is unavailable

## BRL-CAD Tolerance Integration

TerraScape includes advanced tolerance control parameters for precision mesh generation, compatible with BRL-CAD standards:

### Tolerance Parameters

- **Absolute Tolerance** (`abs_tolerance_mm`): Minimum geometric difference in millimeters (default: 0.1)
- **Relative Tolerance** (`rel_tolerance`): Minimum relative difference as fraction (default: 0.01)  
- **Normal Tolerance** (`norm_tolerance_deg`): Maximum normal angle variation in degrees (default: 15.0)
- **Volume Delta** (`volume_delta_pct`): Maximum mesh/cell volume difference percentage (default: 10.0)

### Direct API Usage

```cpp
#include "greedy_cuts.hpp"

terrascape::GreedyCutsOptions opts;
opts.abs_tolerance_mm = 0.1;      // 0.1mm absolute tolerance
opts.rel_tolerance = 0.01;        // 1% relative tolerance  
opts.norm_tolerance_deg = 15.0;   // 15° normal tolerance
opts.volume_delta_pct = 10.0;     // 10% volume tolerance
opts.use_region_growing = true;

terrascape::Mesh mesh;
terrascape::triangulateGreedyCuts(elevations.data(), width, height, nullptr, opts, mesh);
```

### Command-Line Usage

```bash
# Test program with tolerance control
./test_region_growing --abs-tolerance 0.1 --rel-tolerance 0.01 --volume-delta 10.0

# Main demo with tolerance settings (parsing only)
./terrascape_demo --abs-tolerance 0.1 --rel-tolerance 0.01 --norm-tolerance 15.0 --volume-delta 10.0
```

### Testing Tolerance Integration

```bash
# Run tolerance-specific tests
ctest -R ToleranceTests

# Run automated tolerance test matrix
./test_tolerance_matrix.sh

# Individual test configurations
./test_region_growing --abs-tolerance 0.01 --rel-tolerance 0.001 --volume-delta 5.0  # Strict
./test_region_growing --abs-tolerance 1.0 --rel-tolerance 0.05 --volume-delta 50.0   # Relaxed
```

### Volume Validation

The system automatically validates mesh volume against theoretical cell volume:

```
WARNING: Volume delta exceeds tolerance (25.3% > 10.0%)
  Mesh volume: 1234.56, Cell volume: 1543.21
  Consider lowering tolerance thresholds for better accuracy
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
- **Grid-Aware Triangulation**: Custom advancing front algorithm optimized for gridded terrain data
- **Collinear Point Handling**: Natural handling of grid-inherent collinear points without degeneracy issues  
- **Structured Traversal**: Systematic triangulation leveraging grid structure for predictable performance
- **Geometric Robustness**: Built-in simulation of simplicity for handling degenerate cases

### Refinement Layer
- **Grid-Based Sampling**: Strategic point selection based on grid structure and error thresholds
- **Edge-First Advancement**: Boundary-to-interior advancement with quality control
- **Adaptive Density**: Automatic adjustment of triangulation density based on point limits
- **Triangle Quality Optimization**: Post-processing edge swapping for improved triangle quality

## Simulation of Simplicity (SoS)

TerraScape includes comprehensive support for Simulation of Simplicity to handle geometric degeneracies robustly.

### Features

- **SoS Geometric Predicates**: Eliminates exact collinear/cocircular cases through deterministic tie-breaking
- **Index-Based Tie-Breaking**: Uses point indices to simulate infinitesimal perturbations
- **Configurable**: Can be enabled per-triangulation instance
- **Production Ready**: Enabled by default for robust geometric computation

### Usage

```cpp
// Custom config with SoS enabled (default behavior)
template<typename Point, typename Idx>
struct SoSTriangulationConfig : public detria::DefaultTriangulationConfig<Point, Idx> {
    constexpr static bool UseSimulationOfSimplicity = true;
};

// Use SoS-enabled triangulation
detria::Triangulation<detria::PointD, uint32_t, SoSTriangulationConfig<detria::PointD, uint32_t>> tri;
```

### Status

**Production Features:**
- SoS framework fully implemented and tested
- Comprehensive test suite validates robustness and performance
- All triangulations handle collinear and cocircular points correctly
- Deterministic and consistent tie-breaking
- Performance validated on large datasets

**Performance Characteristics:**
- No significant overhead compared to non-SoS implementations
- Robust handling of pathological cases (many collinear/cocircular points)
- Scales well with input size
- Memory usage consistent across different refinement strategies

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
- **Improved Grid Triangulation**: Enhanced region-growing algorithm with adaptive neighborhood search for better handling of sparse vertex distributions in real-world terrain datasets

## Performance Characteristics

TerraScape has been optimized for handling real-world datasets efficiently:

### Strategy Selection (Automatic)
The AUTO strategy automatically selects the best algorithm based on dataset size:

- **Small Grids** (< 50K points): HEAP strategy for optimal mesh quality
- **Medium Grids** (50K - 200K points): HYBRID strategy balancing quality and speed
- **Large Grids** (> 200K points): SPARSE strategy for maximum performance

### Optimized Performance Features
- **Lazy Candidate Generation**: Reduces memory footprint by 75% for large datasets
- **Spatial Indexing**: 40x reduction in candidate updates per point insertion
- **Progressive Fallback**: Automatic batch size reduction when triangulation fails
- **Smart Sampling**: Adaptive sampling steps prevent memory explosion
- **Enhanced Grid Triangulation**: Improved region-growing with adaptive neighborhood search eliminates insufficient triangulation issues for real-world terrain datasets

### Grid Triangulation Improvements
TerraScape's region-growing algorithm now includes enhanced grid triangulation that:
- **Eliminates sparse triangulation issues** that previously required fallback mechanisms
- **Uses adaptive neighborhood search** instead of rigid 2x2 grid cells
- **Scales search radius** based on vertex sampling density
- **Provides triangle quality controls** with area and aspect ratio limits
- **Adapts triangle density limits** based on dataset size (conservative for large grids)

This improvement ensures reliable triangulation for diverse terrain types without relying on fallback algorithms.

### Benchmark Results
Example performance on 336x459 crater dataset (154K points):

- **SPARSE Strategy**: 0ms, 74 vertices, 142 triangles (recommended for large data)
- **HYBRID Strategy**: ~5.6 seconds, 644 vertices (optimized but more complex)

### Memory Usage
- **Before Optimization**: O(N²) behavior, 1500+ candidate updates per insertion
- **After Optimization**: O(N log N) behavior, 20-40 candidate updates per insertion
- **Memory Reduction**: From 154K to 16K initial candidates for large datasets

### Best Practices
1. **Use AUTO strategy** for automatic optimal selection
2. **For very large datasets (>500K points)**, consider pre-processing or tiling
3. **Adjust error thresholds** based on terrain complexity and quality requirements
4. **Monitor output quality** - SPARSE is fast but may miss fine details

Example performance on different grid sizes:
- 50x50 grid (2.5K points): ~1ms
- 100x100 grid (10K points): ~10ms  
- 200x200 grid (40K points): ~100ms
- 336x459 grid (154K points): ~1ms (SPARSE) / ~5.6s (HYBRID)

## File Format Support

### Input: PGM (Portable Gray Map)
```
P2
width height
maxval
elevation1 elevation2 ... elevationN
```

### Input: BIL (Band Interleaved by Line) - via GDAL
When GDAL is available, TerraScape can process BIL elevation files:
- Binary elevation data with accompanying header files
- Commonly used for geographic elevation datasets
- Automatically converted to PGM format for processing
- Supports various coordinate systems and projections

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

TerraScape includes a comprehensive test suite that validates:

- **Correctness**: Triangulation versioning, duplicate prevention, deterministic behavior
- **Performance**: Batch insertion efficiency, spatial acceleration
- **Robustness**: Triangle winding consistency, error threshold enforcement
- **SoS Functionality**: Geometric predicate tie-breaking, degeneracy handling
- **Integration**: Different refinement strategies, format support
- **Terrain Data Processing**: GDAL integration, real-world data handling

Run tests with:
```bash
cd build
ctest --verbose
# or run the consolidated test suite:
./bin/terrascape_tests
```

### Real Terrain Data Testing

When GDAL is available, additional tests validate real-world terrain processing:

```bash
# Test with sample terrain data
./bin/terrain_data_processor

# Run specific terrain data tests
ctest -R RealTerrainDataTest -V

# Download real Hawaii terrain data for testing
make download_terrain_data
```

## Known Issues and Limitations

1. **Memory Usage**: Large grids may require significant RAM for HEAP strategy
2. **Binary PGM**: Currently only textual PGM format supported
3. **Triangulation Scale**: Very large grids (>100k points) may hit performance limits in some cases

## Version History

- **v0.8**: Documentation consolidation, test suite unification, SoS integration
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
- Edelsbrunner, H. & Mücke, E.P. "Simulation of Simplicity: A Technique to Cope with Degenerate Cases in Geometric Algorithms"
- Shewchuk, J.R. "Robust Adaptive Floating-Point Geometric Predicates"
- Original Terra software: http://www.cs.cmu.edu/~garland/scape/

## Contributing

This is research software primarily intended for algorithm development and evaluation. However, contributions that improve robustness, performance, or add new features are welcome.