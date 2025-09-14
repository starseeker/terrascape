# Feature Detection and Graph-Based Optimization in TerraScape

This document describes the new feature detection and graph-based optimization capabilities added to TerraScape.

## Overview

TerraScape now includes advanced terrain analysis features that can improve mesh quality by:

1. **Feature Detection**: Identifying terrain features like ridges, valleys, and terrain breaks using gradient and curvature analysis
2. **Graph-Based Optimization**: Using graph algorithms (MST, min-cut) to guide mesh segmentation and respect terrain features

## Usage

### Command Line Interface

The new features can be enabled through command-line options:

```bash
# Enable feature detection
./terrascape_demo input.pgm output.obj --enable-features --surface 1.0

# Enable both feature detection and graph optimization with MST
./terrascape_demo input.pgm output.obj --enable-features --enable-graph-opt --use-mst --surface 1.0

# Configure feature detection parameters
./terrascape_demo input.pgm output.obj --enable-features --feature-threshold 0.3 --feature-penalty 5.0 --surface 1.0
```

### Command Line Options

- `--enable-features`: Enable terrain feature detection
- `--enable-graph-opt`: Enable graph-based optimization
- `--feature-penalty <weight>`: Edge weight penalty for crossing features (default: 10.0)
- `--feature-threshold <thresh>`: Threshold for strong features (default: 0.5)
- `--use-mst`: Use MST for region connectivity optimization
- `--use-mincut`: Use min-cut for boundary placement optimization

### Programmatic API

```cpp
#include "TerraScape.hpp"

// Configure options for advanced triangulation
TerraScape::RegionGrowingOptions opts;
opts.base_error_threshold = 1.0;

// Enable feature detection and graph optimization
opts.enable_feature_detection = true;
opts.enable_graph_optimization = true;
opts.use_mst_for_regions = true;
opts.feature_penalty_weight = 10.0;
opts.feature_threshold = 0.5;

// Generate mesh with advanced features
auto mesh = TerraScape::region_growing_triangulation_advanced(
    elevations.data(), width, height, nullptr, opts);
```

## Feature Detection Module

The feature detection module (`FeatureDetection.hpp`) provides:

### API Functions

```cpp
// Main API function - compute feature map from elevation grid
std::vector<std::vector<double>> compute_feature_map(
    const ElevationGrid& grid, 
    const FeatureDetectionOptions& opts = FeatureDetectionOptions{});

// Find strong features above threshold
std::vector<std::pair<int, int>> find_strong_features(
    const std::vector<std::vector<double>>& feature_map,
    double threshold = 0.5);
```

### Detection Methods

1. **Gradient-based**: Uses Sobel operators to detect elevation changes
2. **Curvature-based**: Uses Laplacian operators to detect terrain curvature
3. **Smoothing**: Optional Gaussian smoothing to reduce noise

## Graph Optimization Module

The graph optimization module (`lemon.hpp`) provides:

### Graph Structures

- `ListGraph`: Undirected graph representation
- `Edge`: Graph edge with weight and endpoints
- `UnionFind`: Efficient union-find for MST algorithms

### Algorithms

- `kruskalMST()`: Kruskal's minimum spanning tree algorithm
- `primMST()`: Prim's minimum spanning tree algorithm  
- `MaxFlow`: Ford-Fulkerson max-flow/min-cut implementation

### Terrain Graph Construction

```cpp
// Build terrain graph with feature-weighted edges
auto graph = Lemon::buildTerrainGraph(width, height, feature_map, penalty_weight);

// Compute MST for region connectivity
auto mst_edges = Lemon::kruskalMST(graph);
```

## Testing

The implementation includes comprehensive tests:

```bash
# Run feature detection and graph optimization tests
./unified_tests --features --verbose

# Run all tests including new features
./unified_tests --all --verbose
```

Test coverage includes:
- Feature detection on synthetic ridges and flat surfaces
- Graph algorithm correctness (MST, connectivity)
- Terrain graph construction with feature weights
- Integration testing with triangulation pipeline
- Differential testing (with/without features)

## Performance Characteristics

- **Feature Detection**: O(width × height) for gradient and curvature computation
- **Graph Construction**: O(width × height) for terrain graph with 4-connectivity
- **MST Computation**: O(E log E) where E = edges ≈ 2×width×height
- **Memory Usage**: Additional ~8 bytes per grid cell for feature map

## Implementation Details

### Integration Points

The new features integrate into TerraScape at two levels:

1. **High-level API** (`grid_to_mesh_impl`): Automatic feature detection and optimization
2. **Advanced API** (`triangulateRegionGrowing`): Direct control over feature parameters

### Design Principles

- **Header-only**: Maintains TerraScape's header-only architecture
- **Opt-in**: New features are disabled by default, preserving existing behavior
- **Minimal dependencies**: Self-contained graph algorithms without external libraries
- **Backward compatible**: All existing APIs and functionality preserved

## Examples

### Example 1: Ridge Detection

```cpp
// Create terrain with a ridge
std::vector<float> elevations = {
    0, 0, 1, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 1, 0, 0
};

ElevationGrid grid(elevations.data(), 5, 5);
auto feature_map = FeatureDetection::compute_feature_map(grid);

// Feature map will show highest values along the ridge (x=2)
```

### Example 2: MST-Guided Segmentation

```cpp
RegionGrowingOptions opts;
opts.enable_feature_detection = true;
opts.enable_graph_optimization = true;
opts.use_mst_for_regions = true;

// The MST will guide region merging to respect terrain features
auto mesh = region_growing_triangulation_advanced(data, w, h, nullptr, opts);
```

## Future Enhancements

Potential areas for future development:

1. **Additional Algorithms**: Implement min-cut for boundary optimization
2. **Feature Types**: Classify features as ridges, valleys, or breaks
3. **Adaptive Parameters**: Automatically adjust detection thresholds
4. **Multi-scale Analysis**: Detect features at multiple resolutions
5. **Parallel Processing**: Optimize for large terrain datasets