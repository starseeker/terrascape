# TerraScape BRL-CAD Integration

This document describes how to integrate TerraScape's terrain triangulation capabilities into BRL-CAD's DSP (Displacement Map) primitive.

## Overview

The TerraScape library has been enhanced with a C99-compatible interface that can be used directly from BRL-CAD's C code. This allows the existing `rt_dsp_tess` function to be enhanced with TerraScape's advanced triangulation algorithms while maintaining compatibility with BRL-CAD's NMG (N-Manifold Geometry) output format.

## Files Added

### Core C Interface
- `terrascape_c.h` - C99-compatible header with function declarations
- `terrascape_c_interface.cpp` - Implementation of C interface functions
- `TerraScape.hpp` - Enhanced with C interface declarations and helper functions

### Examples and Tests
- `test_c_interface.c` - Basic test of the C interface
- `brlcad_integration_example.c` - Example showing BRL-CAD integration
- `CMakeLists.txt` - Updated to build C interface library

## C Interface API

### Data Structures

```c
typedef struct {
    unsigned short *height_data;  /* 2D height array as 1D, row-major */
    int width;                    /* Number of points in X direction */
    int height;                   /* Number of points in Y direction */
    double transform[16];         /* 4x4 transformation matrix */
    double cell_size;             /* Size of each cell */
    double min_height;            /* Minimum height value */
    double max_height;            /* Maximum height value */
} terrascape_dsp_data_t;

typedef struct {
    terrascape_point3d_t *vertices;
    terrascape_triangle_t *triangles;
    size_t vertex_count;
    size_t triangle_count;
    size_t vertex_capacity;
    size_t triangle_capacity;
} terrascape_mesh_t;
```

### Key Functions

```c
/* Create and manage mesh structures */
terrascape_mesh_t *terrascape_mesh_create(void);
void terrascape_mesh_free(terrascape_mesh_t *mesh);

/* Generate triangle meshes from DSP data */
int terrascape_triangulate_dsp_surface(const terrascape_dsp_data_t *dsp_data,
                                       terrascape_mesh_t *mesh,
                                       const terrascape_params_t *params);

/* Utility functions */
double terrascape_dsp_get_height(const terrascape_dsp_data_t *dsp_data, int x, int y);
void terrascape_transform_point(const terrascape_dsp_data_t *dsp_data,
                               const terrascape_point3d_t *dsp_point,
                               terrascape_point3d_t *model_point);
```

## Integration with BRL-CAD

### Modifying rt_dsp_tess

The `rt_dsp_tess` function in `src/librt/primitives/dsp/dsp.c` can be enhanced as follows:

1. **Convert DSP data format**:
   ```c
   terrascape_dsp_data_t ts_dsp;
   ts_dsp.height_data = dsp_ip->dsp_buf;
   ts_dsp.width = dsp_ip->dsp_xcnt;
   ts_dsp.height = dsp_ip->dsp_ycnt;
   /* Copy transformation matrix */
   for (int i = 0; i < 16; i++) {
       ts_dsp.transform[i] = dsp_ip->dsp_stom[i];
   }
   ```

2. **Generate triangle mesh**:
   ```c
   terrascape_mesh_t *mesh = terrascape_mesh_create();
   terrascape_params_t params;
   params.error_threshold = ttol->rel * 0.1;  /* Scale based on tolerance */
   params.min_triangle_reduction = 30;
   params.preserve_boundaries = 1;
   
   int result = terrascape_triangulate_dsp_surface(&ts_dsp, mesh, &params);
   ```

3. **Convert to NMG format**:
   ```c
   /* Create NMG region and shell */
   *r = nmg_mrsv(m);
   s = BU_LIST_FIRST(shell, &(*r)->s_hd);
   
   /* Create faces from triangles */
   for (size_t i = 0; i < mesh->triangle_count; i++) {
       struct vertex *verts[3];
       /* Create vertices and face */
       fu = nmg_cmface(s, verts, 3);
       nmg_fu_planeeqn(fu, tol);
   }
   ```

### Optional: Converting dsp.c to C++

If converting `dsp.c` to C++ is acceptable, the integration becomes simpler:

1. Rename `dsp.c` to `dsp.cpp`
2. Include `TerraScape.hpp` directly
3. Use the C++ API for more efficient integration

## Build Integration

### CMake Integration
Add to BRL-CAD's CMakeLists.txt:

```cmake
# Add TerraScape as a subdirectory or external project
add_subdirectory(path/to/terrascape)

# Link TerraScape C interface to librt
target_link_libraries(librt terrascape_c)
```

### Manual Integration
1. Copy `terrascape_c.h` and `terrascape_c_interface.cpp` to BRL-CAD source tree
2. Add compilation rules for the C++ interface file
3. Link the object file with librt

## Advantages

1. **Improved Quality**: TerraScape's adaptive triangulation creates better quality meshes
2. **Reduced Triangle Count**: Smart simplification reduces triangle count while preserving important features
3. **Better Performance**: Optimized algorithms for large datasets
4. **C99 Compatible**: No need to convert existing BRL-CAD C code to C++
5. **Backward Compatible**: Existing DSP functionality remains unchanged

## Testing

Run the provided examples to verify the integration:

```bash
cd build
make
./bin/test_c_interface           # Test basic C interface
./bin/brlcad_integration_example # Test BRL-CAD integration pattern
```

## Dependencies

- C++17 compiler for the implementation
- C99 compiler for the interface (standard with BRL-CAD)
- Optional: GDAL for enhanced terrain file support

## Future Enhancements

1. **Adaptive LOD**: Support for level-of-detail based on viewing distance
2. **Feature Preservation**: Better detection and preservation of terrain features
3. **Memory Optimization**: Streaming support for very large datasets
4. **Custom Cutting Patterns**: Support for BRL-CAD's existing cut direction options