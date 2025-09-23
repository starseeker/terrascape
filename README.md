# TerraScape - Pure C99 Header-Only Terrain Triangulation

TerraScape is a pure C99 header-only library for converting height field data to optimized triangle meshes, specifically designed for BRL-CAD DSP integration.

## Features

- **Pure C99 header-only**: Single `TerraScape.hpp` file with no dependencies
- **BRL-CAD DSP compatible**: Direct integration with BRL-CAD's displacement map primitive
- **Optimized triangulation**: Adaptive sampling and simplification algorithms
- **Memory efficient**: Dynamic allocation with proper cleanup
- **Transform support**: Handle DSP-to-model coordinate transformations
- **Mathematical utilities**: Built-in vector math and geometry functions

## Usage

```c
#include "TerraScape.hpp"

// Create DSP data structure from BRL-CAD format
unsigned short *height_data = /* your height array */;
terrascape_dsp_t dsp = terrascape_dsp_create(height_data, width, height, transform_matrix);

// Generate optimized triangle mesh
terrascape_mesh_t *mesh = terrascape_mesh_create();
terrascape_params_t params = terrascape_params_default();
params.min_triangle_reduction = 30;  // 30% reduction

terrascape_triangulate_dsp_surface(&dsp, mesh, &params);

// Use mesh data (mesh->vertices, mesh->triangles)
printf("Generated %zu vertices, %zu triangles\n", 
       mesh->vertex_count, mesh->triangle_count);

// Clean up
terrascape_mesh_free(mesh);
```

## BRL-CAD Integration

TerraScape is designed for direct integration into BRL-CAD's `rt_dsp_tess` function:

```c
/* In BRL-CAD's dsp.c file */
#include "TerraScape.hpp"

int rt_dsp_tess(struct nmgregion **r, struct model *m, 
                struct rt_db_internal *ip, const struct bg_tess_tol *ttol, 
                const struct bn_tol *tol) {
    
    struct rt_dsp_internal *dsp_ip = (struct rt_dsp_internal *)ip->idb_ptr;
    
    /* Create TerraScape DSP structure */
    terrascape_dsp_t ts_dsp = terrascape_dsp_create(dsp_ip->dsp_buf, 
                                                    dsp_ip->dsp_xcnt, 
                                                    dsp_ip->dsp_ycnt, 
                                                    dsp_ip->dsp_stom);
    
    /* Generate optimized triangle mesh */
    terrascape_mesh_t *mesh = terrascape_mesh_create();
    terrascape_params_t params = terrascape_params_default();
    terrascape_triangulate_dsp_surface(&ts_dsp, mesh, &params);
    
    /* Convert to NMG format using existing BRL-CAD functions */
    *r = nmg_mrsv(m);
    struct shell *s = BU_LIST_FIRST(shell, &(*r)->s_hd);
    
    for (size_t i = 0; i < mesh->triangle_count; i++) {
        /* Create NMG faces from TerraScape triangles */
        /* ... use mesh->triangles[i] with nmg_cmface(), etc. */
    }
    
    terrascape_mesh_free(mesh);
    return 0;
}
```

## Building

The library is header-only, so no compilation is needed. For testing:

```bash
mkdir build && cd build
cmake ..
make

# Run tests
./bin/test_terrascape
./bin/brlcad_integration_demo
```

## API Reference

### Core Data Structures

- `terrascape_dsp_t` - Height field data compatible with BRL-CAD DSP format
- `terrascape_mesh_t` - Output triangle mesh with vertices and triangles
- `terrascape_params_t` - Triangulation parameters for quality control

### Key Functions

- `terrascape_dsp_create()` - Create DSP structure from height data
- `terrascape_mesh_create/free()` - Mesh memory management
- `terrascape_triangulate_dsp_surface()` - Main triangulation function
- `terrascape_params_default()` - Get default parameters

### Utility Functions

- Vector math: `terrascape_point3d_*()` functions
- Height access: `terrascape_dsp_get_height()`
- Coordinate transforms: `terrascape_transform_point()`

## Advantages for BRL-CAD

- **Drop-in compatibility**: Can be included directly in existing C code
- **No external dependencies**: Pure C99 with standard library only
- **Optimized output**: Reduces triangle count while preserving features
- **Memory safe**: Proper allocation/deallocation patterns
- **Maintainable**: Single header file, easy to integrate and update