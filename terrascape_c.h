/*
 * TerraScape C Interface Header
 * 
 * This header provides C99-compatible declarations for the TerraScape
 * library functions that can be used from C code, particularly for 
 * BRL-CAD DSP integration.
 */

#ifndef TERRASCAPE_C_H
#define TERRASCAPE_C_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations for C compatibility */
typedef struct terrascape_mesh terrascape_mesh_t;

/* Simple C-compatible 3D point structure */
typedef struct {
    double x, y, z;
} terrascape_point3d_t;

/* Simple C-compatible triangle structure with vertex indices */
typedef struct {
    size_t v0, v1, v2;
    terrascape_point3d_t normal;
} terrascape_triangle_t;

/* C-compatible mesh structure for output */
struct terrascape_mesh {
    terrascape_point3d_t *vertices;
    terrascape_triangle_t *triangles;
    size_t vertex_count;
    size_t triangle_count;
    size_t vertex_capacity;
    size_t triangle_capacity;
};

/* DSP data interface structure - matches BRL-CAD DSP internal format */
typedef struct {
    unsigned short *height_data;  /* 2D height array as 1D, row-major */
    int width;                    /* Number of points in X direction (dsp_xcnt) */
    int height;                   /* Number of points in Y direction (dsp_ycnt) */
    double transform[16];         /* 4x4 transformation matrix (solid to model space) */
    double cell_size;             /* Size of each cell */
    double min_height;            /* Minimum height value */
    double max_height;            /* Maximum height value */
} terrascape_dsp_data_t;

/* Simplification parameters for C interface */
typedef struct {
    double error_threshold;       /* Maximum allowed geometric error */
    int min_triangle_reduction;   /* Minimum percentage of triangles to remove */
    int preserve_boundaries;      /* Boolean: preserve terrain boundaries */
} terrascape_params_t;

/* Function prototypes for C interface */

/**
 * Create a new mesh structure
 * Returns: pointer to allocated mesh, or NULL on failure
 */
terrascape_mesh_t *terrascape_mesh_create(void);

/**
 * Free a mesh structure and all its data
 */
void terrascape_mesh_free(terrascape_mesh_t *mesh);

/**
 * Generate a triangle mesh from DSP height data
 * 
 * @param dsp_data - DSP height data and metadata
 * @param mesh - output mesh structure (must be pre-allocated)
 * @param params - triangulation parameters (can be NULL for defaults)
 * @return 0 on success, non-zero on error
 */
int terrascape_triangulate_dsp(const terrascape_dsp_data_t *dsp_data,
                               terrascape_mesh_t *mesh,
                               const terrascape_params_t *params);

/**
 * Generate surface-only triangle mesh (no volume) from DSP data
 * This is optimized for BRL-CAD's rt_dsp_tess function
 * 
 * @param dsp_data - DSP height data and metadata  
 * @param mesh - output mesh structure (must be pre-allocated)
 * @param params - triangulation parameters (can be NULL for defaults)
 * @return 0 on success, non-zero on error
 */
int terrascape_triangulate_dsp_surface(const terrascape_dsp_data_t *dsp_data,
                                       terrascape_mesh_t *mesh,
                                       const terrascape_params_t *params);

/**
 * Helper function to get height value from DSP data
 * Handles bounds checking and coordinate mapping
 * 
 * @param dsp_data - DSP data structure
 * @param x - X coordinate (0 to width-1)
 * @param y - Y coordinate (0 to height-1) 
 * @return height value, or 0.0 if out of bounds
 */
double terrascape_dsp_get_height(const terrascape_dsp_data_t *dsp_data, int x, int y);

/**
 * Transform a point from DSP coordinate space to model space using the transform matrix
 * 
 * @param dsp_data - DSP data structure containing transform
 * @param dsp_point - input point in DSP coordinates
 * @param model_point - output point in model coordinates
 */
void terrascape_transform_point(const terrascape_dsp_data_t *dsp_data,
                               const terrascape_point3d_t *dsp_point,
                               terrascape_point3d_t *model_point);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* TERRASCAPE_C_H */