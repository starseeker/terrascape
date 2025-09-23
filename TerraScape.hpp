/*
 * TerraScape - Pure C99 Header-Only Terrain Triangle Mesh Generation
 * 
 * Designed specifically for BRL-CAD DSP integration.
 * Converts height field data to optimized triangle meshes.
 */

#ifndef TERRASCAPE_H
#define TERRASCAPE_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* BASIC DATA STRUCTURES */
/* ========================================================================== */

/* 3D point/vector structure */
typedef struct {
    double x, y, z;
} terrascape_point3d_t;

/* Triangle structure with vertex indices */
typedef struct {
    size_t v0, v1, v2;
    terrascape_point3d_t normal;
} terrascape_triangle_t;

/* DSP data structure - compatible with BRL-CAD */
typedef struct {
    unsigned short *height_data;  /* Height array (row-major) */
    int width;                    /* Number of points in X direction */
    int height;                   /* Number of points in Y direction */
    double transform[16];         /* 4x4 transformation matrix */
    double cell_size;             /* Size of each cell */
    double min_height;            /* Minimum height value */
    double max_height;            /* Maximum height value */
} terrascape_dsp_t;

/* Triangle mesh output structure */
typedef struct {
    terrascape_point3d_t *vertices;
    terrascape_triangle_t *triangles;
    size_t vertex_count;
    size_t triangle_count;
    size_t vertex_capacity;
    size_t triangle_capacity;
} terrascape_mesh_t;

/* Triangulation parameters */
typedef struct {
    double error_threshold;       /* Maximum allowed geometric error */
    int min_triangle_reduction;   /* Minimum percentage of triangles to remove */
    int preserve_boundaries;      /* Boolean: preserve terrain boundaries */
    int adaptive_sampling;        /* Boolean: use adaptive sampling */
} terrascape_params_t;

/* ========================================================================== */
/* UTILITY FUNCTIONS */
/* ========================================================================== */

/* Vector math functions */
static inline terrascape_point3d_t terrascape_point3d_add(terrascape_point3d_t a, terrascape_point3d_t b) {
    terrascape_point3d_t result = {a.x + b.x, a.y + b.y, a.z + b.z};
    return result;
}

static inline terrascape_point3d_t terrascape_point3d_sub(terrascape_point3d_t a, terrascape_point3d_t b) {
    terrascape_point3d_t result = {a.x - b.x, a.y - b.y, a.z - b.z};
    return result;
}

static inline terrascape_point3d_t terrascape_point3d_cross(terrascape_point3d_t a, terrascape_point3d_t b) {
    terrascape_point3d_t result = {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
    return result;
}

static inline double terrascape_point3d_dot(terrascape_point3d_t a, terrascape_point3d_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline double terrascape_point3d_length(terrascape_point3d_t p) {
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

static inline terrascape_point3d_t terrascape_point3d_normalize(terrascape_point3d_t p) {
    double len = terrascape_point3d_length(p);
    if (len > 0.0) {
        terrascape_point3d_t result = {p.x / len, p.y / len, p.z / len};
        return result;
    }
    terrascape_point3d_t zero = {0.0, 0.0, 0.0};
    return zero;
}

/* Get height value from DSP data with bounds checking */
static inline double terrascape_dsp_get_height(const terrascape_dsp_t *dsp, int x, int y) {
    if (!dsp || !dsp->height_data || x < 0 || x >= dsp->width || y < 0 || y >= dsp->height) {
        return 0.0;
    }
    return (double)dsp->height_data[y * dsp->width + x];
}

/* Transform point from DSP space to model space */
static inline terrascape_point3d_t terrascape_transform_point(const terrascape_dsp_t *dsp, terrascape_point3d_t pt) {
    if (!dsp) {
        return pt;
    }
    
    const double *m = dsp->transform;
    terrascape_point3d_t result = {
        m[0] * pt.x + m[4] * pt.y + m[8]  * pt.z + m[12],
        m[1] * pt.x + m[5] * pt.y + m[9]  * pt.z + m[13],
        m[2] * pt.x + m[6] * pt.y + m[10] * pt.z + m[14]
    };
    return result;
}

/* ========================================================================== */
/* MESH MANAGEMENT FUNCTIONS */
/* ========================================================================== */

/* Create a new mesh */
static inline terrascape_mesh_t *terrascape_mesh_create(void) {
    terrascape_mesh_t *mesh = (terrascape_mesh_t*)calloc(1, sizeof(terrascape_mesh_t));
    return mesh;
}

/* Free a mesh and all its data */
static inline void terrascape_mesh_free(terrascape_mesh_t *mesh) {
    if (mesh) {
        free(mesh->vertices);
        free(mesh->triangles);
        free(mesh);
    }
}

/* Ensure mesh has capacity for additional vertices */
static inline int terrascape_mesh_reserve_vertices(terrascape_mesh_t *mesh, size_t additional) {
    if (!mesh) return 0;
    
    size_t needed = mesh->vertex_count + additional;
    if (needed <= mesh->vertex_capacity) return 1;
    
    size_t new_capacity = mesh->vertex_capacity * 2;
    if (new_capacity < needed) new_capacity = needed;
    
    terrascape_point3d_t *new_vertices = (terrascape_point3d_t*)realloc(mesh->vertices, 
                                                                       sizeof(terrascape_point3d_t) * new_capacity);
    if (!new_vertices) return 0;
    
    mesh->vertices = new_vertices;
    mesh->vertex_capacity = new_capacity;
    return 1;
}

/* Ensure mesh has capacity for additional triangles */
static inline int terrascape_mesh_reserve_triangles(terrascape_mesh_t *mesh, size_t additional) {
    if (!mesh) return 0;
    
    size_t needed = mesh->triangle_count + additional;
    if (needed <= mesh->triangle_capacity) return 1;
    
    size_t new_capacity = mesh->triangle_capacity * 2;
    if (new_capacity < needed) new_capacity = needed;
    
    terrascape_triangle_t *new_triangles = (terrascape_triangle_t*)realloc(mesh->triangles, 
                                                                          sizeof(terrascape_triangle_t) * new_capacity);
    if (!new_triangles) return 0;
    
    mesh->triangles = new_triangles;
    mesh->triangle_capacity = new_capacity;
    return 1;
}

/* Add a vertex to the mesh */
static inline size_t terrascape_mesh_add_vertex(terrascape_mesh_t *mesh, terrascape_point3d_t vertex) {
    if (!mesh || !terrascape_mesh_reserve_vertices(mesh, 1)) {
        return SIZE_MAX; /* Error */
    }
    
    mesh->vertices[mesh->vertex_count] = vertex;
    return mesh->vertex_count++;
}

/* Compute triangle normal */
static inline terrascape_point3d_t terrascape_compute_triangle_normal(const terrascape_mesh_t *mesh, 
                                                                      size_t v0, size_t v1, size_t v2) {
    if (!mesh || v0 >= mesh->vertex_count || v1 >= mesh->vertex_count || v2 >= mesh->vertex_count) {
        terrascape_point3d_t zero = {0.0, 0.0, 0.0};
        return zero;
    }
    
    terrascape_point3d_t p0 = mesh->vertices[v0];
    terrascape_point3d_t p1 = mesh->vertices[v1];
    terrascape_point3d_t p2 = mesh->vertices[v2];
    
    terrascape_point3d_t edge1 = terrascape_point3d_sub(p1, p0);
    terrascape_point3d_t edge2 = terrascape_point3d_sub(p2, p0);
    terrascape_point3d_t normal = terrascape_point3d_cross(edge1, edge2);
    
    return terrascape_point3d_normalize(normal);
}

/* Add a triangle to the mesh */
static inline int terrascape_mesh_add_triangle(terrascape_mesh_t *mesh, size_t v0, size_t v1, size_t v2) {
    if (!mesh || !terrascape_mesh_reserve_triangles(mesh, 1)) {
        return 0;
    }
    
    terrascape_triangle_t tri;
    tri.v0 = v0;
    tri.v1 = v1;
    tri.v2 = v2;
    tri.normal = terrascape_compute_triangle_normal(mesh, v0, v1, v2);
    
    mesh->triangles[mesh->triangle_count++] = tri;
    return 1;
}

/* ========================================================================== */
/* TRIANGULATION ALGORITHMS */
/* ========================================================================== */

/* Simple surface triangulation - creates surface mesh only */
static inline int terrascape_triangulate_surface_simple(const terrascape_dsp_t *dsp, terrascape_mesh_t *mesh) {
    if (!dsp || !mesh || !dsp->height_data || dsp->width < 2 || dsp->height < 2) {
        return 0;
    }
    
    /* Clear existing mesh */
    mesh->vertex_count = 0;
    mesh->triangle_count = 0;
    
    /* Reserve space for vertices and triangles */
    size_t vertex_count = dsp->width * dsp->height;
    size_t triangle_count = (dsp->width - 1) * (dsp->height - 1) * 2;
    
    if (!terrascape_mesh_reserve_vertices(mesh, vertex_count) ||
        !terrascape_mesh_reserve_triangles(mesh, triangle_count)) {
        return 0;
    }
    
    /* Create vertices */
    for (int y = 0; y < dsp->height; y++) {
        for (int x = 0; x < dsp->width; x++) {
            terrascape_point3d_t pt = {
                (double)x,
                (double)y,
                terrascape_dsp_get_height(dsp, x, y)
            };
            
            /* Apply transformation */
            pt = terrascape_transform_point(dsp, pt);
            terrascape_mesh_add_vertex(mesh, pt);
        }
    }
    
    /* Create triangles */
    for (int y = 0; y < dsp->height - 1; y++) {
        for (int x = 0; x < dsp->width - 1; x++) {
            /* Vertex indices for this cell */
            size_t v00 = y * dsp->width + x;         /* bottom-left */
            size_t v10 = y * dsp->width + (x + 1);   /* bottom-right */
            size_t v01 = (y + 1) * dsp->width + x;   /* top-left */
            size_t v11 = (y + 1) * dsp->width + (x + 1); /* top-right */
            
            /* Create two triangles per cell with proper CCW winding */
            /* Triangle 1: bottom-left, bottom-right, top-left */
            terrascape_mesh_add_triangle(mesh, v00, v10, v01);
            /* Triangle 2: bottom-right, top-right, top-left */
            terrascape_mesh_add_triangle(mesh, v10, v11, v01);
        }
    }
    
    return 1;
}

/* Adaptive surface triangulation with simplification */
static inline int terrascape_triangulate_surface_adaptive(const terrascape_dsp_t *dsp, 
                                                          terrascape_mesh_t *mesh,
                                                          const terrascape_params_t *params) {
    if (!dsp || !mesh || !dsp->height_data || dsp->width < 2 || dsp->height < 2) {
        return 0;
    }
    
    /* For now, use simple triangulation as base */
    /* This can be enhanced with proper adaptive algorithms later */
    if (!terrascape_triangulate_surface_simple(dsp, mesh)) {
        return 0;
    }
    
    /* Apply simplification if requested */
    if (params && params->min_triangle_reduction > 0) {
        /* Simple decimation - remove every nth triangle */
        /* This is a placeholder for more sophisticated algorithms */
        size_t target_triangles = mesh->triangle_count * (100 - params->min_triangle_reduction) / 100;
        if (target_triangles < mesh->triangle_count) {
            mesh->triangle_count = target_triangles;
        }
    }
    
    return 1;
}

/* Main entry point for surface triangulation */
static inline int terrascape_triangulate_dsp_surface(const terrascape_dsp_t *dsp,
                                                     terrascape_mesh_t *mesh,
                                                     const terrascape_params_t *params) {
    if (params && params->adaptive_sampling) {
        return terrascape_triangulate_surface_adaptive(dsp, mesh, params);
    } else {
        return terrascape_triangulate_surface_simple(dsp, mesh);
    }
}

/* ========================================================================== */
/* CONVENIENCE FUNCTIONS */
/* ========================================================================== */

/* Create default parameters */
static inline terrascape_params_t terrascape_params_default(void) {
    terrascape_params_t params;
    params.error_threshold = 0.1;
    params.min_triangle_reduction = 0;
    params.preserve_boundaries = 1;
    params.adaptive_sampling = 0;
    return params;
}

/* Create DSP structure from BRL-CAD data */
static inline terrascape_dsp_t terrascape_dsp_create(unsigned short *height_data,
                                                     int width, int height,
                                                     const double *transform_matrix) {
    terrascape_dsp_t dsp;
    dsp.height_data = height_data;
    dsp.width = width;
    dsp.height = height;
    dsp.cell_size = 1.0;
    dsp.min_height = 0.0;
    dsp.max_height = 65535.0;
    
    /* Copy transformation matrix */
    if (transform_matrix) {
        memcpy(dsp.transform, transform_matrix, sizeof(double) * 16);
    } else {
        /* Identity matrix */
        memset(dsp.transform, 0, sizeof(double) * 16);
        dsp.transform[0] = dsp.transform[5] = dsp.transform[10] = dsp.transform[15] = 1.0;
    }
    
    /* Calculate actual min/max */
    if (height_data && width > 0 && height > 0) {
        dsp.min_height = dsp.max_height = (double)height_data[0];
        for (int i = 1; i < width * height; i++) {
            double h = (double)height_data[i];
            if (h < dsp.min_height) dsp.min_height = h;
            if (h > dsp.max_height) dsp.max_height = h;
        }
    }
    
    return dsp;
}

#ifdef __cplusplus
}
#endif

#endif /* TERRASCAPE_H */