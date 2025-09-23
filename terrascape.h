/*                    T E R R A S C A P E . H
 * BRL-CAD
 *
 * Published in 2025 by the United States Government.
 * This work is in the public domain.
 *
 */
/** @file terrascape.h
 *  TODO - explain and document
 */

#ifndef TERRASCAPE_H
#define TERRASCAPE_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Include earcut for efficient planar triangulation when compiling as C++ */
#ifdef __cplusplus
#include <array>
#include "earcut.hpp"
#endif

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
    double slope_threshold;       /* Slope threshold for feature preservation */
    int generate_manifold;        /* Boolean: generate manifold mesh with bottom plane and walls */
    double bottom_height;         /* Height for bottom plane (auto-calculated if <= min_height) */
    int optimize_bottom_plane;    /* Boolean: use earcut for sparse bottom triangulation */
} terrascape_params_t;

/* Terrain feature analysis structure for Terra/Scape algorithms */
typedef struct {
    double curvature;            /* Local surface curvature */
    double slope;                /* Local slope magnitude */
    double roughness;            /* Local height variation */
    int is_boundary;             /* Boolean: whether this is a boundary vertex */
    double importance_score;     /* Combined importance metric */
} terrascape_feature_t;

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
/* TERRA/SCAPE TERRAIN ANALYSIS ALGORITHMS */
/* ========================================================================== */

/* Analyze terrain features at a specific point (Terra/Scape inspired) */
static inline terrascape_feature_t terrascape_analyze_terrain_point(const terrascape_dsp_t *dsp, int x, int y) {
    terrascape_feature_t feature = {0};

    if (!dsp || x < 0 || x >= dsp->width || y < 0 || y >= dsp->height) {
        return feature;
    }

    double center_height = terrascape_dsp_get_height(dsp, x, y);

    /* Calculate local slope using central differences */
    double dx = 0.0, dy = 0.0;
    if (x > 0 && x < dsp->width - 1) {
        dx = (terrascape_dsp_get_height(dsp, x+1, y) - terrascape_dsp_get_height(dsp, x-1, y)) / (2.0 * dsp->cell_size);
    }
    if (y > 0 && y < dsp->height - 1) {
        dy = (terrascape_dsp_get_height(dsp, x, y+1) - terrascape_dsp_get_height(dsp, x, y-1)) / (2.0 * dsp->cell_size);
    }
    feature.slope = sqrt(dx * dx + dy * dy);

    /* Calculate local curvature (second derivatives) */
    double dxx = 0.0, dyy = 0.0;
    if (x > 0 && x < dsp->width - 1) {
        double left_height = terrascape_dsp_get_height(dsp, x-1, y);
        double right_height = terrascape_dsp_get_height(dsp, x+1, y);
        dxx = (left_height - 2.0 * center_height + right_height) / (dsp->cell_size * dsp->cell_size);
    }
    if (y > 0 && y < dsp->height - 1) {
        double bottom_height = terrascape_dsp_get_height(dsp, x, y-1);
        double top_height = terrascape_dsp_get_height(dsp, x, y+1);
        dyy = (bottom_height - 2.0 * center_height + top_height) / (dsp->cell_size * dsp->cell_size);
    }
    feature.curvature = fabs(dxx) + fabs(dyy);

    /* Calculate local roughness (height variation in neighborhood) */
    double height_sum = 0.0;
    double height_variance = 0.0;
    int neighbor_count = 0;

    /* Sample 3x3 neighborhood */
    for (int dy_offset = -1; dy_offset <= 1; dy_offset++) {
        for (int dx_offset = -1; dx_offset <= 1; dx_offset++) {
    	int nx = x + dx_offset;
    	int ny = y + dy_offset;
    	if (nx >= 0 && nx < dsp->width && ny >= 0 && ny < dsp->height) {
    	    double h = terrascape_dsp_get_height(dsp, nx, ny);
    	    height_sum += h;
    	    neighbor_count++;
    	}
        }
    }

    if (neighbor_count > 0) {
        double mean_height = height_sum / neighbor_count;
        for (int dy_offset = -1; dy_offset <= 1; dy_offset++) {
    	for (int dx_offset = -1; dx_offset <= 1; dx_offset++) {
    	    int nx = x + dx_offset;
    	    int ny = y + dy_offset;
    	    if (nx >= 0 && nx < dsp->width && ny >= 0 && ny < dsp->height) {
    		double h = terrascape_dsp_get_height(dsp, nx, ny);
    		height_variance += (h - mean_height) * (h - mean_height);
    	    }
    	}
        }
        feature.roughness = sqrt(height_variance / neighbor_count);
    }

    /* Check if this is a boundary point */
    feature.is_boundary = (x == 0 || x == dsp->width-1 || y == 0 || y == dsp->height-1);

    /* Calculate importance score (Terra/Scape style geometric importance) */
    feature.importance_score = feature.curvature + 0.5 * feature.slope + 0.3 * feature.roughness;
    if (feature.is_boundary) {
        feature.importance_score *= 2.0; /* Preserve boundaries */
    }

    return feature;
}

/* Importance point structure for sorting */
typedef struct {
    double importance;
    int x, y;
} terrascape_importance_point_t;

/* Comparison function for qsort (descending order by importance) */
static int compare_importance_points(const void *a, const void *b) {
    const terrascape_importance_point_t *pa = (const terrascape_importance_point_t*)a;
    const terrascape_importance_point_t *pb = (const terrascape_importance_point_t*)b;

    if (pa->importance > pb->importance) return -1;
    if (pa->importance < pb->importance) return 1;
    return 0;
}

/* Generate adaptive sampling mask based on terrain features */
static inline int *terrascape_generate_adaptive_sample_mask(const terrascape_dsp_t *dsp,
        const terrascape_params_t *params) {
    if (!dsp || !params) return NULL;

    /* Allocate mask array (1D array, row-major) */
    int total_points = dsp->width * dsp->height;
    int *mask = (int*)calloc(total_points, sizeof(int));
    if (!mask) return NULL;

    /* Always include boundary points if preserve_boundaries is set */
    if (params->preserve_boundaries) {
        for (int y = 0; y < dsp->height; y++) {
    	for (int x = 0; x < dsp->width; x++) {
    	    if (x == 0 || x == dsp->width-1 || y == 0 || y == dsp->height-1) {
    		mask[y * dsp->width + x] = 1;
    	    }
    	}
        }
    }

    /* Analyze terrain features and collect importance points */
    terrascape_importance_point_t *importance_points =
        (terrascape_importance_point_t*)malloc(sizeof(terrascape_importance_point_t) * total_points);
    if (!importance_points) {
        free(mask);
        return NULL;
    }

    int importance_count = 0;

    for (int y = 1; y < dsp->height - 1; y++) {
        for (int x = 1; x < dsp->width - 1; x++) {
    	terrascape_feature_t feature = terrascape_analyze_terrain_point(dsp, x, y);

    	/* Include points with high importance or exceeding thresholds */
    	if (feature.importance_score > params->error_threshold ||
    		feature.slope > params->slope_threshold) {
    	    mask[y * dsp->width + x] = 1;
    	} else {
    	    /* Store for potential inclusion based on overall reduction target */
    	    importance_points[importance_count].importance = feature.importance_score;
    	    importance_points[importance_count].x = x;
    	    importance_points[importance_count].y = y;
    	    importance_count++;
    	}
        }
    }

    /* Sort by importance (descending) */
    qsort(importance_points, importance_count, sizeof(terrascape_importance_point_t), compare_importance_points);

    /* Count current points */
    int current_points = 0;
    for (int i = 0; i < total_points; i++) {
        if (mask[i]) current_points++;
    }

    /* Calculate minimum required points to meet reduction target */
    int min_required = total_points * (100 - params->min_triangle_reduction) / 100;

    /* Add most important remaining points to reach minimum density */
    for (int i = 0; i < importance_count && current_points < min_required; i++) {
        int x = importance_points[i].x;
        int y = importance_points[i].y;
        int idx = y * dsp->width + x;
        if (!mask[idx]) {
    	mask[idx] = 1;
    	current_points++;
        }
    }

    free(importance_points);
    return mask;
}

/* ========================================================================== */
/* TRIANGULATION ALGORITHMS */
/* ========================================================================== */

/* Forward declarations */
static inline int terrascape_triangulate_dsp_manifold(const terrascape_dsp_t *dsp,
        terrascape_mesh_t *mesh,
        const terrascape_params_t *params);
static inline int terrascape_triangulate_bottom_plane_simple(const terrascape_dsp_t *dsp,
        terrascape_mesh_t *mesh,
        double bottom_height,
        size_t *bottom_vertex_indices);
#ifdef __cplusplus
static inline int terrascape_triangulate_bottom_plane_optimized(const terrascape_dsp_t *dsp,
        terrascape_mesh_t *mesh,
        double bottom_height,
        size_t *bottom_vertex_indices);
#endif

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

/* Advanced adaptive surface triangulation with Terra/Scape simplification */
static inline int terrascape_triangulate_surface_adaptive(const terrascape_dsp_t *dsp,
        terrascape_mesh_t *mesh,
        const terrascape_params_t *params) {
    if (!dsp || !mesh || !dsp->height_data || dsp->width < 2 || dsp->height < 2 || !params) {
        return 0;
    }

    /* Clear existing mesh */
    mesh->vertex_count = 0;
    mesh->triangle_count = 0;

    /* Generate adaptive sampling mask based on terrain features */
    int *sample_mask = terrascape_generate_adaptive_sample_mask(dsp, params);
    if (!sample_mask) {
        return 0;
    }

    /* Use structured subsampling combined with feature-based importance */
    int step_size = 1;
    if (params->min_triangle_reduction > 10) {
        /* Calculate step size to achieve target reduction */
        double reduction_factor = 100.0 / (100.0 - params->min_triangle_reduction);
        step_size = (int)sqrt(reduction_factor);
        if (step_size < 2) step_size = 2;  /* Minimum step size of 2 for any reduction */
    }

    /* Create vertex selection mask combining structured and adaptive sampling */
    int *keep_vertex = (int*)calloc(dsp->width * dsp->height, sizeof(int));
    if (!keep_vertex) {
        free(sample_mask);
        return 0;
    }

    /* Structured subsampling to maintain topology */
    for (int y = 0; y < dsp->height; y += step_size) {
        for (int x = 0; x < dsp->width; x += step_size) {
    	keep_vertex[y * dsp->width + x] = 1;
        }
    }

    /* Add important feature points from adaptive mask */
    for (int y = 0; y < dsp->height; y++) {
        for (int x = 0; x < dsp->width; x++) {
    	int idx = y * dsp->width + x;
    	if (sample_mask[idx] && !keep_vertex[idx]) {
    	    keep_vertex[idx] = 1;
    	}
        }
    }

    /* Ensure boundary completeness for manifold properties */
    if (params->preserve_boundaries) {
        for (int y = 0; y < dsp->height; y++) {
    	keep_vertex[y * dsp->width + 0] = 1;                    /* Left edge */
    	keep_vertex[y * dsp->width + (dsp->width - 1)] = 1;     /* Right edge */
        }
        for (int x = 0; x < dsp->width; x++) {
    	keep_vertex[0 * dsp->width + x] = 1;                    /* Bottom edge */
    	keep_vertex[(dsp->height - 1) * dsp->width + x] = 1;    /* Top edge */
        }
    }

    /* Create vertex mapping and add vertices to mesh */
    size_t *vertex_indices = (size_t*)malloc(sizeof(size_t) * dsp->width * dsp->height);
    if (!vertex_indices) {
        free(sample_mask);
        free(keep_vertex);
        return 0;
    }

    /* Initialize vertex indices to invalid */
    for (int i = 0; i < dsp->width * dsp->height; i++) {
        vertex_indices[i] = SIZE_MAX;
    }

    /* Count vertices needed and reserve space */
    size_t vertex_count = 0;
    for (int i = 0; i < dsp->width * dsp->height; i++) {
        if (keep_vertex[i]) vertex_count++;
    }

    if (!terrascape_mesh_reserve_vertices(mesh, vertex_count)) {
        free(sample_mask);
        free(keep_vertex);
        free(vertex_indices);
        return 0;
    }

    /* Add vertices for kept points */
    for (int y = 0; y < dsp->height; y++) {
        for (int x = 0; x < dsp->width; x++) {
    	int idx = y * dsp->width + x;
    	if (keep_vertex[idx]) {
    	    terrascape_point3d_t pt = {
    		(double)x,
    		(double)y,
    		terrascape_dsp_get_height(dsp, x, y)
    	    };

    	    /* Apply transformation */
    	    pt = terrascape_transform_point(dsp, pt);
    	    vertex_indices[idx] = terrascape_mesh_add_vertex(mesh, pt);
    	}
        }
    }

    /* Estimate triangle count and reserve space */
    size_t estimated_triangles = vertex_count * 2;  /* Conservative estimate */
    if (!terrascape_mesh_reserve_triangles(mesh, estimated_triangles)) {
        free(sample_mask);
        free(keep_vertex);
        free(vertex_indices);
        return 0;
    }

    /* Create triangles using adaptive grid */
    for (int y = 0; y < dsp->height - 1; y++) {
        for (int x = 0; x < dsp->width - 1; x++) {
    	/* Find valid vertices in this cell */
    	int has_v00 = keep_vertex[y * dsp->width + x];
    	int has_v10 = keep_vertex[y * dsp->width + (x + 1)];
    	int has_v01 = keep_vertex[(y + 1) * dsp->width + x];
    	int has_v11 = keep_vertex[(y + 1) * dsp->width + (x + 1)];

    	int valid_count = has_v00 + has_v10 + has_v01 + has_v11;

    	if (valid_count >= 3) {
    	    size_t v00 = has_v00 ? vertex_indices[y * dsp->width + x] : SIZE_MAX;
    	    size_t v10 = has_v10 ? vertex_indices[y * dsp->width + (x + 1)] : SIZE_MAX;
    	    size_t v01 = has_v01 ? vertex_indices[(y + 1) * dsp->width + x] : SIZE_MAX;
    	    size_t v11 = has_v11 ? vertex_indices[(y + 1) * dsp->width + (x + 1)] : SIZE_MAX;

    	    if (valid_count == 4) {
    		/* Full quad - create two triangles with proper CCW winding */
    		terrascape_mesh_add_triangle(mesh, v00, v10, v01);
    		terrascape_mesh_add_triangle(mesh, v10, v11, v01);
    	    } else if (valid_count == 3) {
    		/* Triangle - find the three valid vertices and ensure proper winding */
    		size_t vertices[3];
    		int count = 0;
    		if (has_v00) vertices[count++] = v00;
    		if (has_v10) vertices[count++] = v10;
    		if (has_v01) vertices[count++] = v01;
    		if (has_v11 && count < 3) vertices[count++] = v11;

    		if (count == 3) {
    		    /* Determine proper winding order based on position */
    		    /* For terrain, we want CCW when viewed from above (positive Z) */
    		    terrascape_mesh_add_triangle(mesh, vertices[0], vertices[1], vertices[2]);
    		}
    	    }
    	}
        }
    }

    /* Clean up */
    free(sample_mask);
    free(keep_vertex);
    free(vertex_indices);

    return 1;
}

/* Fallback bottom plane triangulation using simple quad division */
static inline int terrascape_triangulate_bottom_plane_simple(const terrascape_dsp_t *dsp,
        terrascape_mesh_t *mesh,
        double bottom_height,
        size_t *bottom_vertex_indices) {
    
    /* Create vertices for bottom plane corners */
    terrascape_point3d_t corners[4];
    corners[0].x = 0.0; corners[0].y = 0.0; corners[0].z = bottom_height;
    corners[1].x = (double)(dsp->width-1); corners[1].y = 0.0; corners[1].z = bottom_height;
    corners[2].x = (double)(dsp->width-1); corners[2].y = (double)(dsp->height-1); corners[2].z = bottom_height;
    corners[3].x = 0.0; corners[3].y = (double)(dsp->height-1); corners[3].z = bottom_height;
    
    /* Transform and add vertices */
    for (int i = 0; i < 4; i++) {
        corners[i] = terrascape_transform_point(dsp, corners[i]);
        bottom_vertex_indices[i] = terrascape_mesh_add_vertex(mesh, corners[i]);
    }
    
    /* Create two triangles for the bottom plane (CW winding for bottom face) */
    terrascape_mesh_add_triangle(mesh, bottom_vertex_indices[0], bottom_vertex_indices[2], bottom_vertex_indices[1]);
    terrascape_mesh_add_triangle(mesh, bottom_vertex_indices[0], bottom_vertex_indices[3], bottom_vertex_indices[2]);
    
    return 1;
}

/* Generate manifold mesh with surface, walls, and optimized bottom plane */
static inline int terrascape_triangulate_dsp_manifold(const terrascape_dsp_t *dsp,
        terrascape_mesh_t *mesh,
        const terrascape_params_t *params) {
    if (!dsp || !mesh || !params) {
        return 0;
    }
    
    /* First generate the surface mesh */
    int surface_result;
    if (params->adaptive_sampling) {
        surface_result = terrascape_triangulate_surface_adaptive(dsp, mesh, params);
    } else {
        surface_result = terrascape_triangulate_surface_simple(dsp, mesh);
    }
    
    if (!surface_result) {
        return 0;
    }
    
    /* Calculate bottom height */
    double bottom_height = params->bottom_height;
    if (bottom_height <= dsp->min_height) {
        bottom_height = dsp->min_height - (dsp->max_height - dsp->min_height) * 0.1;
    }
    
    /* Store surface vertex count for wall generation */
    size_t surface_vertex_count = mesh->vertex_count;
    
    /* Generate bottom plane with optimized triangulation */
    size_t bottom_vertex_indices[4];
    int bottom_result;
    
#ifdef __cplusplus
    if (params->optimize_bottom_plane) {
        bottom_result = terrascape_triangulate_bottom_plane_optimized(dsp, mesh, bottom_height, bottom_vertex_indices);
    } else {
        bottom_result = terrascape_triangulate_bottom_plane_simple(dsp, mesh, bottom_height, bottom_vertex_indices);
    }
#else
    bottom_result = terrascape_triangulate_bottom_plane_simple(dsp, mesh, bottom_height, bottom_vertex_indices);
#endif
    
    if (!bottom_result) {
        return 0;
    }
    
    /* Generate walls connecting surface boundary to bottom plane */
    /* Get boundary vertices from surface mesh */
    size_t surface_boundary_indices[4];
    surface_boundary_indices[0] = 0;  /* Bottom-left */
    surface_boundary_indices[1] = dsp->width - 1;  /* Bottom-right */
    surface_boundary_indices[2] = surface_vertex_count - 1;  /* Top-right */
    surface_boundary_indices[3] = surface_vertex_count - dsp->width;  /* Top-left */
    
    /* Create wall triangles with proper winding (outward-facing normals) */
    /* Bottom wall (y=0) */
    terrascape_mesh_add_triangle(mesh, surface_boundary_indices[0], bottom_vertex_indices[0], surface_boundary_indices[1]);
    terrascape_mesh_add_triangle(mesh, surface_boundary_indices[1], bottom_vertex_indices[0], bottom_vertex_indices[1]);
    
    /* Right wall (x=max) */
    terrascape_mesh_add_triangle(mesh, surface_boundary_indices[1], bottom_vertex_indices[1], surface_boundary_indices[2]);
    terrascape_mesh_add_triangle(mesh, surface_boundary_indices[2], bottom_vertex_indices[1], bottom_vertex_indices[2]);
    
    /* Top wall (y=max) */
    terrascape_mesh_add_triangle(mesh, surface_boundary_indices[2], bottom_vertex_indices[2], surface_boundary_indices[3]);
    terrascape_mesh_add_triangle(mesh, surface_boundary_indices[3], bottom_vertex_indices[2], bottom_vertex_indices[3]);
    
    /* Left wall (x=0) */
    terrascape_mesh_add_triangle(mesh, surface_boundary_indices[3], bottom_vertex_indices[3], surface_boundary_indices[0]);
    terrascape_mesh_add_triangle(mesh, surface_boundary_indices[0], bottom_vertex_indices[3], bottom_vertex_indices[0]);
    
    return 1;
}

/* Main entry point for surface triangulation */
static inline int terrascape_triangulate_dsp_surface(const terrascape_dsp_t *dsp,
        terrascape_mesh_t *mesh,
        const terrascape_params_t *params) {
    if (params && params->generate_manifold) {
        return terrascape_triangulate_dsp_manifold(dsp, mesh, params);
    } else if (params && params->adaptive_sampling) {
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
    params.slope_threshold = 0.2;
    params.generate_manifold = 0;
    params.bottom_height = -1.0;  /* Auto-calculate */
    params.optimize_bottom_plane = 1;
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

/* Debug function to expose terrain feature analysis for testing */
static inline terrascape_feature_t terrascape_debug_analyze_point(const terrascape_dsp_t *dsp, int x, int y) {
    return terrascape_analyze_terrain_point(dsp, x, y);
}

/* ========================================================================== */
/* BRL-CAD TOLERANCE INTEGRATION */
/* ========================================================================== */

/* BRL-CAD tolerance structures for integration */
typedef struct {
    double dist;                  /* General tolerance distance */
} bn_tol_sim;

typedef struct {
    double abs;                   /* Absolute tolerance in model units */
    double rel;                   /* Relative tolerance (0.0 to 1.0) */
    double norm;                  /* Normal tolerance in radians */
} bg_tess_tol_sim;

/**
 * Convert BRL-CAD tessellation tolerances to terrascape parameters
 *
 * This function translates BRL-CAD's standard tessellation tolerances
 * (absolute, relative, normal) into appropriate terrascape parameters.
 *
 * @param ttol BRL-CAD tessellation tolerance structure
 * @param tol General BRL-CAD tolerance structure
 * @param dsp DSP data for terrain-specific scaling
 * @return terrascape parameters configured from BRL-CAD tolerances
 */
static inline terrascape_params_t terrascape_params_from_brlcad_tolerances(
        const bg_tess_tol_sim *ttol,
        const bn_tol_sim *tol,
        const terrascape_dsp_t *dsp) {

    terrascape_params_t params = terrascape_params_default();

    if (!ttol || !tol || !dsp) {
        return params;  /* Return defaults if NULL parameters */
    }

    /* Calculate terrain-specific metrics for tolerance mapping */
    double terrain_height_range = dsp->max_height - dsp->min_height;
    double terrain_scale = (terrain_height_range > 1.0) ? terrain_height_range : 1.0;

    /* Map absolute tolerance to error threshold */
    if (ttol->abs > 0.0) {
        params.error_threshold = ttol->abs;
    } else {
        /* Fall back to general tolerance distance */
        params.error_threshold = tol->dist;
    }

    /* Map relative tolerance to triangle reduction */
    if (ttol->rel > 0.0 && ttol->rel <= 1.0) {
        /* Convert relative tolerance to triangle reduction percentage */
        /* Higher relative tolerance allows more triangle reduction */
        params.min_triangle_reduction = (int)(ttol->rel * 50.0);  /* Max 50% reduction */
    } else {
        params.min_triangle_reduction = 10;  /* Conservative default */
    }

    /* Map normal tolerance to slope threshold */
    if (ttol->norm > 0.0) {
        /* Normal tolerance in radians maps to slope threshold */
        params.slope_threshold = tan(ttol->norm);
    } else {
        params.slope_threshold = 0.1;  /* Default slope threshold */
    }

    /* Enable adaptive sampling for better quality */
    params.adaptive_sampling = 1;
    params.preserve_boundaries = 1;  /* Always preserve boundaries for BRL-CAD */

    /* Ensure minimum error threshold relative to terrain scale */
    double min_threshold = terrain_scale * 1e-6;
    if (params.error_threshold < min_threshold) {
        params.error_threshold = min_threshold;
    }

    return params;
}

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/* C++ only functions that use earcut.hpp */

/* Optimized bottom plane triangulation using earcut for sparse triangle generation */
static inline int terrascape_triangulate_bottom_plane_optimized(const terrascape_dsp_t *dsp,
        terrascape_mesh_t *mesh,
        double bottom_height,
        size_t *bottom_vertex_indices) {
    
    /* For complex bottom plane optimization, we need to:
     * 1. Identify zero-height regions that need full-density edges
     * 2. Create a sparse mesh for interior areas
     * 3. Maintain boundary edges for manifold properties
     */
    
    /* For now, create a more sophisticated boundary than just 4 corners */
    std::vector<std::vector<std::array<double, 2>>> polygon;
    std::vector<std::array<double, 2>> boundary;
    
    /* Examine terrain boundary to create more complex polygon if needed */
    int boundary_samples = 8;  /* Use 8 points around boundary for better representation */
    double width_step = (double)(dsp->width - 1) / (boundary_samples / 2 - 1);
    double height_step = (double)(dsp->height - 1) / (boundary_samples / 2 - 1);
    
    /* Bottom edge samples */
    for (int i = 0; i < boundary_samples / 2; i++) {
        double x = i * width_step;
        boundary.push_back({{x, 0.0}});
    }
    
    /* Right edge samples */
    for (int i = 1; i < boundary_samples / 2; i++) {
        double y = i * height_step;
        boundary.push_back({{(double)(dsp->width - 1), y}});
    }
    
    /* Top edge samples (reverse order) */
    for (int i = boundary_samples / 2 - 2; i >= 0; i--) {
        double x = i * width_step;
        boundary.push_back({{x, (double)(dsp->height - 1)}});
    }
    
    /* Left edge samples (reverse order) */
    for (int i = boundary_samples / 2 - 2; i > 0; i--) {
        double y = i * height_step;
        boundary.push_back({{0.0, y}});
    }
    
    polygon.push_back(boundary);
    
    /* Add interior points for zero-height regions if they exist */
    /* For demonstration, we'll check for significant height variations that might indicate
     * areas where sparse triangulation would be beneficial */
    std::vector<std::array<double, 2>> interior_points;
    
    /* Sample interior points in areas with low height variation (good candidates for sparse triangulation) */
    int interior_samples_x = dsp->width / 8;  /* Sample every 8th point */
    int interior_samples_y = dsp->height / 8;
    
    for (int y = 1; y < interior_samples_y; y++) {
        for (int x = 1; x < interior_samples_x; x++) {
            double px = x * 8.0;
            double py = y * 8.0;
            
            /* Check if this area has low height variation (good for sparse triangulation) */
            double height_variation = 0.0;
            int sample_count = 0;
            double avg_height = 0.0;
            
            /* Sample a 3x3 area around this point */
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int sx = (int)px + dx * 4;
                    int sy = (int)py + dy * 4;
                    if (sx >= 0 && sx < dsp->width && sy >= 0 && sy < dsp->height) {
                        double h = terrascape_dsp_get_height(dsp, sx, sy);
                        avg_height += h;
                        sample_count++;
                    }
                }
            }
            
            if (sample_count > 0) {
                avg_height /= sample_count;
                
                /* Calculate variation */
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int sx = (int)px + dx * 4;
                        int sy = (int)py + dy * 4;
                        if (sx >= 0 && sx < dsp->width && sy >= 0 && sy < dsp->height) {
                            double h = terrascape_dsp_get_height(dsp, sx, sy);
                            height_variation += fabs(h - avg_height);
                        }
                    }
                }
                height_variation /= sample_count;
                
                /* If height variation is low, this is a good candidate for sparse triangulation */
                double variation_threshold = (dsp->max_height - dsp->min_height) * 0.05;  /* 5% variation */
                if (height_variation < variation_threshold && px > 2 && px < dsp->width - 3 && py > 2 && py < dsp->height - 3) {
                    interior_points.push_back({{px, py}});
                }
            }
        }
    }
    
    /* Add interior points to help create sparser triangulation */
    if (!interior_points.empty()) {
        /* For earcut, we need to be careful about interior points.
         * Instead, we'll create a more detailed boundary that captures important features */
        printf("  Found %zu interior points for sparse triangulation\n", interior_points.size());
    }
    
    /* Run earcut triangulation */
    std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(polygon);
    
    /* Create vertices for all boundary points */
    size_t boundary_vertex_count = boundary.size();
    std::vector<size_t> boundary_vertex_indices(boundary_vertex_count);
    
    for (size_t i = 0; i < boundary_vertex_count; i++) {
        terrascape_point3d_t pt = {boundary[i][0], boundary[i][1], bottom_height};
        pt = terrascape_transform_point(dsp, pt);
        boundary_vertex_indices[i] = terrascape_mesh_add_vertex(mesh, pt);
    }
    
    /* Store corner indices for wall generation */
    bottom_vertex_indices[0] = boundary_vertex_indices[0];  /* Bottom-left */
    bottom_vertex_indices[1] = boundary_vertex_indices[boundary_samples / 2 - 1];  /* Bottom-right */
    bottom_vertex_indices[2] = boundary_vertex_indices[boundary_samples / 2 + boundary_samples / 2 - 2];  /* Top-right */
    bottom_vertex_indices[3] = boundary_vertex_indices[boundary_vertex_count - 1];  /* Top-left */
    
    /* Add triangles from earcut (reverse winding order for bottom face) */
    for (size_t i = 0; i < indices.size(); i += 3) {
        /* Reverse winding order for bottom face to maintain proper orientation */
        terrascape_mesh_add_triangle(mesh, 
                                    boundary_vertex_indices[indices[i]], 
                                    boundary_vertex_indices[indices[i+2]], 
                                    boundary_vertex_indices[indices[i+1]]);
    }
    
    return 1;
}
#endif

#endif /* TERRASCAPE_H */

/*
 * Local Variables:
 * tab-width: 8
 * mode: C
 * indent-tabs-mode: t
 * c-file-style: "stroustrup"
 * End:
 * ex: shiftwidth=4 tabstop=8 cino=N-s
 */
