/*
 * TerraScape Test Program with Real Data
 * 
 * Reads crater.pgm file and tests TerraScape triangulation with validation
 * for manifold and orientation properties.
 */

#include "terrascape.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

/* PGM file reading structure */
typedef struct {
    int width, height;
    int max_val;
    unsigned short *data;
} pgm_data_t;

/* Mesh validation results */
typedef struct {
    int is_manifold;
    int is_properly_oriented;
    int has_degenerate_triangles;
    int boundary_edges_count;
    int non_manifold_edges_count;
    double total_surface_area;
    double min_triangle_area;
    double max_triangle_area;
} mesh_validation_t;

/**
 * Read PGM file and convert to unsigned short array
 */
int read_pgm_file(const char *filename, pgm_data_t *pgm) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        printf("ERROR: Cannot open file %s\n", filename);
        return 0;
    }
    
    char magic[3];
    if (fscanf(file, "%2s", magic) != 1 || strcmp(magic, "P2") != 0) {
        printf("ERROR: Only P2 PGM format supported\n");
        fclose(file);
        return 0;
    }
    
    /* Skip comments */
    int c;
    while ((c = fgetc(file)) == '#') {
        while ((c = fgetc(file)) != '\n' && c != EOF);
    }
    ungetc(c, file);
    
    /* Read dimensions and max value */
    if (fscanf(file, "%d %d %d", &pgm->width, &pgm->height, &pgm->max_val) != 3) {
        printf("ERROR: Invalid PGM header\n");
        fclose(file);
        return 0;
    }
    
    printf("PGM file: %dx%d, max_val=%d\n", pgm->width, pgm->height, pgm->max_val);
    
    /* Allocate data array */
    size_t total_pixels = pgm->width * pgm->height;
    pgm->data = (unsigned short*)malloc(sizeof(unsigned short) * total_pixels);
    if (!pgm->data) {
        printf("ERROR: Cannot allocate memory for PGM data\n");
        fclose(file);
        return 0;
    }
    
    /* Read pixel data */
    for (size_t i = 0; i < total_pixels; i++) {
        int pixel_val;
        if (fscanf(file, "%d", &pixel_val) != 1) {
            printf("ERROR: Failed to read pixel data at position %zu\n", i);
            free(pgm->data);
            fclose(file);
            return 0;
        }
        pgm->data[i] = (unsigned short)pixel_val;
    }
    
    fclose(file);
    return 1;
}

/**
 * Free PGM data
 */
void free_pgm_data(pgm_data_t *pgm) {
    if (pgm && pgm->data) {
        free(pgm->data);
        pgm->data = NULL;
    }
}

/**
 * Validate triangle for degeneracy
 */
int is_triangle_degenerate(const terrascape_mesh_t *mesh, size_t tri_idx) {
    if (tri_idx >= mesh->triangle_count) return 1;
    
    const terrascape_triangle_t *tri = &mesh->triangles[tri_idx];
    
    /* Check for duplicate vertices */
    if (tri->v0 == tri->v1 || tri->v1 == tri->v2 || tri->v0 == tri->v2) {
        return 1;
    }
    
    /* Check if vertices are out of bounds */
    if (tri->v0 >= mesh->vertex_count || tri->v1 >= mesh->vertex_count || tri->v2 >= mesh->vertex_count) {
        return 1;
    }
    
    /* Check for very small area (near-collinear vertices) */
    terrascape_point3d_t p0 = mesh->vertices[tri->v0];
    terrascape_point3d_t p1 = mesh->vertices[tri->v1];
    terrascape_point3d_t p2 = mesh->vertices[tri->v2];
    
    terrascape_point3d_t edge1 = terrascape_point3d_sub(p1, p0);
    terrascape_point3d_t edge2 = terrascape_point3d_sub(p2, p0);
    terrascape_point3d_t cross = terrascape_point3d_cross(edge1, edge2);
    
    double area = terrascape_point3d_length(cross) * 0.5;
    return area < 1e-10;  /* Very small area threshold */
}

/**
 * Calculate triangle area
 */
double calculate_triangle_area(const terrascape_mesh_t *mesh, size_t tri_idx) {
    if (tri_idx >= mesh->triangle_count) return 0.0;
    
    const terrascape_triangle_t *tri = &mesh->triangles[tri_idx];
    terrascape_point3d_t p0 = mesh->vertices[tri->v0];
    terrascape_point3d_t p1 = mesh->vertices[tri->v1];
    terrascape_point3d_t p2 = mesh->vertices[tri->v2];
    
    terrascape_point3d_t edge1 = terrascape_point3d_sub(p1, p0);
    terrascape_point3d_t edge2 = terrascape_point3d_sub(p2, p0);
    terrascape_point3d_t cross = terrascape_point3d_cross(edge1, edge2);
    
    return terrascape_point3d_length(cross) * 0.5;
}

/**
 * Check if triangle has consistent winding (CCW when viewed from outside)
 */
int is_triangle_ccw_oriented(const terrascape_mesh_t *mesh, size_t tri_idx) {
    if (tri_idx >= mesh->triangle_count) return 0;
    
    const terrascape_triangle_t *tri = &mesh->triangles[tri_idx];
    terrascape_point3d_t p0 = mesh->vertices[tri->v0];
    terrascape_point3d_t p1 = mesh->vertices[tri->v1];
    terrascape_point3d_t p2 = mesh->vertices[tri->v2];
    
    terrascape_point3d_t edge1 = terrascape_point3d_sub(p1, p0);
    terrascape_point3d_t edge2 = terrascape_point3d_sub(p2, p0);
    terrascape_point3d_t cross = terrascape_point3d_cross(edge1, edge2);
    
    /* For terrain meshes, normal should generally point upward (positive Z) */
    return cross.z > 0.0;
}

/**
 * Build edge adjacency information for manifold checking
 */
typedef struct {
    size_t v0, v1;  /* Edge vertices (v0 < v1) */
    size_t tri_count;  /* Number of triangles using this edge */
    size_t triangles[2];  /* Triangle indices (up to 2 for manifold) */
} edge_info_t;

/**
 * Compare edges for sorting
 */
int compare_edges(const void *a, const void *b) {
    const edge_info_t *ea = (const edge_info_t*)a;
    const edge_info_t *eb = (const edge_info_t*)b;
    
    if (ea->v0 != eb->v0) return (ea->v0 < eb->v0) ? -1 : 1;
    return (ea->v1 < eb->v1) ? -1 : 1;
}

/**
 * Comprehensive mesh validation
 */
mesh_validation_t validate_mesh_properties(const terrascape_mesh_t *mesh) {
    mesh_validation_t result = {0};
    
    if (!mesh || mesh->triangle_count == 0) {
        return result;
    }
    
    printf("\nValidating mesh properties...\n");
    
    /* Check for degenerate triangles and calculate areas */
    result.min_triangle_area = 1e10;
    result.max_triangle_area = 0.0;
    int degenerate_count = 0;
    int ccw_count = 0;
    
    for (size_t i = 0; i < mesh->triangle_count; i++) {
        if (is_triangle_degenerate(mesh, i)) {
            degenerate_count++;
        } else {
            double area = calculate_triangle_area(mesh, i);
            result.total_surface_area += area;
            
            if (area < result.min_triangle_area) result.min_triangle_area = area;
            if (area > result.max_triangle_area) result.max_triangle_area = area;
        }
        
        if (is_triangle_ccw_oriented(mesh, i)) {
            ccw_count++;
        }
    }
    
    result.has_degenerate_triangles = (degenerate_count > 0);
    result.is_properly_oriented = (ccw_count == (int)mesh->triangle_count);
    
    printf("  Degenerate triangles: %d / %zu\n", degenerate_count, mesh->triangle_count);
    printf("  CCW oriented triangles: %d / %zu\n", ccw_count, mesh->triangle_count);
    printf("  Triangle area range: %.6f to %.6f\n", result.min_triangle_area, result.max_triangle_area);
    printf("  Total surface area: %.6f\n", result.total_surface_area);
    
    /* Build edge list for manifold checking */
    size_t edge_count = mesh->triangle_count * 3;
    edge_info_t *edges = (edge_info_t*)calloc(edge_count, sizeof(edge_info_t));
    if (!edges) {
        printf("  ERROR: Cannot allocate memory for edge validation\n");
        return result;
    }
    
    /* Extract all edges from triangles */
    size_t edge_idx = 0;
    for (size_t i = 0; i < mesh->triangle_count; i++) {
        const terrascape_triangle_t *tri = &mesh->triangles[i];
        
        /* Three edges per triangle */
        size_t vertices[3] = {tri->v0, tri->v1, tri->v2};
        for (int j = 0; j < 3; j++) {
            size_t v0 = vertices[j];
            size_t v1 = vertices[(j + 1) % 3];
            
            /* Ensure v0 < v1 for consistent edge representation */
            if (v0 > v1) {
                size_t temp = v0;
                v0 = v1;
                v1 = temp;
            }
            
            edges[edge_idx].v0 = v0;
            edges[edge_idx].v1 = v1;
            edges[edge_idx].tri_count = 1;
            edges[edge_idx].triangles[0] = i;
            edge_idx++;
        }
    }
    
    /* Sort edges to group identical ones */
    qsort(edges, edge_count, sizeof(edge_info_t), compare_edges);
    
    /* Count edge usage for manifold checking */
    size_t unique_edges = 0;
    result.boundary_edges_count = 0;
    result.non_manifold_edges_count = 0;
    
    for (size_t i = 0; i < edge_count; ) {
        size_t start = i;
        size_t count = 1;
        
        /* Count how many times this edge appears */
        while (i + count < edge_count && 
               edges[i].v0 == edges[i + count].v0 && 
               edges[i].v1 == edges[i + count].v1) {
            count++;
        }
        
        unique_edges++;
        
        if (count == 1) {
            result.boundary_edges_count++;
        } else if (count > 2) {
            result.non_manifold_edges_count++;
        }
        
        i += count;
    }
    
    result.is_manifold = (result.non_manifold_edges_count == 0);
    
    printf("  Unique edges: %zu\n", unique_edges);
    printf("  Boundary edges: %d\n", result.boundary_edges_count);
    printf("  Non-manifold edges: %d\n", result.non_manifold_edges_count);
    printf("  Is manifold: %s\n", result.is_manifold ? "YES" : "NO");
    printf("  Is properly oriented: %s\n", result.is_properly_oriented ? "YES" : "NO");
    
    free(edges);
    return result;
}

/**
 * Test different triangulation parameters
 */
void test_triangulation_parameters(const terrascape_dsp_t *dsp) {
    printf("\n=== Testing Different Triangulation Parameters ===\n");
    
    typedef struct {
        terrascape_params_t params;
        const char *name;
    } test_config_t;
    
    test_config_t configs[] = {
        {{0.0, 0, 1, 0, 0.0}, "Simple (no reduction)"},
        {{0.1, 25, 1, 0, 0.2}, "25% reduction (simple)"},
        {{0.1, 50, 1, 0, 0.2}, "50% reduction (simple)"},
        {{0.1, 25, 1, 1, 0.2}, "25% reduction + adaptive (Terra/Scape)"},
        {{0.2, 50, 1, 1, 0.3}, "50% reduction + adaptive (Terra/Scape)"},
        {{0.05, 70, 1, 1, 0.15}, "70% reduction + adaptive (aggressive)"}
    };
    
    int num_configs = sizeof(configs) / sizeof(configs[0]);
    
    for (int i = 0; i < num_configs; i++) {
        printf("\nConfiguration: %s\n", configs[i].name);
        printf("  Error threshold: %.3f, Slope threshold: %.3f\n", 
               configs[i].params.error_threshold, configs[i].params.slope_threshold);
        
        terrascape_mesh_t *mesh = terrascape_mesh_create();
        if (!mesh) {
            printf("  ERROR: Failed to create mesh\n");
            continue;
        }
        
        int result = terrascape_triangulate_dsp_surface(dsp, mesh, &configs[i].params);
        if (!result) {
            printf("  ERROR: Triangulation failed\n");
            terrascape_mesh_free(mesh);
            continue;
        }
        
        printf("  Generated: %zu vertices, %zu triangles\n", 
               mesh->vertex_count, mesh->triangle_count);
        
        /* Calculate triangle reduction percentage */
        size_t original_triangles = (dsp->width - 1) * (dsp->height - 1) * 2;
        double reduction = 100.0 * (1.0 - (double)mesh->triangle_count / original_triangles);
        printf("  Triangle reduction: %.1f%% (from %zu to %zu)\n", 
               reduction, original_triangles, mesh->triangle_count);
        
        /* Validate this configuration */
        mesh_validation_t validation = validate_mesh_properties(mesh);
        
        terrascape_mesh_free(mesh);
    }
}

/**
 * Main test program
 */
int main() {
    printf("TerraScape Real Data Test\n");
    printf("=========================\n");
    
    /* Read the crater PGM file */
    pgm_data_t pgm = {0};
    if (!read_pgm_file("crater.pgm", &pgm)) {
        return 1;
    }
    
    printf("Successfully loaded crater data\n");
    
    /* Create TerraScape DSP structure */
    terrascape_dsp_t dsp = terrascape_dsp_create(pgm.data, pgm.width, pgm.height, NULL);
    printf("DSP data: %dx%d, height range %.0f to %.0f\n", 
           dsp.width, dsp.height, dsp.min_height, dsp.max_height);
    
    /* Test basic triangulation */
    printf("\n=== Basic Triangulation Test ===\n");
    terrascape_mesh_t *mesh = terrascape_mesh_create();
    if (!mesh) {
        printf("ERROR: Failed to create mesh\n");
        free_pgm_data(&pgm);
        return 1;
    }
    
    terrascape_params_t params = terrascape_params_default();
    int result = terrascape_triangulate_dsp_surface(&dsp, mesh, &params);
    if (!result) {
        printf("ERROR: Triangulation failed\n");
        terrascape_mesh_free(mesh);
        free_pgm_data(&pgm);
        return 1;
    }
    
    printf("Generated mesh: %zu vertices, %zu triangles\n", 
           mesh->vertex_count, mesh->triangle_count);
    
    /* Validate the basic mesh */
    mesh_validation_t validation = validate_mesh_properties(mesh);
    
    /* Print sample vertices and triangles */
    printf("\nFirst 5 vertices:\n");
    for (size_t i = 0; i < mesh->vertex_count && i < 5; i++) {
        printf("  Vertex %zu: (%.2f, %.2f, %.2f)\n", 
               i, mesh->vertices[i].x, mesh->vertices[i].y, mesh->vertices[i].z);
    }
    
    printf("\nFirst 5 triangles:\n");
    for (size_t i = 0; i < mesh->triangle_count && i < 5; i++) {
        printf("  Triangle %zu: [%zu, %zu, %zu], normal (%.3f, %.3f, %.3f)\n", 
               i, mesh->triangles[i].v0, mesh->triangles[i].v1, mesh->triangles[i].v2,
               mesh->triangles[i].normal.x, mesh->triangles[i].normal.y, mesh->triangles[i].normal.z);
    }
    
    terrascape_mesh_free(mesh);
    
    /* Test different parameter configurations */
    test_triangulation_parameters(&dsp);
    
    /* Test terrain feature analysis */
    printf("\n=== Terrain Feature Analysis Test (Terra/Scape) ===\n");
    printf("Analyzing terrain features using Terra/Scape algorithms...\n");
    
    /* Sample feature analysis at various points */
    int sample_points[][2] = {
        {dsp.width/4, dsp.height/4},     /* Quarter point */
        {dsp.width/2, dsp.height/2},     /* Center point */
        {3*dsp.width/4, 3*dsp.height/4}, /* Three-quarter point */
        {10, 10},                        /* Near corner */
        {dsp.width-10, dsp.height-10}    /* Near opposite corner */
    };
    
    for (int i = 0; i < 5; i++) {
        int x = sample_points[i][0];
        int y = sample_points[i][1];
        
        terrascape_feature_t feature = terrascape_debug_analyze_point(&dsp, x, y);
        printf("  Point (%d,%d): height=%.1f, slope=%.3f, curvature=%.3f, roughness=%.3f, importance=%.3f%s\n", 
               x, y, terrascape_dsp_get_height(&dsp, x, y),
               feature.slope, feature.curvature, feature.roughness, feature.importance_score,
               feature.is_boundary ? " [BOUNDARY]" : "");
    }
    printf("\n=== Coordinate Transformation Test ===\n");
    terrascape_point3d_t test_points[] = {
        {0, 0, 100},
        {dsp.width/2.0, dsp.height/2.0, 200},
        {dsp.width-1.0, dsp.height-1.0, 300}
    };
    
    for (int i = 0; i < 3; i++) {
        terrascape_point3d_t transformed = terrascape_transform_point(&dsp, test_points[i]);
        printf("  Point (%.1f, %.1f, %.1f) -> (%.1f, %.1f, %.1f)\n",
               test_points[i].x, test_points[i].y, test_points[i].z,
               transformed.x, transformed.y, transformed.z);
    }
    
    /* Test vector math utilities */
    printf("\n=== Vector Math Test ===\n");
    terrascape_point3d_t a = {1.0, 0.0, 0.0};
    terrascape_point3d_t b = {0.0, 1.0, 0.0};
    terrascape_point3d_t c = {1.0, 1.0, 1.0};
    
    terrascape_point3d_t cross_ab = terrascape_point3d_cross(a, b);
    double dot_ab = terrascape_point3d_dot(a, b);
    double len_c = terrascape_point3d_length(c);
    terrascape_point3d_t norm_c = terrascape_point3d_normalize(c);
    
    printf("  Cross(a,b) = (%.1f, %.1f, %.1f)\n", cross_ab.x, cross_ab.y, cross_ab.z);
    printf("  Dot(a,b) = %.1f\n", dot_ab);
    printf("  Length(c) = %.3f\n", len_c);
    printf("  Normalize(c) = (%.3f, %.3f, %.3f)\n", norm_c.x, norm_c.y, norm_c.z);
    
    /* Clean up */
    free_pgm_data(&pgm);
    
    printf("\n=== Test Summary ===\n");
    printf("All tests completed successfully!\n");
    printf("The mesh is %s and %s\n",
           validation.is_manifold ? "manifold" : "NON-MANIFOLD",
           validation.is_properly_oriented ? "properly oriented" : "IMPROPERLY ORIENTED");
    
    if (validation.has_degenerate_triangles) {
        printf("WARNING: Mesh contains degenerate triangles\n");
    }
    
    return 0;
}
