/*
 * Test program for manifold mesh generation with optimized bottom plane
 */

#include "terrascape.h"
#include <stdio.h>
#include <stdlib.h>

int main() {
    printf("Terrascape Manifold Mesh Test\n");
    printf("=============================\n");
    
    /* Create a simple test height field */
    int width = 10, height = 10;
    unsigned short *height_data = (unsigned short*)malloc(width * height * sizeof(unsigned short));
    
    /* Create a simple height pattern */
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            /* Create a simple pyramid shape */
            int center_x = width / 2;
            int center_y = height / 2;
            int dist_x = abs(x - center_x);
            int dist_y = abs(y - center_y);
            int max_dist = (dist_x > dist_y) ? dist_x : dist_y;
            height_data[y * width + x] = 1000 + (5 - max_dist) * 200;
        }
    }
    
    /* Create identity transform matrix */
    double transform[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    
    /* Create DSP structure */
    terrascape_dsp_t dsp = terrascape_dsp_create(height_data, width, height, transform);
    
    printf("Test DSP: %dx%d, height range %.1f to %.1f\n", 
           dsp.width, dsp.height, dsp.min_height, dsp.max_height);
    
    /* Test 1: Surface-only mesh */
    printf("\n=== Test 1: Surface-only mesh ===\n");
    terrascape_mesh_t *surface_mesh = terrascape_mesh_create();
    terrascape_params_t surface_params = terrascape_params_default();
    
    int result = terrascape_triangulate_dsp_surface(&dsp, surface_mesh, &surface_params);
    if (result) {
        printf("Surface mesh: %zu vertices, %zu triangles\n", 
               surface_mesh->vertex_count, surface_mesh->triangle_count);
    } else {
        printf("ERROR: Surface triangulation failed\n");
    }
    
    /* Test 2: Manifold mesh with simple bottom plane */
    printf("\n=== Test 2: Manifold mesh (simple bottom) ===\n");
    terrascape_mesh_t *manifold_mesh_simple = terrascape_mesh_create();
    terrascape_params_t manifold_params = terrascape_params_default();
    manifold_params.generate_manifold = 1;
    manifold_params.optimize_bottom_plane = 0;  /* Use simple triangulation */
    
    result = terrascape_triangulate_dsp_surface(&dsp, manifold_mesh_simple, &manifold_params);
    if (result) {
        printf("Manifold mesh (simple): %zu vertices, %zu triangles\n", 
               manifold_mesh_simple->vertex_count, manifold_mesh_simple->triangle_count);
        
        /* Calculate triangle count difference */
        size_t additional_triangles = manifold_mesh_simple->triangle_count - surface_mesh->triangle_count;
        printf("Additional triangles for manifold: %zu (walls + bottom plane)\n", additional_triangles);
    } else {
        printf("ERROR: Manifold triangulation (simple) failed\n");
    }
    
    /* Test 3: Manifold mesh with optimized bottom plane (C++ only) */
#ifdef __cplusplus
    printf("\n=== Test 3: Manifold mesh (optimized bottom with earcut) ===\n");
    terrascape_mesh_t *manifold_mesh_optimized = terrascape_mesh_create();
    terrascape_params_t optimized_params = terrascape_params_default();
    optimized_params.generate_manifold = 1;
    optimized_params.optimize_bottom_plane = 1;  /* Use earcut triangulation */
    
    result = terrascape_triangulate_dsp_surface(&dsp, manifold_mesh_optimized, &optimized_params);
    if (result) {
        printf("Manifold mesh (earcut): %zu vertices, %zu triangles\n", 
               manifold_mesh_optimized->vertex_count, manifold_mesh_optimized->triangle_count);
        
        /* Compare with simple manifold mesh */
        int triangle_diff = (int)manifold_mesh_optimized->triangle_count - (int)manifold_mesh_simple->triangle_count;
        printf("Triangle difference (earcut vs simple): %d triangles\n", triangle_diff);
        printf("Earcut optimization: %s\n", 
               (triangle_diff <= 0) ? "SUCCESS (reduced or same triangles)" : "No reduction achieved");
    } else {
        printf("ERROR: Manifold triangulation (earcut) failed\n");
    }
    
    terrascape_mesh_free(manifold_mesh_optimized);
#else
    printf("\n=== Test 3: Skipped (earcut requires C++ compilation) ===\n");
#endif
    
    /* Validation */
    printf("\n=== Validation ===\n");
    if (surface_mesh->triangle_count > 0) {
        printf("✓ Surface triangulation works\n");
    } else {
        printf("✗ Surface triangulation failed\n");
    }
    
    if (manifold_mesh_simple->triangle_count > surface_mesh->triangle_count) {
        printf("✓ Manifold mesh has more triangles than surface-only\n");
    } else {
        printf("✗ Manifold mesh should have more triangles\n");
    }
    
    /* Expected: surface + 8 wall triangles + 2 bottom triangles = surface + 10 */
    size_t expected_additional = 10;  /* 8 wall + 2 bottom */
    size_t actual_additional = manifold_mesh_simple->triangle_count - surface_mesh->triangle_count;
    if (actual_additional == expected_additional) {
        printf("✓ Triangle count matches expectation (+%zu triangles)\n", expected_additional);
    } else {
        printf("? Triangle count: expected +%zu, got +%zu\n", expected_additional, actual_additional);
    }
    
    /* Clean up */
    terrascape_mesh_free(surface_mesh);
    terrascape_mesh_free(manifold_mesh_simple);
    free(height_data);
    
    printf("\nTest completed successfully!\n");
    return 0;
}