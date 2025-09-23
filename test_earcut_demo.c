/*
 * Test program demonstrating earcut optimization with complex bottom plane scenarios
 */

#include "terrascape.h"
#include <stdio.h>
#include <stdlib.h>

int main() {
    printf("Terrascape Earcut Optimization Demonstration\n");
    printf("============================================\n");
    
    /* Create a larger test terrain to better demonstrate sparse triangulation benefits */
    int width = 50, height = 50;
    unsigned short *height_data = (unsigned short*)malloc(width * height * sizeof(unsigned short));
    
    /* Create terrain with some flat areas that would benefit from sparse triangulation */
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (x < 10 || x >= width - 10 || y < 10 || y >= height - 10) {
                /* Border area with some height variation */
                height_data[y * width + x] = 1000 + (x + y) * 5;
            } else {
                /* Large flat interior area - perfect for sparse triangulation */
                height_data[y * width + x] = 1200 + (rand() % 50);  /* Minimal variation */
            }
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
    
    printf("Large test terrain: %dx%d, height range %.1f to %.1f\n", 
           dsp.width, dsp.height, dsp.min_height, dsp.max_height);
    
    /* Test 1: Surface-only mesh */
    printf("\n=== Test 1: Surface-only mesh ===\n");
    terrascape_mesh_t *surface_mesh = terrascape_mesh_create();
    terrascape_params_t surface_params = terrascape_params_default();
    
    terrascape_triangulate_dsp_surface(&dsp, surface_mesh, &surface_params);
    printf("Surface: %zu vertices, %zu triangles\n", 
           surface_mesh->vertex_count, surface_mesh->triangle_count);
    
    /* Test 2: Simple manifold (4-vertex bottom plane) */
    printf("\n=== Test 2: Simple manifold (4-vertex bottom) ===\n");
    terrascape_mesh_t *manifold_simple = terrascape_mesh_create();
    terrascape_params_t simple_params = terrascape_params_default();
    simple_params.generate_manifold = 1;
    simple_params.optimize_bottom_plane = 0;
    
    terrascape_triangulate_dsp_surface(&dsp, manifold_simple, &simple_params);
    printf("Simple manifold: %zu vertices, %zu triangles\n", 
           manifold_simple->vertex_count, manifold_simple->triangle_count);
    
    size_t simple_additional = manifold_simple->triangle_count - surface_mesh->triangle_count;
    printf("Additional triangles: %zu\n", simple_additional);
    
#ifdef __cplusplus
    /* Test 3: Earcut optimized manifold */
    printf("\n=== Test 3: Earcut optimized manifold ===\n");
    terrascape_mesh_t *manifold_earcut = terrascape_mesh_create();
    terrascape_params_t earcut_params = terrascape_params_default();
    earcut_params.generate_manifold = 1;
    earcut_params.optimize_bottom_plane = 1;
    
    terrascape_triangulate_dsp_surface(&dsp, manifold_earcut, &earcut_params);
    printf("Earcut manifold: %zu vertices, %zu triangles\n", 
           manifold_earcut->vertex_count, manifold_earcut->triangle_count);
    
    size_t earcut_additional = manifold_earcut->triangle_count - surface_mesh->triangle_count;
    printf("Additional triangles: %zu\n", earcut_additional);
    
    int triangle_difference = (int)earcut_additional - (int)simple_additional;
    printf("Triangle difference (earcut vs simple): %d\n", triangle_difference);
    
    if (triangle_difference < 0) {
        printf("✓ Earcut reduced triangles by %d\n", -triangle_difference);
    } else if (triangle_difference == 0) {
        printf("= Same triangle count (minimal case)\n");
    } else {
        printf("! Earcut added %d triangles (more detailed boundary)\n", triangle_difference);
    }
    
    terrascape_mesh_free(manifold_earcut);
#else
    printf("\n=== Test 3: Skipped (requires C++ compilation) ===\n");
#endif
    
    /* Test 4: Demonstrate the concept with manual sparse triangulation */
    printf("\n=== Test 4: Bottom plane triangulation comparison ===\n");
    printf("For a %dx%d terrain:\n", width, height);
    printf("- Dense bottom plane: %d triangles (2 per cell)\n", (width-1) * (height-1) * 2);
    printf("- Simple bottom plane: 2 triangles (single rectangle)\n");
    printf("- Earcut optimized: Variable based on terrain complexity\n");
    printf("\nEarcut benefits are most apparent with:\n");
    printf("  • Complex terrain boundaries (non-rectangular)\n");
    printf("  • Terrain with holes or exclusion areas\n");
    printf("  • Very large flat areas requiring minimal triangulation\n");
    printf("  • Zero-height regions needing full-density interior edges\n");
    
    printf("\n=== Key Insight ===\n");
    printf("The current implementation demonstrates the infrastructure for\n");
    printf("earcut integration. Real benefits will be seen when processing\n");
    printf("terrain with complex boundaries or requiring hole handling.\n");
    
    /* Clean up */
    terrascape_mesh_free(surface_mesh);
    terrascape_mesh_free(manifold_simple);
    free(height_data);
    
    return 0;
}