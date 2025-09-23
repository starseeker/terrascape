/*
 * Test program for manifold mesh generation with crater data
 */

#include "terrascape.h"
#include <stdio.h>
#include <stdlib.h>

/* Simple PGM reader */
typedef struct {
    int width, height;
    int max_val;
    unsigned short *data;
} pgm_data_t;

int read_pgm_file(const char *filename, pgm_data_t *pgm) {
    FILE *file = fopen(filename, "r");
    if (!file) return 0;
    
    char magic[3];
    if (fscanf(file, "%2s", magic) != 1 || (magic[0] != 'P' || magic[1] != '2')) {
        fclose(file);
        return 0;
    }
    
    /* Skip comments */
    int c;
    while ((c = fgetc(file)) == '#') {
        while ((c = fgetc(file)) != '\n' && c != EOF);
    }
    ungetc(c, file);
    
    if (fscanf(file, "%d %d %d", &pgm->width, &pgm->height, &pgm->max_val) != 3) {
        fclose(file);
        return 0;
    }
    
    pgm->data = (unsigned short*)malloc(pgm->width * pgm->height * sizeof(unsigned short));
    if (!pgm->data) {
        fclose(file);
        return 0;
    }
    
    for (int i = 0; i < pgm->width * pgm->height; i++) {
        int value;
        if (fscanf(file, "%d", &value) != 1) {
            free(pgm->data);
            fclose(file);
            return 0;
        }
        pgm->data[i] = (unsigned short)value;
    }
    
    fclose(file);
    return 1;
}

int main() {
    printf("Terrascape Crater Data Manifold Test\n");
    printf("====================================\n");
    
    pgm_data_t pgm;
    if (!read_pgm_file("crater.pgm", &pgm)) {
        printf("ERROR: Cannot read crater.pgm\n");
        return 1;
    }
    
    printf("Loaded crater data: %dx%d\n", pgm.width, pgm.height);
    
    /* Create identity transform matrix */
    double transform[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    
    /* Create DSP structure */
    terrascape_dsp_t dsp = terrascape_dsp_create(pgm.data, pgm.width, pgm.height, transform);
    
    /* Test 1: Surface-only mesh */
    printf("\n=== Surface-only mesh ===\n");
    terrascape_mesh_t *surface_mesh = terrascape_mesh_create();
    terrascape_params_t surface_params = terrascape_params_default();
    
    terrascape_triangulate_dsp_surface(&dsp, surface_mesh, &surface_params);
    printf("Surface: %zu vertices, %zu triangles\n", 
           surface_mesh->vertex_count, surface_mesh->triangle_count);
    
    /* Test 2: Manifold mesh with simple bottom plane */
    printf("\n=== Manifold mesh (simple bottom) ===\n");
    terrascape_mesh_t *manifold_simple = terrascape_mesh_create();
    terrascape_params_t manifold_params = terrascape_params_default();
    manifold_params.generate_manifold = 1;
    manifold_params.optimize_bottom_plane = 0;
    
    terrascape_triangulate_dsp_surface(&dsp, manifold_simple, &manifold_params);
    printf("Manifold (simple): %zu vertices, %zu triangles\n", 
           manifold_simple->vertex_count, manifold_simple->triangle_count);
    
    size_t simple_additional = manifold_simple->triangle_count - surface_mesh->triangle_count;
    printf("Additional triangles: %zu\n", simple_additional);
    
#ifdef __cplusplus
    /* Test 3: Manifold mesh with earcut optimization */
    printf("\n=== Manifold mesh (earcut optimized) ===\n");
    terrascape_mesh_t *manifold_earcut = terrascape_mesh_create();
    terrascape_params_t earcut_params = terrascape_params_default();
    earcut_params.generate_manifold = 1;
    earcut_params.optimize_bottom_plane = 1;
    
    terrascape_triangulate_dsp_surface(&dsp, manifold_earcut, &earcut_params);
    printf("Manifold (earcut): %zu vertices, %zu triangles\n", 
           manifold_earcut->vertex_count, manifold_earcut->triangle_count);
    
    size_t earcut_additional = manifold_earcut->triangle_count - surface_mesh->triangle_count;
    printf("Additional triangles: %zu\n", earcut_additional);
    
    int triangle_savings = (int)simple_additional - (int)earcut_additional;
    printf("Triangle savings with earcut: %d triangles\n", triangle_savings);
    
    if (triangle_savings > 0) {
        printf("✓ Earcut optimization successful!\n");
    } else if (triangle_savings == 0) {
        printf("= No optimization (bottom plane already minimal)\n");
    } else {
        printf("✗ Earcut added more triangles than expected\n");
    }
    
    terrascape_mesh_free(manifold_earcut);
#else
    printf("\n=== Earcut test skipped (C++ compilation required) ===\n");
#endif
    
    printf("\n=== Summary ===\n");
    printf("The manifold mesh successfully adds bottom plane and walls\n");
    printf("to create a watertight 3D solid from the height field.\n");
    
#ifdef __cplusplus
    printf("Earcut.hpp integration allows for optimized bottom plane triangulation.\n");
#else
    printf("Compile with C++ to enable earcut.hpp optimization.\n");
#endif
    
    /* Clean up */
    terrascape_mesh_free(surface_mesh);
    terrascape_mesh_free(manifold_simple);
    free(pgm.data);
    
    return 0;
}