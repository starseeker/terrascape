/*
 * BRL-CAD DSP Integration Example
 * 
 * This demonstrates how the TerraScape C API would be integrated
 * into BRL-CAD's rt_dsp_tess function.
 */

#include "TerraScape.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Simulated BRL-CAD structures for demonstration */
typedef struct {
    double stom[16];              /* solid to model space transform */
    unsigned short *dsp_buf;      /* height data buffer */
    int dsp_xcnt;                 /* width */
    int dsp_ycnt;                 /* height */
} rt_dsp_internal_sim;

typedef struct {
    void *s_hd;                   /* shell head pointer (simulated) */
} nmgregion_sim;

typedef struct {
    void *placeholder;            /* model placeholder */
} model_sim;

typedef struct {
    void *idb_ptr;                /* pointer to internal data */
} rt_db_internal_sim;

/**
 * Demo wrapper for terrascape_params_from_brlcad_tolerances with debug output
 */
terrascape_params_t terrascape_params_from_brlcad_tolerances_verbose(
    const bg_tess_tol_sim *ttol, 
    const bn_tol_sim *tol,
    const terrascape_dsp_t *dsp) {
    
    terrascape_params_t params = terrascape_params_from_brlcad_tolerances(ttol, tol, dsp);
    
    if (!ttol || !tol || !dsp) {
        printf("Warning: NULL tolerance parameters, using defaults\n");
        return params;
    }
    
    /* Print what was selected */
    if (ttol->abs > 0.0) {
        printf("Using absolute tolerance: %.6f for error threshold\n", ttol->abs);
    } else {
        printf("Using general tolerance: %.6f for error threshold\n", tol->dist);
    }
    
    if (ttol->rel > 0.0 && ttol->rel <= 1.0) {
        printf("Using relative tolerance: %.6f -> %d%% triangle reduction\n", 
               ttol->rel, params.min_triangle_reduction);
    }
    
    if (ttol->norm > 0.0) {
        printf("Using normal tolerance: %.6f rad -> slope threshold %.6f\n", 
               ttol->norm, params.slope_threshold);
    }
    
    /* Check if threshold was adjusted */
    double terrain_height_range = dsp->max_height - dsp->min_height;
    double terrain_scale = (terrain_height_range > 1.0) ? terrain_height_range : 1.0;
    double min_threshold = terrain_scale * 1e-6;
    if (params.error_threshold == min_threshold) {
        printf("Adjusted error threshold to %.6f (minimum for terrain scale)\n", 
               params.error_threshold);
    }
    
    return params;
}

/**
 * Enhanced rt_dsp_tess function using TerraScape
 * This shows how BRL-CAD's rt_dsp_tess would be modified to properly
 * use tessellation tolerances
 */
int rt_dsp_tess_enhanced(nmgregion_sim **r, 
                        model_sim *m, 
                        rt_db_internal_sim *ip, 
                        const bg_tess_tol_sim *ttol,        /* BRL-CAD tessellation tolerance */
                        const bn_tol_sim *tol) {
    
    printf("Enhanced rt_dsp_tess using TerraScape C API\n");
    printf("===========================================\n");
    
    /* Get DSP internal data */
    rt_dsp_internal_sim *dsp_ip = (rt_dsp_internal_sim *)ip->idb_ptr;
    if (!dsp_ip) {
        printf("ERROR: No DSP internal data\n");
        return -1;
    }
    
    printf("DSP dimensions: %dx%d\n", dsp_ip->dsp_xcnt, dsp_ip->dsp_ycnt);
    
    /* Create TerraScape DSP structure from BRL-CAD data */
    terrascape_dsp_t ts_dsp = terrascape_dsp_create(dsp_ip->dsp_buf, 
                                                    dsp_ip->dsp_xcnt, 
                                                    dsp_ip->dsp_ycnt, 
                                                    dsp_ip->stom);
    
    printf("TerraScape DSP: height range %.1f to %.1f\n", 
           ts_dsp.min_height, ts_dsp.max_height);
    
    /* Create TerraScape mesh */
    terrascape_mesh_t *mesh = terrascape_mesh_create();
    if (!mesh) {
        printf("ERROR: Failed to create mesh\n");
        return -1;
    }
    
    /* Convert BRL-CAD tessellation tolerances to TerraScape parameters */
    printf("Converting BRL-CAD tolerances to TerraScape parameters...\n");
    terrascape_params_t params = terrascape_params_from_brlcad_tolerances_verbose(ttol, tol, &ts_dsp);
    
    /* Generate optimized triangle mesh using TerraScape */
    printf("Generating triangle mesh...\n");
    int result = terrascape_triangulate_dsp_surface(&ts_dsp, mesh, &params);
    if (!result) {
        printf("ERROR: Triangulation failed\n");
        terrascape_mesh_free(mesh);
        return -1;
    }
    
    printf("Generated %zu vertices and %zu triangles\n", 
           mesh->vertex_count, mesh->triangle_count);
    
    /* Convert TerraScape mesh to BRL-CAD NMG format */
    printf("Converting to NMG format...\n");
    printf("  Would create NMG region with shell containing:\n");
    printf("    - %zu vertices\n", mesh->vertex_count);
    printf("    - %zu triangular faces\n", mesh->triangle_count);
    printf("    - Appropriate edge topology\n");
    printf("    - Face plane equations\n");
    
    /* In real BRL-CAD integration, this would be:
     * 
     * 1. *r = nmg_mrsv(m);  // Create NMG region
     * 2. s = BU_LIST_FIRST(shell, &(*r)->s_hd);  // Get shell
     * 3. For each triangle in mesh:
     *    - Create vertices using nmg_vertex_gv()
     *    - Create faces using nmg_cmface()
     *    - Set face plane equations using nmg_fu_planeeqn()
     * 4. nmg_mark_edges_real(&s->l.magic, vlfree);
     * 5. nmg_region_a(*r, tol);
     * 6. nmg_make_faces_within_tol(s, vlfree, tol);
     */
    
    /* Simulate successful NMG creation */
    *r = (nmgregion_sim*)malloc(sizeof(nmgregion_sim));
    if (*r) {
        (*r)->s_hd = NULL;  /* Would point to actual shell */
        printf("NMG region created successfully\n");
    }
    
    /* Clean up */
    terrascape_mesh_free(mesh);
    
    printf("rt_dsp_tess_enhanced completed successfully\n");
    return 0;  /* Success */
}

/**
 * Test the enhanced DSP tessellation with different tolerance values
 */
int test_tolerance_variations(void) {
    printf("Testing Tolerance Variations\n");
    printf("============================\n\n");
    
    /* Create minimal test DSP data */
    rt_dsp_internal_sim dsp_internal;
    dsp_internal.dsp_xcnt = 5;
    dsp_internal.dsp_ycnt = 5;
    
    /* Create identity transform matrix */
    for (int i = 0; i < 16; i++) {
        dsp_internal.stom[i] = 0.0;
    }
    dsp_internal.stom[0] = dsp_internal.stom[5] = dsp_internal.stom[10] = dsp_internal.stom[15] = 1.0;
    
    /* Create simple height data */
    int total_points = dsp_internal.dsp_xcnt * dsp_internal.dsp_ycnt;
    dsp_internal.dsp_buf = (unsigned short*)malloc(sizeof(unsigned short) * total_points);
    
    for (int i = 0; i < total_points; i++) {
        dsp_internal.dsp_buf[i] = 10000 + i * 1000;  /* Simple height variation */
    }
    
    rt_db_internal_sim db_internal;
    db_internal.idb_ptr = &dsp_internal;
    
    model_sim model;
    model.placeholder = NULL;
    
    bn_tol_sim tolerance;
    tolerance.dist = 0.01;
    
    /* Test case 1: Very tight tolerances */
    {
        bg_tess_tol_sim tight_tol;
        tight_tol.abs = 0.001;   /* 1mm absolute */
        tight_tol.rel = 0.05;    /* 5% relative */
        tight_tol.norm = 0.02;   /* ~1 degree normal */
        
        printf("Test 1 - Tight tolerances (abs=%.3f, rel=%.1f%%, norm=%.3f rad):\n", 
               tight_tol.abs, tight_tol.rel * 100.0, tight_tol.norm);
        
        nmgregion_sim *region1 = NULL;
        int result1 = rt_dsp_tess_enhanced(&region1, &model, &db_internal, &tight_tol, &tolerance);
        printf("Result: %s\n\n", result1 == 0 ? "SUCCESS" : "FAILED");
        free(region1);
    }
    
    /* Test case 2: Loose tolerances */
    {
        bg_tess_tol_sim loose_tol;
        loose_tol.abs = 0.1;     /* 10cm absolute */
        loose_tol.rel = 0.5;     /* 50% relative */
        loose_tol.norm = 0.5;    /* ~28 degree normal */
        
        printf("Test 2 - Loose tolerances (abs=%.3f, rel=%.1f%%, norm=%.3f rad):\n", 
               loose_tol.abs, loose_tol.rel * 100.0, loose_tol.norm);
        
        nmgregion_sim *region2 = NULL;
        int result2 = rt_dsp_tess_enhanced(&region2, &model, &db_internal, &loose_tol, &tolerance);
        printf("Result: %s\n\n", result2 == 0 ? "SUCCESS" : "FAILED");
        free(region2);
    }
    
    /* Test case 3: Zero tolerances (should use defaults) */
    {
        bg_tess_tol_sim zero_tol;
        zero_tol.abs = 0.0;
        zero_tol.rel = 0.0;
        zero_tol.norm = 0.0;
        
        printf("Test 3 - Zero tolerances (should use defaults):\n");
        
        nmgregion_sim *region3 = NULL;
        int result3 = rt_dsp_tess_enhanced(&region3, &model, &db_internal, &zero_tol, &tolerance);
        printf("Result: %s\n\n", result3 == 0 ? "SUCCESS" : "FAILED");
        free(region3);
    }
    
    free(dsp_internal.dsp_buf);
    return 0;
}
int main() {
    printf("BRL-CAD DSP Integration Example\n");
    printf("===============================\n\n");
    
    /* Create simulated BRL-CAD DSP data */
    rt_dsp_internal_sim dsp_internal;
    dsp_internal.dsp_xcnt = 15;
    dsp_internal.dsp_ycnt = 15;
    
    /* Create identity transform matrix */
    for (int i = 0; i < 16; i++) {
        dsp_internal.stom[i] = 0.0;
    }
    dsp_internal.stom[0] = dsp_internal.stom[5] = dsp_internal.stom[10] = dsp_internal.stom[15] = 1.0;
    
    /* Create sample height data - a more complex terrain */
    int total_points = dsp_internal.dsp_xcnt * dsp_internal.dsp_ycnt;
    dsp_internal.dsp_buf = (unsigned short*)malloc(sizeof(unsigned short) * total_points);
    
    /* Generate a combination of waves and ridges */
    for (int y = 0; y < dsp_internal.dsp_ycnt; y++) {
        for (int x = 0; x < dsp_internal.dsp_xcnt; x++) {
            double fx = (double)x / (dsp_internal.dsp_xcnt - 1);
            double fy = (double)y / (dsp_internal.dsp_ycnt - 1);
            
            /* Combine multiple patterns */
            double wave1 = sin(fx * 2.0 * M_PI) * cos(fy * 2.0 * M_PI);
            double wave2 = sin(fx * 4.0 * M_PI) * 0.3;
            double ridge = exp(-((fx - 0.5) * (fx - 0.5) + (fy - 0.5) * (fy - 0.5)) * 8.0);
            
            double height = (wave1 + wave2 + ridge + 1.0) * 0.25;  /* Normalize to [0,0.5] */
            unsigned short height_val = (unsigned short)(height * 65535);
            dsp_internal.dsp_buf[y * dsp_internal.dsp_xcnt + x] = height_val;
        }
    }
    
    /* Create simulated BRL-CAD structures */
    rt_db_internal_sim db_internal;
    db_internal.idb_ptr = &dsp_internal;
    
    model_sim model;
    model.placeholder = NULL;
    
    bn_tol_sim tolerance;
    tolerance.dist = 0.01;  /* 1cm general tolerance */
    
    /* Create BRL-CAD tessellation tolerance structure */
    bg_tess_tol_sim tessellation_tolerance;
    tessellation_tolerance.abs = 0.005;    /* 5mm absolute tolerance */
    tessellation_tolerance.rel = 0.2;      /* 20% relative tolerance */
    tessellation_tolerance.norm = 0.1;     /* ~5.7 degree normal tolerance */
    
    printf("BRL-CAD Tessellation Tolerances:\n");
    printf("  Absolute: %.6f model units\n", tessellation_tolerance.abs);
    printf("  Relative: %.1f%% \n", tessellation_tolerance.rel * 100.0);
    printf("  Normal: %.6f radians (%.1f degrees)\n", 
           tessellation_tolerance.norm, tessellation_tolerance.norm * 180.0 / M_PI);
    printf("\n");
    
    nmgregion_sim *region = NULL;
    
    /* Test the enhanced tessellation function */
    int result = rt_dsp_tess_enhanced(&region, &model, &db_internal, &tessellation_tolerance, &tolerance);
    
    printf("\n");
    if (result == 0) {
        printf("Integration test PASSED\n");
        printf("Successfully generated NMG region from DSP data\n");
        printf("\nKey benefits of TerraScape integration:\n");
        printf("  - Optimized triangle count with quality preservation\n");
        printf("  - Adaptive sampling preserves important terrain features\n");
        printf("  - Boundary preservation maintains manifold properties\n");
        printf("  - Header-only implementation for easy integration\n");
    } else {
        printf("Integration test FAILED with code %d\n", result);
    }
    
    /* Clean up */
    free(dsp_internal.dsp_buf);
    free(region);
    
    /* Run tolerance variation tests */
    test_tolerance_variations();
    
    return result;
}