/*
 * TerraScape C Interface Implementation
 * 
 * This file implements the C99-compatible interface functions for BRL-CAD integration.
 * It provides a bridge between the C API and the C++ TerraScape implementation.
 */

#include "TerraScape.hpp"
#include <cstring>
#include <cstdlib>

extern "C" {

/* Implementation of C interface functions */

terrascape_mesh_t *terrascape_mesh_create(void) {
    terrascape_mesh_t *mesh = (terrascape_mesh_t*)calloc(1, sizeof(terrascape_mesh_t));
    if (!mesh) return NULL;
    
    mesh->vertices = NULL;
    mesh->triangles = NULL;
    mesh->vertex_count = 0;
    mesh->triangle_count = 0;
    mesh->vertex_capacity = 0;
    mesh->triangle_capacity = 0;
    
    return mesh;
}

void terrascape_mesh_free(terrascape_mesh_t *mesh) {
    if (!mesh) return;
    
    free(mesh->vertices);
    free(mesh->triangles);
    free(mesh);
}

double terrascape_dsp_get_height(const terrascape_dsp_data_t *dsp_data, int x, int y) {
    if (!dsp_data || !dsp_data->height_data || 
        x < 0 || x >= dsp_data->width || 
        y < 0 || y >= dsp_data->height) {
        return 0.0;
    }
    
    return static_cast<double>(dsp_data->height_data[y * dsp_data->width + x]);
}

void terrascape_transform_point(const terrascape_dsp_data_t *dsp_data,
                               const terrascape_point3d_t *dsp_point,
                               terrascape_point3d_t *model_point) {
    if (!dsp_data || !dsp_point || !model_point) return;
    
    const double *m = dsp_data->transform;
    double x = dsp_point->x;
    double y = dsp_point->y; 
    double z = dsp_point->z;
    
    // Apply 4x4 transformation matrix (column-major format)
    model_point->x = m[0] * x + m[4] * y + m[8]  * z + m[12];
    model_point->y = m[1] * x + m[5] * y + m[9]  * z + m[13];
    model_point->z = m[2] * x + m[6] * y + m[10] * z + m[14];
}

int terrascape_triangulate_dsp(const terrascape_dsp_data_t *dsp_data,
                               terrascape_mesh_t *mesh,
                               const terrascape_params_t *params) {
    if (!dsp_data || !mesh) return -1;
    
    try {
        // Convert DSP data to TerraScape format
        TerraScape::TerrainData terrain;
        if (!TerraScape::convertDSPToTerrain(dsp_data, terrain)) {
            return -2;
        }
        
        // Set up simplification parameters
        TerraScape::SimplificationParams ts_params;
        if (params) {
            ts_params.error_threshold = params->error_threshold;
            ts_params.min_triangle_reduction = params->min_triangle_reduction;
            ts_params.preserve_boundaries = (params->preserve_boundaries != 0);
        }
        
        // Generate mesh using TerraScape
        TerraScape::TerrainMesh ts_mesh;
        TerraScape::triangulateTerrainVolumeSimplified(terrain, ts_mesh, ts_params);
        
        // Apply transform to vertices
        for (auto &vertex : ts_mesh.vertices) {
            terrascape_point3d_t dsp_pt = {vertex.x, vertex.y, vertex.z};
            terrascape_point3d_t model_pt;
            terrascape_transform_point(dsp_data, &dsp_pt, &model_pt);
            vertex.x = model_pt.x;
            vertex.y = model_pt.y;
            vertex.z = model_pt.z;
        }
        
        // Convert back to C format
        if (!TerraScape::convertMeshToC(ts_mesh, mesh)) {
            return -3;
        }
        
        return 0;
        
    } catch (...) {
        return -4;
    }
}

int terrascape_triangulate_dsp_surface(const terrascape_dsp_data_t *dsp_data,
                                       terrascape_mesh_t *mesh,
                                       const terrascape_params_t *params) {
    if (!dsp_data || !mesh) return -1;
    
    try {
        // Convert DSP data to TerraScape format
        TerraScape::TerrainData terrain;
        if (!TerraScape::convertDSPToTerrain(dsp_data, terrain)) {
            return -2;
        }
        
        // Set up simplification parameters
        TerraScape::SimplificationParams ts_params;
        if (params) {
            ts_params.error_threshold = params->error_threshold;
            ts_params.min_triangle_reduction = params->min_triangle_reduction;
            ts_params.preserve_boundaries = (params->preserve_boundaries != 0);
        }
        
        // Generate surface-only mesh using TerraScape
        TerraScape::TerrainMesh ts_mesh;
        TerraScape::triangulateTerrainSurfaceOnly(terrain, ts_mesh, ts_params);
        
        // Apply transform to vertices
        for (auto &vertex : ts_mesh.vertices) {
            terrascape_point3d_t dsp_pt = {vertex.x, vertex.y, vertex.z};
            terrascape_point3d_t model_pt;
            terrascape_transform_point(dsp_data, &dsp_pt, &model_pt);
            vertex.x = model_pt.x;
            vertex.y = model_pt.y;
            vertex.z = model_pt.z;
        }
        
        // Convert back to C format
        if (!TerraScape::convertMeshToC(ts_mesh, mesh)) {
            return -3;
        }
        
        return 0;
        
    } catch (...) {
        return -4;
    }
}

} /* extern "C" */