#pragma once

/*
 * File I/O functions for TerraScape demo applications
 * 
 * These functions handle reading terrain data from various file formats
 * and writing mesh data to output files. They are separated from the core
 * TerraScape library to keep the core focused on in-memory data processing
 * for BRL-CAD integration.
 */

#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include "TerraScape.hpp"

#ifdef HAVE_GDAL
#include <gdal_priv.h>
#include <gdal.h>
#endif

namespace TerraScapeIO {

    // Helper function to check file extension
    bool hasExtension(const std::string& filename, const std::string& ext);

    // Read terrain data from file using GDAL or custom PGM reader
    bool readTerrainFile(const std::string& filename, TerraScape::TerrainData& terrain);

    // Simple PGM file reader
    bool readPGMFile(const std::string& filename, TerraScape::TerrainData& terrain);

    // Write mesh to OBJ file
    bool writeObjFile(const std::string& filename, const TerraScape::TerrainMesh& mesh);

    // Implementation

    inline bool hasExtension(const std::string& filename, const std::string& ext) {
        if (filename.length() >= ext.length()) {
            return (0 == filename.compare(filename.length() - ext.length(), ext.length(), ext));
        }
        return false;
    }

    inline bool readTerrainFile(const std::string& filename, TerraScape::TerrainData& terrain) {
        // Try PGM format first
        if (hasExtension(filename, ".pgm") || hasExtension(filename, ".PGM")) {
            return readPGMFile(filename, terrain);
        }

    #ifdef HAVE_GDAL
        // Use GDAL for other formats
        GDALDataset* dataset = (GDALDataset*)GDALOpen(filename.c_str(), GA_ReadOnly);
        if (!dataset) {
            return false;
        }

        terrain.width = dataset->GetRasterXSize();
        terrain.height = dataset->GetRasterYSize();
        
        // Get geotransform for scaling
        double geotransform[6];
        if (dataset->GetGeoTransform(geotransform) == CE_None) {
            terrain.cell_size = geotransform[1]; // pixel width
            terrain.origin = TerraScape::Point3D(geotransform[0], geotransform[3], 0);
        } else {
            terrain.cell_size = 1.0;
            terrain.origin = TerraScape::Point3D(0, 0, 0);
        }

        GDALRasterBand* band = dataset->GetRasterBand(1);
        if (!band) {
            GDALClose(dataset);
            return false;
        }

        // Read the height data
        terrain.heights.resize(terrain.height);
        for (int y = 0; y < terrain.height; ++y) {
            terrain.heights[y].resize(terrain.width);
        }

        std::vector<float> scanline(terrain.width);
        terrain.min_height = std::numeric_limits<double>::max();
        terrain.max_height = std::numeric_limits<double>::lowest();

        for (int y = 0; y < terrain.height; ++y) {
            if (band->RasterIO(GF_Read, 0, y, terrain.width, 1, 
                             scanline.data(), terrain.width, 1, GDT_Float32, 0, 0) != CE_None) {
                GDALClose(dataset);
                return false;
            }

            for (int x = 0; x < terrain.width; ++x) {
                double height = static_cast<double>(scanline[x]);
                terrain.heights[y][x] = height;
                terrain.min_height = std::min(terrain.min_height, height);
                terrain.max_height = std::max(terrain.max_height, height);
            }
        }

        GDALClose(dataset);
        return true;
    #endif
        
        return false;
    }

    inline bool readPGMFile(const std::string& filename, TerraScape::TerrainData& terrain) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }

        std::string magic;
        file >> magic;
        if (magic != "P2") {
            return false; // Only support ASCII PGM
        }

        int width, height, max_val;
        file >> width >> height >> max_val;

        terrain.width = width;
        terrain.height = height;
        terrain.cell_size = 1.0;
        terrain.origin = TerraScape::Point3D(0, 0, 0);

        // Read the height data
        terrain.heights.resize(terrain.height);
        terrain.min_height = std::numeric_limits<double>::max();
        terrain.max_height = std::numeric_limits<double>::lowest();

        for (int y = 0; y < terrain.height; ++y) {
            terrain.heights[y].resize(terrain.width);
            for (int x = 0; x < terrain.width; ++x) {
                int pixel_value;
                file >> pixel_value;

                // Convert pixel value to height (normalize to reasonable range)
                double height = static_cast<double>(pixel_value) / max_val * 100.0; // Scale to 0-100 range
                terrain.heights[y][x] = height;
                terrain.min_height = std::min(terrain.min_height, height);
                terrain.max_height = std::max(terrain.max_height, height);
            }
        }

        file.close();
        return true;
    }

    inline bool writeObjFile(const std::string& filename, const TerraScape::TerrainMesh& mesh) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }

        file << "# TerraScape generated OBJ file" << std::endl;
        file << "# Vertices: " << mesh.vertices.size() << std::endl;
        file << "# Triangles: " << mesh.triangles.size() << std::endl;
        file << std::endl;

        // Write vertices
        for (const auto& vertex : mesh.vertices) {
            file << "v " << std::fixed << std::setprecision(6)
                 << vertex.x << " " << vertex.y << " " << vertex.z << std::endl;
        }

        file << std::endl;

        // Write triangles (OBJ uses 1-based indexing)
        for (const auto& triangle : mesh.triangles) {
            file << "f " << (triangle.vertices[0] + 1) << " "
                         << (triangle.vertices[1] + 1) << " "
                         << (triangle.vertices[2] + 1) << std::endl;
        }

        file.close();
        return true;
    }

} // namespace TerraScapeIO