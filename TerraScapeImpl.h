#pragma once

#include <vector>
#include <memory>

namespace TerraScape {

// Forward declarations for compatibility 
struct Vertex;
struct Triangle;
struct MeshResult;

// --- Input Preprocessing for Robustness ---

struct PreprocessingResult {
    std::vector<float> processed_elevations;  // Converted to float and preprocessed
    float adjusted_error_threshold;           // Clamped to reasonable minimum
    bool has_warnings = false;                // Whether any warnings were issued
    std::vector<std::string> warnings;       // Diagnostic messages
    bool is_degenerate = false;              // Whether input is degenerate (all flat/collinear)
    bool needs_jitter = false;               // Whether small jitter was added
    float scale_factor = 1.0f;               // Applied coordinate scaling
    float z_offset = 0.0f;                   // Applied Z offset for normalization
};

/**
 * Preprocess input elevation data to handle degenerate cases and improve robustness.
 * This function implements the hardening recommendations to prevent assertion failures
 * in the triangulation library.
 */
template<typename T>
PreprocessingResult preprocess_input_data(
    int width, int height, const T* elevations, 
    float& error_threshold, bool enable_jitter = true);

/**
 * Grid-aware triangulation implementation that takes advantage of the structured
 * nature of gridded terrain data. Uses advancing front algorithm for better
 * handling of collinear points and more predictable triangle quality.
 */
template<typename T>
MeshResult grid_to_mesh_impl(
    int width, int height, const T* elevations,
    float error_threshold, int point_limit);

} // namespace TerraScape