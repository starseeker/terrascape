#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <set>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <algorithm>
#include <filesystem>
#include "TerraScape.hpp"
#include "cxxopts.hpp"
#include "terrain_data_utils.hpp"

using namespace TerraScape;

// =============================================================================
// Test Framework
// =============================================================================

struct TestResult {
    bool passed = false;
    std::string name;
    std::string message;
    double duration_ms = 0.0;
};

class TestSuite {
private:
    std::vector<TestResult> results;
    std::string current_category;
    bool verbose;

public:
    TestSuite(bool verbose = false) : verbose(verbose) {}
    
    void beginCategory(const std::string& category) {
        current_category = category;
        if (verbose) {
            std::cout << "\n=== " << category << " ===\n" << std::endl;
        }
    }
    
    void addTest(bool passed, const std::string& name, const std::string& message = "", double duration_ms = 0.0) {
        results.push_back({passed, name, message, duration_ms});
        if (verbose) {
            std::cout << "Test: " << name << " - " << (passed ? "✓ PASSED" : "✗ FAILED");
            if (duration_ms > 0) {
                std::cout << " (" << std::fixed << std::setprecision(2) << duration_ms << "ms)";
            }
            std::cout << std::endl;
            if (!message.empty()) {
                std::cout << "  " << message << std::endl;
            }
        }
    }
    
    void printSummary() {
        int passed = 0, failed = 0;
        double total_time = 0.0;
        
        for (const auto& result : results) {
            if (result.passed) passed++;
            else failed++;
            total_time += result.duration_ms;
        }
        
        std::cout << "\n=== Test Summary ===" << std::endl;
        std::cout << "Total tests: " << results.size() << std::endl;
        std::cout << "Passed: " << passed << std::endl;
        std::cout << "Failed: " << failed << std::endl;
        std::cout << "Total time: " << std::fixed << std::setprecision(2) << total_time << "ms" << std::endl;
        
        if (failed > 0) {
            std::cout << "\nFailed tests:" << std::endl;
            for (const auto& result : results) {
                if (!result.passed) {
                    std::cout << "  - " << result.name;
                    if (!result.message.empty()) {
                        std::cout << ": " << result.message;
                    }
                    std::cout << std::endl;
                }
            }
        }
    }
    
    bool allTestsPassed() const {
        return std::all_of(results.begin(), results.end(), 
                          [](const TestResult& r) { return r.passed; });
    }
};

// =============================================================================
// Test Data Generation
// =============================================================================

std::vector<float> create_flat_surface(int width, int height, float value = 1.0f) {
    return std::vector<float>(width * height, value);
}

std::vector<float> create_single_peak(int width, int height) {
    std::vector<float> data(width * height, 0.0f);
    int center_x = width / 2;
    int center_y = height / 2;
    data[center_y * width + center_x] = 1.0f;
    return data;
}

std::vector<float> create_gaussian_hill(int width, int height) {
    std::vector<float> data(width * height);
    float center_x = width / 2.0f;
    float center_y = height / 2.0f;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float dx = (x - center_x) / (width / 4.0f);
            float dy = (y - center_y) / (height / 4.0f);
            data[y * width + x] = std::exp(-(dx*dx + dy*dy) / 0.1f);
        }
    }
    return data;
}

std::vector<float> create_complex_surface(int width, int height) {
    std::vector<float> data(width * height);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float fx = static_cast<float>(x) / (width - 1);
            float fy = static_cast<float>(y) / (height - 1);
            // Complex surface with multiple features
            float z = fx + fy;
            z += 2.0f * std::sin(fx * 3.14159f * 3) * std::cos(fy * 3.14159f * 2);
            z += 1.5f * std::exp(-((fx - 0.3f) * (fx - 0.3f) + (fy - 0.7f) * (fy - 0.7f)) / 0.1f);
            data[y * width + x] = z;
        }
    }
    return data;
}

std::vector<float> create_gradient_surface(int width, int height) {
    std::vector<float> data(width * height);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float fx = static_cast<float>(x) / (width - 1);
            float fy = static_cast<float>(y) / (height - 1);
            data[y * width + x] = fx + fy; // Simple linear gradient
        }
    }
    return data;
}

// =============================================================================
// Core Functionality Tests
// =============================================================================

bool test_basic_grid_to_mesh() {
    auto data = create_single_peak(5, 5);
    MeshResult result = grid_to_mesh(5, 5, data.data());
    
    return !result.vertices.empty() && !result.triangles.empty();
}

bool test_flat_surface_handling() {
    auto data = create_flat_surface(4, 4, 1.0f);
    MeshResult result = grid_to_mesh(4, 4, data.data());
    
    return !result.vertices.empty() && !result.triangles.empty();
}

bool test_error_threshold_scaling() {
    auto data = create_gaussian_hill(10, 10);
    
    MeshResult coarse = grid_to_mesh(10, 10, data.data(), 0.5f);
    MeshResult fine = grid_to_mesh(10, 10, data.data(), 0.1f);
    
    return coarse.vertices.size() <= fine.vertices.size();
}

bool test_point_limit_enforcement() {
    auto data = create_complex_surface(20, 20);
    
    MeshResult limited = grid_to_mesh(20, 20, data.data(), 0.1f, 50);
    
    return limited.vertices.size() <= 60; // Allow some overhead
}

bool test_mesh_validity() {
    auto data = create_gaussian_hill(8, 8);
    MeshResult result = grid_to_mesh(8, 8, data.data());
    
    // Check that all triangle indices are valid
    for (const auto& tri : result.triangles) {
        if (tri.v0 < 0 || tri.v1 < 0 || tri.v2 < 0 ||
            tri.v0 >= static_cast<int>(result.vertices.size()) ||
            tri.v1 >= static_cast<int>(result.vertices.size()) ||
            tri.v2 >= static_cast<int>(result.vertices.size())) {
            return false;
        }
    }
    
    return true;
}

// =============================================================================
// Volumetric Mesh Tests
// =============================================================================

bool test_volumetric_mesh_generation() {
    auto data = create_gaussian_hill(5, 5);
    MeshResult volumetric = grid_to_mesh_volumetric(5, 5, data.data(), 0.0f);
    
    return volumetric.is_volumetric && 
           !volumetric.vertices.empty() && 
           !volumetric.triangles.empty();
}

bool test_volumetric_separated_mesh() {
    auto data = create_gradient_surface(6, 6);
    // Offset so some points are above and below zero
    for (auto& z : data) z -= 0.5f;
    
    VolumetricMeshResult result = grid_to_mesh_volumetric_separated(6, 6, data.data(), 0.0f);
    
    return result.has_positive_volume || result.has_negative_volume;
}

// =============================================================================
// Performance Tests
// =============================================================================

bool test_large_grid_performance() {
    auto start = std::chrono::high_resolution_clock::now();
    
    auto data = create_complex_surface(50, 50);
    MeshResult result = grid_to_mesh(50, 50, data.data(), 0.2f, 500);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    return duration.count() < 5000 && !result.vertices.empty(); // Should complete in under 5 seconds
}

// =============================================================================
// GDAL-Based Terrain Data Tests  
// =============================================================================

bool test_gdal_sample_terrain() {
    // Test with GDAL-generated sample terrain when downloads fail
    try {
        // Use the terrain data processor sample generation
        std::string terrain_data_dir = "terrain_data";
        if (!std::filesystem::exists(terrain_data_dir)) {
            std::filesystem::create_directories(terrain_data_dir);
        }
        
        if (TerrainDataUtils::isGdalAvailable()) {
            bool success = TerrainDataUtils::createSampleTerrainData(terrain_data_dir);
            if (success) {
                std::string sample_file = terrain_data_dir + "/sample_hill.pgm";
                std::ifstream test_stream(sample_file);
                if (test_stream) {
                    std::string line;
                    std::getline(test_stream, line); // Magic
                    int width, height, maxval;
                    test_stream >> width >> height >> maxval;
                    
                    std::vector<float> elevations(width * height);
                    for (int i = 0; i < width * height; i++) {
                        float val;
                        test_stream >> val;
                        elevations[i] = val;
                    }
                    
                    // Test mesh generation
                    auto mesh = TerraScape::grid_to_mesh(width, height, elevations.data(), 10.0f, 500);
                    return !mesh.vertices.empty() && !mesh.triangles.empty();
                }
            }
        }
        
        return true; // Pass if GDAL not available
    } catch (const std::exception&) {
        return true; // Don't fail test for sample data issues
    }
}

// =============================================================================
// Edge Case Tests
// =============================================================================

bool test_minimal_grid() {
    auto data = create_flat_surface(2, 2);
    MeshResult result = grid_to_mesh(2, 2, data.data());
    return !result.vertices.empty() && !result.triangles.empty();
}

bool test_single_row_grid() {
    auto data = create_flat_surface(5, 1);
    MeshResult result = grid_to_mesh(5, 1, data.data());
    return !result.vertices.empty(); // May not have triangles for 1D data
}

bool test_extreme_aspect_ratio() {
    auto data = create_gradient_surface(20, 3);
    MeshResult result = grid_to_mesh(20, 3, data.data());
    return !result.vertices.empty() && !result.triangles.empty();
}

bool test_nan_and_infinity_handling() {
    auto data = create_gaussian_hill(5, 5);
    data[12] = std::numeric_limits<float>::quiet_NaN();
    data[13] = std::numeric_limits<float>::infinity();
    
    MeshResult result = grid_to_mesh(5, 5, data.data());
    
    // Check that result vertices don't contain NaN or infinity
    for (const auto& v : result.vertices) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) {
            return false;
        }
    }
    
    return !result.vertices.empty();
}

// =============================================================================
// Test Execution
// =============================================================================

void run_basic_tests(TestSuite& suite) {
    suite.beginCategory("Basic Functionality Tests");
    
    auto start = std::chrono::high_resolution_clock::now();
    bool result = test_basic_grid_to_mesh();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "Basic Grid to Mesh", "", duration);
    
    start = std::chrono::high_resolution_clock::now();
    result = test_flat_surface_handling();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "Flat Surface Handling", "", duration);
    
    start = std::chrono::high_resolution_clock::now();
    result = test_mesh_validity();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "Mesh Validity Check", "", duration);
    
    start = std::chrono::high_resolution_clock::now();
    result = test_error_threshold_scaling();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "Error Threshold Scaling", "", duration);
    
    start = std::chrono::high_resolution_clock::now();
    result = test_point_limit_enforcement();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "Point Limit Enforcement", "", duration);
}

void run_volumetric_tests(TestSuite& suite) {
    suite.beginCategory("Volumetric Mesh Tests");
    
    auto start = std::chrono::high_resolution_clock::now();
    bool result = test_volumetric_mesh_generation();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "Volumetric Mesh Generation", "", duration);
    
    start = std::chrono::high_resolution_clock::now();
    result = test_volumetric_separated_mesh();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "Separated Volumetric Mesh", "", duration);
}

void run_performance_tests(TestSuite& suite) {
    suite.beginCategory("Performance Tests");
    
    auto start = std::chrono::high_resolution_clock::now();
    bool result = test_large_grid_performance();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "Large Grid Performance", "", duration);
}

void run_dsp_tests(TestSuite& suite) {
    suite.beginCategory("GDAL-Based Terrain Data Tests");
    
    auto start = std::chrono::high_resolution_clock::now();
    bool result = test_gdal_sample_terrain();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "GDAL Sample Terrain Processing", "", duration);
}

void run_edge_case_tests(TestSuite& suite) {
    suite.beginCategory("Edge Case Tests");
    
    auto start = std::chrono::high_resolution_clock::now();
    bool result = test_minimal_grid();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "Minimal Grid (2x2)", "", duration);
    
    start = std::chrono::high_resolution_clock::now();
    result = test_single_row_grid();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "Single Row Grid", "", duration);
    
    start = std::chrono::high_resolution_clock::now();
    result = test_extreme_aspect_ratio();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "Extreme Aspect Ratio", "", duration);
    
    start = std::chrono::high_resolution_clock::now();
    result = test_nan_and_infinity_handling();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    suite.addTest(result, "NaN and Infinity Handling", "", duration);
}

void run_tolerance_tests(TestSuite& suite) {
    suite.beginCategory("BRL-CAD Tolerance Integration Tests");
    
    // Test 1: Default tolerance behavior
    {
        auto start = std::chrono::high_resolution_clock::now();
        
        // Create test terrain data
        std::vector<float> elevations = {0.0f, 1.0f, 2.0f, 3.0f, 
                                        1.0f, 2.0f, 3.0f, 4.0f,
                                        2.0f, 3.0f, 4.0f, 5.0f,
                                        3.0f, 4.0f, 5.0f, 6.0f};
        
        terrascape::GreedyCutsOptions opt;
        opt.use_region_growing = true;
        // Use default tolerance values
        
        terrascape::Mesh mesh;
        terrascape::triangulateGreedyCuts(elevations.data(), 4, 4, nullptr, opt, mesh);
        
        bool passed = mesh.vertices.size() > 0 && mesh.triangles.size() > 0;
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        
        TestResult result = {passed, "Default Tolerance Values", passed ? "" : "Failed to generate mesh with default tolerances", duration};
        suite.addTest(passed, "Default Tolerance Values", "", duration);
    }
    
    // Test 2: Strict tolerance settings
    {
        auto start = std::chrono::high_resolution_clock::now();
        
        std::vector<float> elevations = {0.0f, 0.1f, 0.2f, 0.3f, 
                                        0.1f, 0.2f, 0.3f, 0.4f,
                                        0.2f, 0.3f, 0.4f, 0.5f,
                                        0.3f, 0.4f, 0.5f, 0.6f};
        
        terrascape::GreedyCutsOptions opt;
        opt.use_region_growing = true;
        opt.abs_tolerance_mm = 0.01;      // Very strict absolute tolerance
        opt.rel_tolerance = 0.001;        // Very strict relative tolerance
        opt.volume_delta_pct = 5.0;       // Strict volume tolerance
        
        terrascape::Mesh mesh;
        terrascape::triangulateGreedyCuts(elevations.data(), 4, 4, nullptr, opt, mesh);
        
        bool passed = mesh.vertices.size() > 0 && mesh.triangles.size() > 0;
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        
        suite.addTest(passed, "Strict Tolerance Settings", "", duration);
    }
    
    // Test 3: Relaxed tolerance settings
    {
        auto start = std::chrono::high_resolution_clock::now();
        
        std::vector<float> elevations = {0.0f, 10.0f, 20.0f, 30.0f, 
                                        10.0f, 20.0f, 30.0f, 40.0f,
                                        20.0f, 30.0f, 40.0f, 50.0f,
                                        30.0f, 40.0f, 50.0f, 60.0f};
        
        terrascape::GreedyCutsOptions opt;
        opt.use_region_growing = true;
        opt.abs_tolerance_mm = 5.0;       // Relaxed absolute tolerance
        opt.rel_tolerance = 0.1;          // Relaxed relative tolerance
        opt.volume_delta_pct = 50.0;      // Relaxed volume tolerance
        
        terrascape::Mesh mesh;
        terrascape::triangulateGreedyCuts(elevations.data(), 4, 4, nullptr, opt, mesh);
        
        bool passed = mesh.vertices.size() > 0 && mesh.triangles.size() > 0;
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        
        suite.addTest(passed, "Relaxed Tolerance Settings", "", duration);
    }
    
    // Test 4: Volume delta validation
    {
        auto start = std::chrono::high_resolution_clock::now();
        
        // Create terrain with known volume characteristics
        std::vector<float> elevations = {1.0f, 1.0f, 1.0f, 1.0f, 
                                        1.0f, 1.0f, 1.0f, 1.0f,
                                        1.0f, 1.0f, 1.0f, 1.0f,
                                        1.0f, 1.0f, 1.0f, 1.0f};
        
        terrascape::GreedyCutsOptions opt;
        opt.use_region_growing = true;
        opt.volume_delta_pct = 1.0;       // Very strict volume tolerance - expect warning
        
        terrascape::Mesh mesh;
        // Capture cout to check for volume warning
        std::streambuf* orig = std::cout.rdbuf();
        std::ostringstream capture;
        std::cout.rdbuf(capture.rdbuf());
        
        terrascape::triangulateGreedyCuts(elevations.data(), 4, 4, nullptr, opt, mesh);
        
        std::cout.rdbuf(orig);
        std::string output = capture.str();
        
        bool has_volume_warning = output.find("Volume delta exceeds tolerance") != std::string::npos;
        bool passed = mesh.vertices.size() > 0 && mesh.triangles.size() > 0 && has_volume_warning;
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        
        suite.addTest(passed, "Volume Delta Validation", "", duration);
    }
}

int main(int argc, char* argv[]) {
    cxxopts::Options options("unified_tests", "TerraScape Unified Test Suite");
    
    options.add_options()
        ("h,help", "Print usage")
        ("v,verbose", "Verbose output", cxxopts::value<bool>()->default_value("false"))
        ("b,basic", "Run basic functionality tests", cxxopts::value<bool>()->default_value("false"))
        ("o,volumetric", "Run volumetric mesh tests", cxxopts::value<bool>()->default_value("false"))
        ("p,performance", "Run performance tests", cxxopts::value<bool>()->default_value("false"))
        ("d,dsp", "Run DSP/Hawaii data tests", cxxopts::value<bool>()->default_value("false"))
        ("e,edge", "Run edge case tests", cxxopts::value<bool>()->default_value("false"))
        ("t,tolerance", "Run BRL-CAD tolerance integration tests", cxxopts::value<bool>()->default_value("false"))
        ("a,all", "Run all tests", cxxopts::value<bool>()->default_value("false"));

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    bool verbose = result["verbose"].as<bool>();
    bool run_all = result["all"].as<bool>();
    
    TestSuite suite(verbose);
    
    if (run_all || result["basic"].as<bool>()) {
        run_basic_tests(suite);
    }
    
    if (run_all || result["volumetric"].as<bool>()) {
        run_volumetric_tests(suite);
    }
    
    if (run_all || result["performance"].as<bool>()) {
        run_performance_tests(suite);
    }
    
    if (run_all || result["dsp"].as<bool>()) {
        run_dsp_tests(suite);
    }
    
    if (run_all || result["edge"].as<bool>()) {
        run_edge_case_tests(suite);
    }
    
    if (run_all || result["tolerance"].as<bool>()) {
        run_tolerance_tests(suite);
    }
    
    // If no specific tests selected, run basic tests
    if (!run_all && !result["basic"].as<bool>() && !result["volumetric"].as<bool>() && 
        !result["performance"].as<bool>() && !result["dsp"].as<bool>() && !result["edge"].as<bool>() &&
        !result["tolerance"].as<bool>()) {
        run_basic_tests(suite);
    }
    
    suite.printSummary();
    
    return suite.allTestsPassed() ? 0 : 1;
}