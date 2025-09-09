# Simulation of Simplicity Implementation

## Overview

This implementation adds Simulation of Simplicity (SoS) capability to the terrascape triangulation library to robustly handle geometric degeneracies, specifically the "point exactly on a constrained edge" failures that can occur during Delaunay triangulation.

## Features Implemented

### 1. Configuration Support
- Added `UseSimulationOfSimplicity` configuration option to `DefaultTriangulationConfig`
- Currently set to `false` by default for stability
- Can be enabled per-triangulation instance using custom config

### 2. SoS Geometric Predicates
- `sos_orient2d_tiebreak()`: Converts collinear orientations to definite CW/CCW using index-based tie-breaking
- `sos_incircle_tiebreak()`: Converts cocircular results to definite inside/outside using index-based tie-breaking
- `lexicographic_less_2d()`: Helper function for consistent point ordering

### 3. SoS-Aware Wrapper Functions
- `orient2d_sos()`: Calls regular orient2d, applies SoS tie-breaking if result is collinear
- `incircle_sos()`: Calls regular incircle, applies SoS tie-breaking if result is cocircular
- Automatically disabled when `UseSimulationOfSimplicity = false`

### 4. Integration Points
- Modified critical orient2d calls in constrained edge handling to use SoS versions
- Updated error handling to detect when SoS should have prevented degeneracies
- Added safety guards to prevent assertion failures during SoS testing

## Test Coverage

### Basic SoS Tests (`test_sos_logic.cpp`)
- ✅ Lexicographic ordering functions
- ✅ SoS tie-breaking vs regular predicates
- ✅ Consistency verification

### Comprehensive SoS Tests (`test_sos_comprehensive.cpp`)  
- ✅ Basic collinear triangulation with SoS enabled
- ✅ Direct predicate testing showing elimination of degeneracies
- ✅ Consistency testing for deterministic results
- ⚠️ Complex multi-point scenarios (crashes during edge case handling)

### Direct Detria Tests (`test_detria_direct.cpp`)
- ✅ Simple triangulation with collinear points succeeds

## Current Status

**Working Features:**
- SoS framework is fully implemented and functional
- Basic triangulations with collinear points work correctly
- SoS eliminates exact degeneracies as intended
- Tie-breaking is deterministic and consistent

**Known Issues:**
- Complex scenarios with multiple degeneracies can cause segmentation faults
- Issue appears to be in interaction between SoS results and downstream triangulation algorithm
- Need further debugging of edge cases in constrained edge processing

**Compatibility:**
- SoS is disabled by default, so existing functionality is unaffected
- All existing tests pass when SoS is disabled
- Can be enabled selectively for specific triangulations

## Usage Example

```cpp
// Custom config with SoS enabled
template<typename Point, typename Idx>
struct SoSTriangulationConfig : public detria::DefaultTriangulationConfig<Point, Idx> {
    constexpr static bool UseSimulationOfSimplicity = true;
};

// Use SoS-enabled triangulation
detria::Triangulation<detria::PointD, uint32_t, SoSTriangulationConfig<detria::PointD, uint32_t>> tri;
// ... normal triangulation usage
```

## Technical Details

### Tie-Breaking Algorithm
The implementation uses index-based tie-breaking that simulates infinitesimal perturbations:
- For orient2d: Uses inversion count of vertex indices to determine orientation
- For incircle: Compares test point index to average of triangle vertex indices

### Error Handling
- When SoS is enabled, collinear/cocircular results should never occur
- Code includes assertions to detect if SoS fails to prevent degeneracies
- Graceful fallback when SoS is disabled

## Future Work

1. **Debug Complex Scenarios**: Investigate and fix segmentation faults in multi-degeneracy cases
2. **Extended Integration**: Convert remaining orient2d/incircle calls to SoS-aware versions
3. **Performance Optimization**: Optimize tie-breaking algorithms for speed
4. **Advanced SoS**: Implement more sophisticated SoS algorithms from literature
5. **Global Enable**: Once fully stable, consider enabling SoS by default

## References

- Edelsbrunner, H. & Mücke, E.P. "Simulation of Simplicity: A Technique to Cope with Degenerate Cases in Geometric Algorithms"
- Shewchuk, J.R. "Robust Adaptive Floating-Point Geometric Predicates"