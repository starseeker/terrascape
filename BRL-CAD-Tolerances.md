# BRL-CAD Tolerance Support in TerraScape

## Overview

TerraScape now supports BRL-CAD style facetization tolerances (abs/rel/norm) in addition to the original Terra/Scape parameters. This enables compatibility with BRL-CAD's DSP tessellation system and provides better control over mesh quality vs. performance trade-offs.

## Tolerance Types

### Absolute Tolerance (`abs`)
- **Units**: Model units (same as terrain coordinates)
- **Purpose**: Sets maximum geometric error in absolute terms
- **Usage**: Good when you know the exact precision needed

### Relative Tolerance (`rel`) 
- **Units**: Fraction of bounding box diagonal (0.0 to 1.0)
- **Purpose**: Sets tolerance proportional to model size
- **Usage**: Good for scale-independent quality control

### Normal Tolerance (`norm`)
- **Units**: Degrees (angular deviation)  
- **Purpose**: Controls surface normal approximation quality
- **Usage**: Good for preserving surface features and curvature

## Usage Examples

### Command Line Interface

```bash
# Use BRL-CAD tolerances
./terrascape_demo --planar-patches --abs 0.5 --rel 0.01 --norm 2.0

# Mix different tolerance types
./terrascape_demo -s --abs 1.0 --norm 5.0

# Legacy Terra/Scape parameters (backward compatible)
./terrascape_demo -s --error 0.1 --reduction 70
```

### Programmatic Interface

```cpp
#include "TerraScape.hpp"

// BRL-CAD style tolerances
TerraScape::SimplificationParams params;
params.setBrlcadTolerances(1.0, 0.02, 3.0); // abs=1.0, rel=2%, norm=3°

// Legacy style (still supported)
TerraScape::SimplificationParams legacy_params;
legacy_params.setErrorTol(0.5);
legacy_params.setMinReduction(80);
```

## Tolerance Conversion Logic

### Feature Size for mmesh
The system computes an effective feature size for mmesh decimation:

```
effective_tolerance = max(
    abs_tol,                                    // absolute tolerance
    rel_tol * bounding_box_diagonal,           // relative tolerance 
    cell_size * tan(norm_tol * π/180)          // normal tolerance
)
feature_size = effective_tolerance
```

### Coplanar Tolerance for Planar Patches
For coplanar patch detection, tolerances are scaled to be more aggressive:

```
coplanar_tolerance = max(
    abs_tol * 3.0,                                      // 3x more permissive
    rel_tol * bounding_box_diagonal * 2.0,             // 2x more permissive
    cell_size * tan((norm_tol * 2.0) * π/180)          // double the angle
)
```

### Design Rationale

1. **Loose tolerances → More aggressive simplification**: As specified in the requirements, loose tolerances translate to larger feature sizes and more aggressive coplanar patching.

2. **Maximum-based combination**: When multiple tolerances are specified, the system uses the maximum computed value to ensure all constraints are satisfied.

3. **Flat area optimization**: Coplanar tolerances are deliberately more permissive because "flat areas contribute the least to overall shape."

## Integration with BRL-CAD DSP

This implementation is designed to be compatible with BRL-CAD's `dsp_tess.cpp` approach:

- Maps abs/rel/norm tolerances to mmesh feature sizes
- Handles coplanar preprocessing for better flat area triangulation  
- Provides similar behavior to BRL-CAD's "feature size" bot decimate command

## Backward Compatibility

The original Terra/Scape parameters (`error_tol`, `slope_tol`, etc.) are fully preserved and continue to work as before. BRL-CAD tolerances are only used when explicitly set via `setBrlcadTolerances()` or the command line options.