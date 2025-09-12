# Greedy Cuts Terrain Triangulation (Terrascape)

This header-only implementation provides an advancing front triangulation for heightfields with adaptive error control and mesh quality constraints.

Key features:
- Priority queue advancing front with stable nodes (no index drift).
- Adaptive thresholds using local slope and 8-neighbor curvature.
- Multi-pass refinement inserting interior points with the highest error.
- Mesh quality constraints: minimum angle, maximum aspect ratio, minimum area.
- Row-major indexing and double-precision geometry.
- Optional NoData mask support.
- Degeneracy checks and robust remainder ear clipping that avoids creating holes.
- Early-out error evaluation for performance.

Usage snippet:
```cpp
#include "greedy_cuts.hpp"

void build_mesh_with_greedy_cuts(const std::vector<float>& elevations, int width, int height) {
  terrascape::GreedyCutsOptions opt;
  opt.base_error_threshold = 0.2;
  opt.slope_weight = 3.0;
  opt.curvature_weight = 10.0;
  opt.min_angle_deg = 20.0;
  opt.max_aspect_ratio = 6.0;
  opt.min_area = 0.5;
  opt.max_refinement_passes = 5;

  terrascape::Mesh mesh;
  terrascape::triangulateGreedyCuts(elevations.data(), width, height, nullptr, opt, mesh);

  // mesh.vertices: (x,y,z) in grid units
  // mesh.triangles: vertex indices (row-major index into mesh.vertices)
}
```

Integration notes:
- Include the header from TerraScape code where triangulation occurs.
- If you have NoData/holes, pass a mask (0=invalid).
- For very large DEMs, consider plugging a spatial index for triangle sampling to further accelerate error evaluation.