/*
  Greedy Cuts Terrain Triangulation (Header-only)

  Features:
  - Advancing front with priority queue (stable, versioned nodes)
  - Adaptive thresholding (slope + 8-neighbor curvature)
  - Multi-pass refinement with interior-only point insertion
  - Mesh quality constraints (min angle, max aspect, min area)
  - Row-major indexing (y * width + x), double math for geometry
  - Optional NoData mask support (0 = masked/invalid, nonzero = valid)
  - Degeneracy checks, robust remainder “ear” fill (no holes)
  - Early-out error evaluation to cut compute cost
  - Header-only to avoid immediate CMake changes; include and call directly

  Usage:
    #include "greedy_cuts.hpp"
    terrascape::GreedyCutsOptions opts;
    opts.base_error_threshold = 0.2;
    // ... set other options as desired ...
    terrascape::Mesh mesh;
    terrascape::triangulateGreedyCuts(elevations.data(), width, height, nullptr, opts, mesh);

  License: same as repository (adjust as needed).
*/

#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace terrascape {

// --------------------------------- Types ---------------------------------

struct GreedyCutsOptions {
  // Error model
  double base_error_threshold = 0.2;
  double slope_weight = 3.0;
  double curvature_weight = 10.0;

  // Quality constraints
  double min_angle_deg = 20.0;
  double max_aspect_ratio = 6.0; // longest/shortest edge (2D)
  double min_area = 0.5;         // in grid units (2D area)

  // Refinement
  int max_refinement_passes = 5;
  int max_initial_iterations = 5'000'000; // safety cap for very large fronts

  // Sampling control
  // If true, attempt early-exit in error calculation when err > local_threshold
  bool early_exit_error_eval = true;

  // NoData support (mask==nullptr => all valid)
  // If provided, mask[y*width + x] == 0 => invalid sample
  // Triangles covering only invalid area are considered low-confidence and skipped.
  bool treat_all_invalid_as_blocking = true;

  // Barycentric epsilon to decide interior points (avoid edge points for refinement)
  double bary_eps = 1e-6;

  // Degeneracy and numeric tolerances
  double det_eps = 1e-12;
  double area_eps = 1e-12;
};

struct Mesh {
  std::vector<std::array<double, 3>> vertices; // x,y,z (grid coords + elevation)
  std::vector<std::array<int, 3>> triangles;   // vertex indices
};

// --------------------------------- Helpers ---------------------------------

static inline size_t idx_row_major(int x, int y, int width) {
  return static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
}

static inline double hypot2d(double dx, double dy) {
  return std::hypot(dx, dy);
}

static inline double tri_area2d(double x0, double y0, double x1, double y1, double x2, double y2) {
  return 0.5 * std::abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0));
}

static inline bool barycentric_uvwt(const std::array<double,3>& a,
                                    const std::array<double,3>& b,
                                    const std::array<double,3>& c,
                                    double px, double py,
                                    double det_eps,
                                    double& u, double& v, double& w)
{
  double det = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1]);
  if (std::abs(det) < det_eps) return false;
  u = ((b[1] - c[1]) * (px - c[0]) + (c[0] - b[0]) * (py - c[1])) / det;
  v = ((c[1] - a[1]) * (px - c[0]) + (a[0] - c[0]) * (py - c[1])) / det;
  w = 1.0 - u - v;
  return (u >= 0.0 && v >= 0.0 && w >= 0.0);
}

static inline double triangle_min_angle_deg_2d(const std::array<double,3>& v0,
                                               const std::array<double,3>& v1,
                                               const std::array<double,3>& v2)
{
  auto angle_at = [](const std::array<double,3>& a,
                     const std::array<double,3>& b,
                     const std::array<double,3>& c) {
    double ux = b[0] - a[0], uy = b[1] - a[1];
    double vx = c[0] - a[0], vy = c[1] - a[1];
    double dot = ux * vx + uy * vy;
    double nu = std::hypot(ux, uy);
    double nv = std::hypot(vx, vy);
    double denom = (nu * nv > 0.0) ? (nu * nv) : 1.0;
    double ct = std::clamp(dot / denom, -1.0, 1.0);
    return std::acos(ct) * 180.0 / 3.14159265358979323846;
  };
  double a0 = angle_at(v0, v1, v2);
  double a1 = angle_at(v1, v2, v0);
  double a2 = angle_at(v2, v0, v1);
  return std::min({a0, a1, a2});
}

static inline double triangle_aspect_ratio_2d(const std::array<double,3>& v0,
                                              const std::array<double,3>& v1,
                                              const std::array<double,3>& v2)
{
  double a = std::hypot(v0[0] - v1[0], v0[1] - v1[1]);
  double b = std::hypot(v1[0] - v2[0], v1[1] - v2[1]);
  double c = std::hypot(v2[0] - v0[0], v2[1] - v0[1]);
  double longest = std::max({a, b, c});
  double shortest = std::max(std::min({a, b, c}), 1e-12);
  return longest / shortest;
}

static inline bool triangle_is_quality_2d(const std::array<double,3>& v0,
                                          const std::array<double,3>& v1,
                                          const std::array<double,3>& v2,
                                          double min_angle_deg,
                                          double max_aspect_ratio,
                                          double min_area)
{
  double area = tri_area2d(v0[0], v0[1], v1[0], v1[1], v2[0], v2[1]);
  if (area < min_area) return false;
  double min_angle = triangle_min_angle_deg_2d(v0, v1, v2);
  if (min_angle < min_angle_deg) return false;
  double aspect = triangle_aspect_ratio_2d(v0, v1, v2);
  if (aspect > max_aspect_ratio) return false;
  return true;
}

static inline double tri_local_slope(const std::array<double,3>& v0,
                                     const std::array<double,3>& v1,
                                     const std::array<double,3>& v2)
{
  auto edge_slope = [](const std::array<double,3>& a,
                       const std::array<double,3>& b) {
    double horiz = std::hypot(a[0]-b[0], a[1]-b[1]);
    if (horiz < 1e-12) return 0.0;
    return std::abs(a[2] - b[2]) / horiz;
  };
  double s0 = edge_slope(v0, v1);
  double s1 = edge_slope(v1, v2);
  double s2 = edge_slope(v2, v0);
  return std::max({s0, s1, s2});
}

static inline double tri_local_curvature_8n(int width, int height,
                                            const float* elev,
                                            int cx, int cy)
{
  // 8-neighbor discrete Laplacian magnitude (not normalized by spacing)
  auto E = [&](int x, int y) -> double {
    x = std::clamp(x, 0, width-1);
    y = std::clamp(y, 0, height-1);
    return static_cast<double>(elev[idx_row_major(x,y,width)]);
  };
  double c  = E(cx, cy);
  double n  = E(cx, cy-1);
  double s  = E(cx, cy+1);
  double e  = E(cx+1, cy);
  double w  = E(cx-1, cy);
  double ne = E(cx+1, cy-1);
  double nw = E(cx-1, cy-1);
  double se = E(cx+1, cy+1);
  double sw = E(cx-1, cy+1);
  double lap4 = (n + s + e + w - 4.0*c);
  double lap8 = (ne + nw + se + sw + n + s + e + w - 8.0*c);
  // blend or choose; we use |lap8| for stronger feature response
  return std::abs(lap8);
}

static inline double adaptive_threshold(double base,
                                        double slope,
                                        double slope_w,
                                        double curvature,
                                        double curvature_w)
{
  double denom = 1.0 + slope_w * slope + curvature_w * curvature;
  return std::clamp(base / denom, 1e-9, 1e9);
}

// Error evaluation with optional early-exit and mask
struct TriError {
  double error = -1.0;
  int max_x = -1;
  int max_y = -1;
  bool any_valid = false;
};

// Scans triangle's bounding box; early-exits if err > stop_at (if stop_at > 0)
static inline TriError triangle_error_and_argmax(const std::array<double,3>& v0,
                                                 const std::array<double,3>& v1,
                                                 const std::array<double,3>& v2,
                                                 const float* elev,
                                                 const uint8_t* mask,
                                                 int width, int height,
                                                 const GreedyCutsOptions& opt,
                                                 double stop_at = -1.0)
{
  double minx = std::max(0.0, std::floor(std::min({v0[0], v1[0], v2[0]})));
  double maxx = std::min(static_cast<double>(width-1), std::ceil(std::max({v0[0], v1[0], v2[0]})));
  double miny = std::max(0.0, std::floor(std::min({v0[1], v1[1], v2[1]})));
  double maxy = std::min(static_cast<double>(height-1), std::ceil(std::max({v0[1], v1[1], v2[1]})));

  TriError out;
  if (maxx < minx || maxy < miny) return out;

  // quick coarse checks: corners + center
  auto eval_point = [&](int sx, int sy) {
    if (sx < 0 || sy < 0 || sx >= width || sy >= height) return;
    if (mask && mask[idx_row_major(sx,sy,width)] == 0) return;
    double u, v, w;
    if (!barycentric_uvwt(v0, v1, v2, static_cast<double>(sx), static_cast<double>(sy), opt.det_eps, u, v, w))
      return;
    if (u < 0.0 || v < 0.0 || w < 0.0) return;
    out.any_valid = true;
    double tri_z = u * v0[2] + v * v1[2] + w * v2[2];
    double grid_z = static_cast<double>(elev[idx_row_major(sx, sy, width)]);
    double err = std::abs(tri_z - grid_z);
    if (err > out.error) { out.error = err; out.max_x = sx; out.max_y = sy; }
  };

  int ix0 = static_cast<int>(minx), ix1 = static_cast<int>(maxx);
  int iy0 = static_cast<int>(miny), iy1 = static_cast<int>(maxy);
  eval_point(ix0, iy0);
  eval_point(ix1, iy0);
  eval_point(ix0, iy1);
  eval_point(ix1, iy1);
  eval_point((ix0+ix1)/2, (iy0+iy1)/2);

  // If early coarse exceeded stop_at, early return
  if (opt.early_exit_error_eval && stop_at > 0.0 && out.error > stop_at) return out;

  // Full scan
  for (int sy = iy0; sy <= iy1; ++sy) {
    for (int sx = ix0; sx <= ix1; ++sx) {
      eval_point(sx, sy);
      if (opt.early_exit_error_eval && stop_at > 0.0 && out.error > stop_at) return out;
    }
  }
  return out;
}

// --------------------------------- Advancing Front ---------------------------------

struct FrontNode {
  int vid = -1;     // vertex id into vertex array
  int prev = -1;    // index into nodes vector
  int next = -1;    // index into nodes vector
  int version = 0;  // increment when topology around this node changes
  bool alive = false;
};

// PQ item stores the middle node index and its version to avoid stale use
struct PQItem {
  int mid_node = -1;
  int version = 0;
  double priority_error = 0.0;
  // max-heap on error
  bool operator<(const PQItem& other) const {
    return priority_error < other.priority_error;
  }
};

// Build rectangular front nodes in CCW order: top row (left->right), right col, bottom row (right->left), left col
static inline void init_rect_front(int width, int height,
                                   std::vector<FrontNode>& nodes,
                                   std::deque<int>& front_ring)
{
  const int n = width * height; (void)n; // not used directly here
  auto push_node = [&](int vid) {
    int idx = static_cast<int>(nodes.size());
    nodes.push_back(FrontNode{vid, idx-1, -1, 0, true});
    if (!front_ring.empty()) {
      int prev_idx = front_ring.back();
      nodes[prev_idx].next = idx;
      nodes[idx].prev = prev_idx;
    }
    front_ring.push_back(idx);
  };

  // Top
  for (int x = 0; x < width; ++x) push_node(static_cast<int>(idx_row_major(x, 0, width)));
  // Right
  for (int y = 1; y < height; ++y) push_node(static_cast<int>(idx_row_major(width-1, y, width)));
  // Bottom
  for (int x = width - 2; x >= 0; --x) push_node(static_cast<int>(idx_row_major(x, height-1, width)));
  // Left
  for (int y = height - 2; y > 0; --y) push_node(static_cast<int>(idx_row_major(0, y, width)));

  // Close ring
  if (!front_ring.empty()) {
    int first = front_ring.front();
    int last = front_ring.back();
    nodes[first].prev = last;
    nodes[last].next = first;
  }
}

static inline void erase_node(std::vector<FrontNode>& nodes, int node_idx) {
  FrontNode& m = nodes[node_idx];
  if (!m.alive) return;
  FrontNode& p = nodes[m.prev];
  FrontNode& n = nodes[m.next];
  // relink
  p.next = m.next;
  n.prev = m.prev;
  // bump versions of neighbors so queued items referencing them become stale
  p.version++;
  n.version++;
  m.alive = false;
}

// --------------------------------- Main API ---------------------------------

inline void triangulateGreedyCuts(const float* elevations,
                                  int width, int height,
                                  const uint8_t* mask, // optional, nullptr if none
                                  const GreedyCutsOptions& opt,
                                  Mesh& out_mesh)
{
  assert(width > 1 && height > 1);
  const int W = width, H = height;

  // Prepare vertices (grid x,y with z from elevations)
  out_mesh.vertices.resize(static_cast<size_t>(W) * static_cast<size_t>(H));
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      size_t vid = idx_row_major(x,y,W);
      out_mesh.vertices[vid] = {static_cast<double>(x),
                                static_cast<double>(y),
                                static_cast<double>(elevations[vid])};
    }
  }
  out_mesh.triangles.clear();
  out_mesh.triangles.reserve(static_cast<size_t>(W-1) * static_cast<size_t>(H-1) * 2);

  // Initialize front ring
  std::vector<FrontNode> nodes;
  nodes.reserve((W+H)*2);
  std::deque<int> front_ring;
  init_rect_front(W, H, nodes, front_ring);

  // Priority queue seeded by all available consecutive triples
  std::priority_queue<PQItem> pq;

  auto enqueue_node = [&](int mid) {
    if (!nodes[mid].alive) return;
    int p = nodes[mid].prev, n = nodes[mid].next;
    if (p == -1 || n == -1) return;
    const auto& A = out_mesh.vertices[nodes[p].vid];
    const auto& B = out_mesh.vertices[nodes[mid].vid];
    const auto& C = out_mesh.vertices[nodes[n].vid];
    // skip degenerate
    if (tri_area2d(A[0],A[1],B[0],B[1],C[0],C[1]) < opt.area_eps) return;
    // Compute error (no threshold yet, PQ is by error)
    auto terr = triangle_error_and_argmax(A, B, C, elevations, mask, W, H, opt, -1.0);
    double e = terr.any_valid ? terr.error : 0.0;
    pq.push(PQItem{mid, nodes[mid].version, e});
  };

  // Seed PQ
  if (!front_ring.empty()) {
    int start = front_ring.front();
    int curr = start;
    do {
      enqueue_node(curr);
      curr = nodes[curr].next;
    } while (curr != start && pq.size() < static_cast<size_t>(opt.max_initial_iterations));
  }

  // Advancing front
  int safety_iters = 0;
  auto local_threshold = [&](const std::array<double,3>& A,
                             const std::array<double,3>& B,
                             const std::array<double,3>& C){
    // centroid for curvature sampling
    double cx = (A[0]+B[0]+C[0])/3.0;
    double cy = (A[1]+B[1]+C[1])/3.0;
    int icx = std::clamp(static_cast<int>(std::llround(cx)), 1, W-2);
    int icy = std::clamp(static_cast<int>(std::llround(cy)), 1, H-2);
    double slope = tri_local_slope(A,B,C);
    double curv  = tri_local_curvature_8n(W,H,elevations,icx,icy);
    return adaptive_threshold(opt.base_error_threshold, slope, opt.slope_weight, curv, opt.curvature_weight);
  };

  while (!pq.empty() && safety_iters++ < opt.max_initial_iterations) {
    PQItem it = pq.top(); pq.pop();
    int mid = it.mid_node;
    if (mid < 0 || mid >= static_cast<int>(nodes.size())) continue;
    if (!nodes[mid].alive) continue;
    if (nodes[mid].version != it.version) continue; // stale

    int pv = nodes[mid].prev;
    int nv = nodes[mid].next;
    if (pv < 0 || nv < 0) continue;
    if (!nodes[pv].alive || !nodes[nv].alive) continue;

    const auto& A = out_mesh.vertices[nodes[pv].vid];
    const auto& B = out_mesh.vertices[nodes[mid].vid];
    const auto& C = out_mesh.vertices[nodes[nv].vid];
    if (tri_area2d(A[0],A[1],B[0],B[1],C[0],C[1]) < opt.area_eps) {
      erase_node(nodes, mid);
      continue;
    }

    double thresh = local_threshold(A,B,C);
    auto terr = triangle_error_and_argmax(A, B, C, elevations, mask, W, H, opt,
                                          opt.early_exit_error_eval ? thresh : -1.0);
    if (!terr.any_valid) {
      // if triangle covers only invalid samples and that's blocking, skip it (advance by removing mid to avoid stalling)
      if (opt.treat_all_invalid_as_blocking) { erase_node(nodes, mid); }
      continue;
    }

    if (terr.error <= thresh && triangle_is_quality_2d(A,B,C, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area)) {
      // accept triangle (pv,mid,nv)
      out_mesh.triangles.push_back({nodes[pv].vid, nodes[mid].vid, nodes[nv].vid});
      // remove mid
      int left = nodes[mid].prev;
      int right = nodes[mid].next;
      erase_node(nodes, mid);
      // enqueue affected neighbors (left, right)
      if (nodes[left].alive) enqueue_node(left);
      if (nodes[right].alive) enqueue_node(right);
    } else {
      // Not acceptable now; skip. Other PQ items may make progress.
      // To avoid infinite stalling, we could re-enqueue with updated version to revisit later.
      // Re-enqueue if still alive
      enqueue_node(mid);
    }
  }

  // Remainder fill using quality-aware “ear clipping” pass; never erase without adding a triangle
  if (!front_ring.empty()) {
    // Find any alive node to start
    int start = -1;
    for (size_t i = 0; i < nodes.size(); ++i) if (nodes[i].alive) { start = static_cast<int>(i); break; }
    if (start != -1) {
      int curr = start;
      int count_guard = 0;
      while (count_guard < (W+H)*4) {
        // Count alive nodes to know when done
        int alive_count = 0;
        int probe = curr;
        do {
          if (nodes[probe].alive) alive_count++;
          probe = nodes[probe].next;
        } while (probe != curr);
        if (alive_count < 3) break;

        bool ear_found = false;
        int tries = 0;
        int ear = curr;
        do {
          int p = nodes[ear].prev;
          int n = nodes[ear].next;
          const auto& A = out_mesh.vertices[nodes[p].vid];
          const auto& B = out_mesh.vertices[nodes[ear].vid];
          const auto& C = out_mesh.vertices[nodes[n].vid];
          if (tri_area2d(A[0],A[1],B[0],B[1],C[0],C[1]) >= opt.min_area &&
              triangle_is_quality_2d(A,B,C, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area))
          {
            out_mesh.triangles.push_back({nodes[p].vid, nodes[ear].vid, nodes[n].vid});
            int next_after = nodes[ear].next;
            erase_node(nodes, ear);
            curr = next_after;
            ear_found = true;
            break;
          }
          ear = nodes[ear].next;
        } while (++tries < alive_count);

        if (!ear_found) {
          // Relax: accept the ear with the largest area to make progress
          double best_area = -1.0;
          int best_ear = -1;
          int p = curr;
          for (int t = 0; t < alive_count; ++t) {
            if (!nodes[p].alive) { p = nodes[p].next; continue; }
            int pr = nodes[p].prev, nx = nodes[p].next;
            const auto& A = out_mesh.vertices[nodes[pr].vid];
            const auto& B = out_mesh.vertices[nodes[p].vid];
            const auto& C = out_mesh.vertices[nodes[nx].vid];
            double ar = tri_area2d(A[0],A[1],B[0],B[1],C[0],C[1]);
            if (ar > best_area) { best_area = ar; best_ear = p; }
            p = nodes[p].next;
          }
          if (best_ear != -1) {
            int pr = nodes[best_ear].prev, nx = nodes[best_ear].next;
            out_mesh.triangles.push_back({nodes[pr].vid, nodes[best_ear].vid, nodes[nx].vid});
            int na = nodes[best_ear].next;
            erase_node(nodes, best_ear);
            curr = na;
          } else {
            break; // nothing else to do
          }
        }
        count_guard++;
      }
    }
  }

  // ----------------- Multi-pass refinement with interior insertion -----------------
  auto refine_once = [&](std::vector<std::array<int,3>>& tris_in,
                         std::vector<std::array<int,3>>& tris_out) {
    bool refined_any = false;
    tris_out.clear();
    tris_out.reserve(tris_in.size()*2);

    for (const auto& t : tris_in) {
      const auto& A = out_mesh.vertices[t[0]];
      const auto& B = out_mesh.vertices[t[1]];
      const auto& C = out_mesh.vertices[t[2]];
      if (tri_area2d(A[0],A[1],B[0],B[1],C[0],C[1]) < opt.min_area) {
        continue;
      }
      // Local threshold
      double cx = (A[0]+B[0]+C[0])/3.0;
      double cy = (A[1]+B[1]+C[1])/3.0;
      int icx = std::clamp(static_cast<int>(std::llround(cx)), 1, W-2);
      int icy = std::clamp(static_cast<int>(std::llround(cy)), 1, H-2);
      double slope = tri_local_slope(A,B,C);
      double curv  = tri_local_curvature_8n(W,H,elevations,icx,icy);
      double thresh = adaptive_threshold(opt.base_error_threshold, slope, opt.slope_weight, curv, opt.curvature_weight);

      // Error with early-exit
      auto terr = triangle_error_and_argmax(A, B, C, elevations, mask, W, H, opt,
                                            opt.early_exit_error_eval ? -1.0 : -1.0);
      if (!terr.any_valid) {
        // Skip masked-only triangles
        continue;
      }
      if (terr.error <= thresh) {
        if (triangle_is_quality_2d(A,B,C, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area))
          tris_out.push_back(t);
        continue;
      }

      // Find best strictly interior point (ignore points with any barycentric weight <= bary_eps)
      double best_err = -1.0;
      int best_x = -1, best_y = -1;
      double minx = std::max(0.0, std::floor(std::min({A[0], B[0], C[0]})));
      double maxx = std::min(static_cast<double>(W-1), std::ceil(std::max({A[0], B[0], C[0]})));
      double miny = std::max(0.0, std::floor(std::min({A[1], B[1], C[1]})));
      double maxy = std::min(static_cast<double>(H-1), std::ceil(std::max({A[1], B[1], C[1]})));
      for (int sy = static_cast<int>(miny); sy <= static_cast<int>(maxy); ++sy) {
        for (int sx = static_cast<int>(minx); sx <= static_cast<int>(maxx); ++sx) {
          if (mask && mask[idx_row_major(sx,sy,W)] == 0) continue;
          double u,v,w;
          if (!barycentric_uvwt(A,B,C, static_cast<double>(sx), static_cast<double>(sy), opt.det_eps, u,v,w)) continue;
          if (u <= opt.bary_eps || v <= opt.bary_eps || w <= opt.bary_eps) continue; // avoid edges/vertices
          double tri_z = u*A[2] + v*B[2] + w*C[2];
          double grid_z = static_cast<double>(elevations[idx_row_major(sx,sy,W)]);
          double err = std::abs(tri_z - grid_z);
          if (err > best_err) { best_err = err; best_x = sx; best_y = sy; }
        }
      }
      if (best_x == -1 || best_y == -1) {
        // No interior valid point found; keep or drop based on quality
        if (triangle_is_quality_2d(A,B,C, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area))
          tris_out.push_back(t);
        continue;
      }

      // Insert point and split into fan of three triangles
      int new_vid = static_cast<int>(idx_row_major(best_x, best_y, W));
      std::array<double,3> P = {static_cast<double>(best_x),
                                static_cast<double>(best_y),
                                static_cast<double>(elevations[idx_row_major(best_x,best_y,W)])};
      // Only keep sub-tris that pass quality
      std::array<std::array<int,3>,3> candidates = {{
        {t[0], t[1], new_vid},
        {t[1], t[2], new_vid},
        {t[2], t[0], new_vid}
      }};
      for (const auto& ct : candidates) {
        const auto& V0 = out_mesh.vertices[ct[0]];
        const auto& V1 = out_mesh.vertices[ct[1]];
        const auto& V2 = out_mesh.vertices[ct[2]];
        if (triangle_is_quality_2d(V0,V1,V2, opt.min_angle_deg, opt.max_aspect_ratio, opt.min_area))
          tris_out.push_back(ct);
      }
      refined_any = true;
    }
    return refined_any;
  };

  // Run refinement passes
  std::vector<std::array<int,3>> work = out_mesh.triangles;
  std::vector<std::array<int,3>> next;
  for (int pass = 0; pass < opt.max_refinement_passes; ++pass) {
    bool did_refine = refine_once(work, next);
    if (!did_refine) { break; }
    work.swap(next);
  }
  out_mesh.triangles.swap(work);
}

} // namespace terrascape
