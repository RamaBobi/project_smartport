#pragma once
// ============================================================================
//  ackermann.hpp  –  Pure Ackermann steering geometry (no ROS dependency)
//
//  Reference equations (from image):
//
//    R        = L / tan(δ_cmd)          turning radius at the rear-axle centre
//    δ_inner  = atan( L / (R − W/2) )  inner wheel (sharper turn)
//    δ_outer  = atan( L / (R + W/2) )  outer wheel (flatter turn)
//
//  Sign convention (positive δ = turn LEFT):
//    δ_cmd > 0  →  turn left  →  LEFT  front = inner wheel
//    δ_cmd < 0  →  turn right →  RIGHT front = inner wheel
//    δ_cmd = 0  →  straight   →  both angles = 0
//
//  Truck geometry defaults (must match tractor.urdf):
//    L = wheelbase   = 3.0 m   (front axle X=+1.5, rear axle X=−1.5)
//    W = track_width = 2.4 m   (hubs at Y=±1.2)
// ============================================================================

#include <cmath>
#include <limits>

namespace smartport {

// ---------------------------------------------------------------------------
struct AckermannGeometry {
    double wheelbase{3.0};    // L [m] – longitudinal distance front↔rear axle
    double track_width{2.4};  // W [m] – lateral distance left↔right wheel centre
};

// ---------------------------------------------------------------------------
struct AckermannResult {
    double R{0.0};           // turning radius  [m]  (∞ when δ_cmd ≈ 0)
    double delta_inner{0.0}; // inner-wheel angle [rad]
    double delta_outer{0.0}; // outer-wheel angle [rad]
    double delta_fl{0.0};    // front-LEFT  wheel command [rad]
    double delta_fr{0.0};    // front-RIGHT wheel command [rad]
};

// ---------------------------------------------------------------------------
/// Compute per-wheel steering angles from a commanded centre delta.
///
/// @param delta_cmd   Commanded centre steering angle [rad]  (+= left, −= right)
/// @param geom        Vehicle geometry (wheelbase L, track width W)
/// @return            AckermannResult with R, inner/outer angles, and fl/fr
// ---------------------------------------------------------------------------
inline AckermannResult compute_ackermann(
    double delta_cmd,
    const AckermannGeometry & geom = AckermannGeometry{})
{
    AckermannResult res;

    // ── Dead-band: treat as straight ────────────────────────────────────────
    if (std::abs(delta_cmd) < 1e-6) {
        res.R = std::numeric_limits<double>::infinity();
        return res;
    }

    const double L    = geom.wheelbase;
    const double W    = geom.track_width;
    const double sign = (delta_cmd > 0.0) ? 1.0 : -1.0;
    const double abs_d = std::abs(delta_cmd);

    // ── Step 1: turning radius from commanded centre delta ───────────────────
    //    R = L / tan(|δ_cmd|)
    res.R = L / std::tan(abs_d);

    // ── Step 2: per-wheel angles (both carry sign of turn direction) ─────────
    //    δ_i = sign · atan( L / (R − W/2) )
    //    δ_o = sign · atan( L / (R + W/2) )
    res.delta_inner = sign * std::atan(L / (res.R - W / 2.0));
    res.delta_outer = sign * std::atan(L / (res.R + W / 2.0));

    // ── Step 3: assign inner/outer to left/right based on turn direction ─────
    //    Turn LEFT  (δ_cmd > 0): left  = inside  → larger angle
    //                             right = outside → smaller angle
    //    Turn RIGHT (δ_cmd < 0): right = inside  → larger magnitude
    //                             left  = outside → smaller magnitude
    if (delta_cmd > 0.0) {
        res.delta_fl = res.delta_inner;   // left front is the inner wheel
        res.delta_fr = res.delta_outer;   // right front is the outer wheel
    } else {
        res.delta_fl = res.delta_outer;   // left front is the outer wheel
        res.delta_fr = res.delta_inner;   // right front is the inner wheel
    }

    return res;
}

}  // namespace smartport
