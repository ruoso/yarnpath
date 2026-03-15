#include "curvature_utils.hpp"
#include <cmath>
#include <algorithm>

namespace yarnpath {

Vec3 safe_normalized(const Vec3& v, const Vec3& fallback) {
    float len = v.length();
    return (len > 1e-8f) ? v * (1.0f / len) : fallback;
}

bool has_valid_control_points(const CubicBezier& curve) {
    for (const auto& cp : curve.control_points) {
        if (std::isnan(cp.x) || std::isnan(cp.y) || std::isnan(cp.z) ||
            std::isinf(cp.x) || std::isinf(cp.y) || std::isinf(cp.z)) {
            return false;
        }
    }
    return true;
}

std::pair<float, float> compute_curvature_safe_magnitudes(
    const Vec3& d0_unit, const Vec3& d1_unit,
    const Vec3& chord, float max_k) {

    float dist = chord.length();
    if (dist < 1e-6f || max_k < 1e-8f) {
        return {0.0f, 0.0f};
    }

    // Precompute cross products
    Vec3 d0xD = d0_unit.cross(chord);
    Vec3 d0xd1 = d0_unit.cross(d1_unit);
    Vec3 d1xD = d1_unit.cross(chord);

    // Start with rule-of-thirds as baseline
    float m0 = dist / 3.0f;
    float m1 = dist / 3.0f;

    // Iteratively solve the coupled endpoint curvature constraints
    for (int iter = 0; iter < 3; ++iter) {
        // κ(0) = 6 * |d0×D - (m1/3)(d0×d1)| / m0²  ≤ max_k
        Vec3 num0_vec = d0xD - d0xd1 * (m1 / 3.0f);
        float num0 = num0_vec.length();
        if (num0 > 1e-8f) {
            float needed_m0 = std::sqrt(6.0f * num0 / max_k);
            m0 = std::max(m0, needed_m0);
        }

        // κ(1) = 6 * |d1×D + (m0/3)(d0×d1)| / m1²  ≤ max_k
        // (sign: d1×d0 = -(d0×d1), so d1×(-D+m0*d0/3) = -d1×D-(m0/3)(d0×d1)
        //  magnitude = |d1×D + (m0/3)(d0×d1)| )
        Vec3 num1_vec = d1xD + d0xd1 * (m0 / 3.0f);
        float num1 = num1_vec.length();
        if (num1 > 1e-8f) {
            float needed_m1 = std::sqrt(6.0f * num1 / max_k);
            m1 = std::max(m1, needed_m1);
        }
    }

    // Safety factor for interior curvature peaks (endpoints don't bound the whole curve).
    // The interior peak can be up to ~2x the endpoint curvature for S-curves,
    // so we use a generous factor to ensure compliance.
    m0 *= 2.0f;
    m1 *= 2.0f;

    // Cap at 2*dist to prevent self-intersecting control polygons
    m0 = std::min(m0, dist * 2.0f);
    m1 = std::min(m1, dist * 2.0f);

    return {m0, m1};
}

CubicBezier build_curvature_safe_hermite(
    const Vec3& start, const Vec3& start_dir,
    const Vec3& end, const Vec3& end_dir,
    float max_k) {

    Vec3 chord = end - start;
    float dist = chord.length();
    if (dist < 1e-6f) return CubicBezier(start, start, end, end);

    Vec3 d0 = safe_normalized(start_dir);
    Vec3 d1 = safe_normalized(end_dir);

    auto [m0, m1] = compute_curvature_safe_magnitudes(d0, d1, chord, max_k);
    return CubicBezier::from_hermite(start, d0 * m0, end, d1 * m1);
}

// ── Tridiagonal solver for Vec3 unknowns (Thomas algorithm) ──────────
// Solves  a[i]*U[i-1] + b[i]*U[i] + c[i]*U[i+1] = d[i]
// with a[0]=0, c[n-1]=0.
static std::vector<Vec3> solve_tridiagonal_vec3(
    const std::vector<float>& a,
    const std::vector<float>& b,
    const std::vector<float>& c,
    const std::vector<Vec3>&  d) {

    int n = static_cast<int>(d.size());
    if (n == 0) return {};
    if (n == 1) return { d[0] * (1.0f / b[0]) };

    // Forward elimination
    std::vector<float> cp(n);   // modified upper diagonal
    std::vector<Vec3>  dp(n);   // modified RHS

    cp[0] = c[0] / b[0];
    dp[0] = d[0] * (1.0f / b[0]);

    for (int i = 1; i < n; ++i) {
        float denom = b[i] - a[i] * cp[i - 1];
        if (std::abs(denom) < 1e-12f) denom = 1e-12f;  // guard
        float inv = 1.0f / denom;
        cp[i] = c[i] * inv;
        dp[i] = (d[i] - dp[i - 1] * a[i]) * inv;
    }

    // Back substitution
    std::vector<Vec3> x(n);
    x[n - 1] = dp[n - 1];
    for (int i = n - 2; i >= 0; --i) {
        x[i] = dp[i] - x[i + 1] * cp[i];
    }
    return x;
}

std::vector<CubicBezier> build_curvature_safe_hermite_chain(
    const std::vector<Vec3>& waypoints,
    const Vec3& entry_tangent,
    float /*max_k*/) {

    int N = static_cast<int>(waypoints.size());   // number of points
    int n = N - 1;                                 // number of segments
    if (n < 1) return {};

    // ── Chord lengths ────────────────────────────────────────────────
    std::vector<float> h(n);
    for (int j = 0; j < n; ++j) {
        h[j] = (waypoints[j + 1] - waypoints[j]).length();
        if (h[j] < 1e-8f) h[j] = 1e-8f;  // guard against zero-length chords
    }

    // ── Entry tangent ────────────────────────────────────────────────
    // The caller supplies M_0, the unit-interval tangent for the first
    // segment.  Convert to chord-length derivative: m_0 = M_0 / h_0.
    Vec3 M0 = entry_tangent;
    if (M0.length() < 1e-8f) {
        Vec3 chord0 = waypoints[1] - waypoints[0];
        M0 = (chord0.length() > 1e-8f) ? chord0 : Vec3(1, 0, 0);
    }
    Vec3 m0 = M0 * (1.0f / h[0]);

    // ── Single segment: analytic natural-end solution ────────────────
    if (n == 1) {
        Vec3 D = waypoints[1] - waypoints[0];
        // Natural end: m_1 = (3*D/h_0² - m_0/h_0) * h_0 / 2
        //            = (3*D/h_0 - m_0) / 2
        Vec3 m1 = (D * (3.0f / h[0]) - m0) * 0.5f;
        // Convert back to unit-interval tangents for from_hermite
        return { CubicBezier::from_hermite(waypoints[0], m0 * h[0],
                                           waypoints[1], m1 * h[0]) };
    }

    // ── Multi-segment: chord-length-parameterised clamped natural spline ─
    //
    // C2 continuity at interior waypoint k with non-uniform spacing:
    //
    //   (1/h_{k-1})·m_{k-1} + 2·(1/h_{k-1} + 1/h_k)·m_k + (1/h_k)·m_{k+1}
    //       = 3·(D_{k-1}/h_{k-1}² + D_k/h_k²)
    //
    // where h_k = |P_{k+1} - P_k|, D_k = P_{k+1} - P_k.
    //
    // Clamped start: m_0 = M_0/h_0 (given).
    // Natural end (S''=0 at last point):
    //   (1/h_{n-1})·m_{n-1} + (2/h_{n-1})·m_n = 3·D_{n-1}/h_{n-1}²
    //
    // Unknowns: U[i] = m_{i+1}, i = 0 … n-1.

    // Precompute secants D_k / h_k²
    std::vector<Vec3> Dh2(n);
    for (int j = 0; j < n; ++j) {
        Dh2[j] = (waypoints[j + 1] - waypoints[j]) * (1.0f / (h[j] * h[j]));
    }

    std::vector<float> a(n), b(n), c(n);
    std::vector<Vec3>  rhs(n);

    // First equation (k=1, m_0 moved to RHS)
    float inv_h0 = 1.0f / h[0];
    float inv_h1 = 1.0f / h[1];
    a[0] = 0.0f;
    b[0] = 2.0f * (inv_h0 + inv_h1);
    c[0] = inv_h1;
    rhs[0] = (Dh2[0] + Dh2[1]) * 3.0f - m0 * inv_h0;

    // Interior equations (k = 2 … n-1)
    for (int i = 1; i < n - 1; ++i) {
        int k = i + 1;  // interior waypoint index
        float inv_hk1 = 1.0f / h[k - 1];
        float inv_hk  = 1.0f / h[k];
        a[i] = inv_hk1;
        b[i] = 2.0f * (inv_hk1 + inv_hk);
        c[i] = inv_hk;
        rhs[i] = (Dh2[k - 1] + Dh2[k]) * 3.0f;
    }

    // Natural end equation
    float inv_hn1 = 1.0f / h[n - 1];
    a[n - 1] = inv_hn1;
    b[n - 1] = 2.0f * inv_hn1;
    c[n - 1] = 0.0f;
    rhs[n - 1] = Dh2[n - 1] * 3.0f;

    auto U = solve_tridiagonal_vec3(a, b, c, rhs);

    // Assemble full chord-length derivative array m[0..N-1]
    std::vector<Vec3> m(N);
    m[0] = m0;
    for (int i = 0; i < n; ++i) {
        m[i + 1] = U[i];
    }

    // ── Build Hermite curves ─────────────────────────────────────────
    // Convert chord-length derivatives back to unit-interval tangents:
    //   T_j(start of segment j) = h_j · m_j
    //   T_j(end of segment j)   = h_j · m_{j+1}
    std::vector<CubicBezier> curves;
    curves.reserve(n);
    for (int j = 0; j < n; ++j) {
        curves.push_back(CubicBezier::from_hermite(
            waypoints[j],     m[j]     * h[j],
            waypoints[j + 1], m[j + 1] * h[j]));
    }
    return curves;
}

}  // namespace yarnpath
