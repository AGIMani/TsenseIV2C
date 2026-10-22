#include "a3/Stroke.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace a3 {
namespace {

/**
 * 角度（度）→ 动子行程（mm），相对 0° 取正值。
 * C = cos(πθ/180)，S = sin(πθ/180)
 * x = b(1-C) + c S + sqrt(L²-(d-c)²) - sqrt(L²-(d-b S-c C)²)
 * x(0°)=0。90° 为该连杆行程上限。
 *
 * 1 四指 MCP；2 食/中/无名指 PIP 与拇指 MCP（F2）；
 * 3 四指 DIP 与拇指 DIP（F1）；4 小拇指 PIP。
 */
struct Link {
    double b, c, d, L, base, stroke_hi;
};

struct Joint {
    bool linear;
    double lo, hi;
    int dest;
    int link;
};

double stroke_mm(const Link &g, double deg)
{
    const double th = deg * (3.14159265358979323846 / 180.0);
    const double C = std::cos(th);
    const double S = std::sin(th);
    const double u = g.d - g.b * S - g.c * C;
    const double inner = g.L * g.L - u * u;
    return g.b * (1.0 - C) + g.c * S + g.base - std::sqrt(std::max(0.0, inner));
}

Link make_link(double b, double c, double d, double L)
{
    Link g{};
    g.b = b;
    g.c = c;
    g.d = d;
    g.L = L;
    const double dc = d - c;
    g.base = std::sqrt(L * L - dc * dc);
    g.stroke_hi = stroke_mm(g, 90.0);
    return g;
}

const Link kLinks[5] = {
    {},
    make_link(5.5, 12.0, 11.0, 8.0),
    make_link(4.0, 10.190119226, 11.0, 10.2),
    make_link(2.0, 7.998645498, 8.0, 8.6),
    make_link(2.0, 8.000119226, 8.0, 11.1),
};

const Joint kJoints[20] = {
    {false, 0.0, 2.09, -2, 0},
    {false, 0.0, 1.74, -1, 0},
    {true, 0.0, 1.5707963267948966, 17, 2},
    {true, 0.0, 1.5707963267948966, 16, 3},
    {true, 0.0, 1.5707963267948966, 13, 1},
    {false, -0.35, 0.35, 15, 0},
    {true, 0.0, 1.5707963267948966, 12, 2},
    {true, 0.0, 1.5707963267948966, 11, 3},
    {true, 0.0, 1.5707963267948966, 10, 1},
    {false, -0.35, 0.35, 14, 0},
    {true, 0.0, 1.5707963267948966, 9, 2},
    {true, 0.0, 1.5707963267948966, 8, 3},
    {true, 0.0, 1.5707963267948966, 5, 1},
    {false, -0.35, 0.35, 7, 0},
    {true, 0.0, 1.5707963267948966, 4, 2},
    {true, 0.0, 1.5707963267948966, 3, 3},
    {true, 0.0, 1.5707963267948966, 2, 1},
    {false, -0.35, 0.35, 6, 0},
    {true, 0.0, 1.5707963267948966, 1, 4},
    {true, 0.0, 1.5707963267948966, 0, 3},
};

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kLinQMax = 1.5707963267948966;
/** float32 在约 20 mm 处的舍入约 1e-6 mm；0.00001 mm 只盖住限位舍入。 */
constexpr double kFeedbackStrokeTolMm = 0.00001;
/** 约 20° 的 float32 单位末位远小于 1e-6 rad。 */
constexpr double kFeedbackAngleTolRad = 1e-6;
/** 与 a32_fast 相同：角度→行程的线性插值截断误差不超过 1e-9 m。 */
constexpr double kLutToleranceMm = 1e-6;
/** 行程→角度的线性插值截断误差不超过 1e-6°，约等于 float32 在 90° 附近的末位。 */
constexpr double kLutToleranceDeg = 1e-6;

/**
 * |x''(θ)| 在 [0, π/2] 上的闭式上界，单位 mm/rad²。
 * v = -b cos + c sin，w = b sin + c cos，|v| 与 |w| 不超过 hypot(b, c)。
 * u = d - b sin - c cos，极值在端点和 tanθ = b/c。
 */
double curvature_bound(const Link &g)
{
    const double bc = std::hypot(g.b, g.c);
    const double u0 = g.d - g.c;
    const double u90 = g.d - g.b;
    const double u_min = g.d - bc;
    const double umax = std::max(std::fabs(u_min), std::max(std::fabs(u0), std::fabs(u90)));
    const double rmin2 = g.L * g.L - umax * umax;
    if (!(rmin2 > 0.0) || !(bc > 0.0)) return -1.0;
    const double rmin = std::sqrt(rmin2);
    const double v2 = bc * bc;
    return bc + (v2 + umax * bc) / rmin + (umax * umax * v2) / (rmin2 * rmin);
}

struct Table {
    double scale = 0.0;
    std::vector<double> values;
    bool ok = false;
};

double lookup(const Table &t, double x)
{
    const double position = x * t.scale;
    const int last = static_cast<int>(t.values.size()) - 2;
    int i = static_cast<int>(position);
    if (i < 0) i = 0;
    if (i > last) i = last;
    double fraction = position - static_cast<double>(i);
    if (fraction < 0.0) fraction = 0.0;
    if (fraction > 1.0) fraction = 1.0;
    const auto at = static_cast<size_t>(i);
    return t.values[at] + fraction * (t.values[at + 1] - t.values[at]);
}

/** dx/dθ，单位 mm/rad。 */
double stroke_rate(const Link &g, double th)
{
    const double C = std::cos(th);
    const double S = std::sin(th);
    const double u = g.d - g.b * S - g.c * C;
    const double v = -g.b * C + g.c * S;
    const double w = g.b * S + g.c * C;
    const double r2 = g.L * g.L - u * u;
    if (!(r2 > 0.0)) return 0.0;
    return w + (u * v) / std::sqrt(r2);
}

/** |d²θ/ds²| 的上界，单位 rad/mm²。由采样斜率减去 |x''| 的区间余量得到。 */
double inverse_curvature(const Link &g)
{
    const double xpp = curvature_bound(g);
    const double span = 90.0 * kDeg;
    if (!(xpp > 0.0) || !(span > 0.0)) return -1.0;
    const int counts[] = {1024, 4096, 16384, 65536};
    for (int count : counts) {
        double lo = std::numeric_limits<double>::infinity();
        const double half = span / (2.0 * static_cast<double>(count));
        for (int n = 0; n <= count; ++n) {
            const double th = span * (static_cast<double>(n) / static_cast<double>(count));
            lo = std::min(lo, stroke_rate(g, th));
        }
        lo -= xpp * half;
        if (lo > 0.0) return xpp / (lo * lo * lo);
    }
    return -1.0;
}

Table make_table(const Link &g)
{
    Table table;
    const double curvature = curvature_bound(g);
    const double span = 90.0 * kDeg;
    if (!(curvature > 0.0) || !std::isfinite(curvature) || !(span > 0.0)) return table;
    const double raw = span * std::sqrt(curvature / (8.0 * kLutToleranceMm * 0.9));
    if (!std::isfinite(raw) || raw > 262144.0) return table;
    const int intervals = std::max(1, static_cast<int>(std::ceil(raw)));
    table.values.resize(static_cast<size_t>(intervals) + 1);
    for (int n = 0; n <= intervals; ++n) {
        const double deg = 90.0 * (static_cast<double>(n) / static_cast<double>(intervals));
        const double s = stroke_mm(g, deg);
        if (!std::isfinite(s) || s < -1e-9) return Table{};
        table.values[static_cast<size_t>(n)] = std::max(0.0, s);
    }
    table.scale = static_cast<double>(intervals) / span;
    table.ok = true;
    return table;
}

const Table &table_of(int link)
{
    static const Table ts[5] = {
        {},
        make_table(kLinks[1]),
        make_table(kLinks[2]),
        make_table(kLinks[3]),
        make_table(kLinks[4]),
    };
    return ts[link];
}

bool solve_linear(int link, double deg, double *mm)
{
    if (deg < 0.0) deg = 0.0;
    if (deg > 90.0) deg = 90.0;
    const Table &table = table_of(link);
    if (!table.ok) return false;
    const double s = lookup(table, deg * kDeg);
    const Link &g = kLinks[link];
    if (!std::isfinite(s) || s < -kFeedbackStrokeTolMm || s > g.stroke_hi + kFeedbackStrokeTolMm) return false;
    *mm = std::min(g.stroke_hi, std::max(0.0, s));
    return true;
}

bool angle_of_stroke(const Link &g, double mm, double *deg)
{
    const double y = mm - g.b - g.base;
    const double A = 2.0 * (y * g.b - g.d * g.c);
    const double B = -2.0 * (y * g.c + g.d * g.b);
    const double rhs = g.L * g.L - g.b * g.b - g.c * g.c - g.d * g.d - y * y;
    const double R = std::hypot(A, B);
    if (!(R > 1.0)) return false;
    const double ratio = rhs / R;
    if (std::fabs(ratio) > 1.0 + 1e-9) return false;
    const double phi = std::atan2(B, A);
    const double delta = std::acos(std::min(1.0, std::max(-1.0, ratio)));
    const double cands[2] = {phi + delta, phi - delta};
    double best_err = 1.0;
    double best_deg = 0.0;
    bool found = false;
    for (double q : cands) {
        q = std::remainder(q, 2.0 * 3.14159265358979323846);
        if (q < -1e-8 || q > kLinQMax + 1e-8) continue;
        q = std::min(kLinQMax, std::max(0.0, q));
        const double s = stroke_mm(g, q / kDeg);
        const double err = std::fabs(s - mm);
        if (err < best_err) {
            best_err = err;
            best_deg = q / kDeg;
            found = true;
        }
    }
    if (!found || best_err > 1e-6) return false;
    *deg = best_deg;
    return true;
}

Table make_inverse_table(const Link &g)
{
    Table table;
    const double curvature = inverse_curvature(g) / kDeg;
    const double span = g.stroke_hi;
    if (!(curvature > 0.0) || !std::isfinite(curvature) || !(span > 0.0)) return table;
    const double raw = span * std::sqrt(curvature / (8.0 * kLutToleranceDeg * 0.9));
    if (!std::isfinite(raw) || raw > 262144.0) return table;
    const int intervals = std::max(1, static_cast<int>(std::ceil(raw)));
    table.values.resize(static_cast<size_t>(intervals) + 1);
    for (int n = 0; n <= intervals; ++n) {
        const double mm = span * (static_cast<double>(n) / static_cast<double>(intervals));
        double deg = 0.0;
        if (!angle_of_stroke(g, mm, &deg)) return Table{};
        table.values[static_cast<size_t>(n)] = std::min(90.0, std::max(0.0, deg));
    }
    table.scale = static_cast<double>(intervals) / span;
    table.ok = true;
    return table;
}

const Table &inverse_table_of(int link)
{
    static const Table ts[5] = {
        {},
        make_inverse_table(kLinks[1]),
        make_inverse_table(kLinks[2]),
        make_inverse_table(kLinks[3]),
        make_inverse_table(kLinks[4]),
    };
    return ts[link];
}

bool inverse_linear(int link, double mm, double *deg)
{
    const Link &g = kLinks[link];
    if (!std::isfinite(mm)) return false;
    if (mm < -kFeedbackStrokeTolMm || mm > g.stroke_hi + kFeedbackStrokeTolMm) return false;
    mm = std::min(g.stroke_hi, std::max(0.0, mm));
    const Table &table = inverse_table_of(link);
    if (!table.ok) return false;
    const double a = lookup(table, mm);
    if (!std::isfinite(a)) return false;
    *deg = std::min(90.0, std::max(0.0, a));
    return true;
}

}  // namespace

bool anglesDegToActuator(const float angle_deg[kJointNum], float out[kJointNum]) noexcept
{
    if (!angle_deg || !out) return false;
    float tmp[kJointNum];
    for (int i = 0; i < kJointNum; ++i) {
        const double deg = static_cast<double>(angle_deg[i]);
        if (!std::isfinite(deg)) return false;
        const Joint &j = kJoints[i];
        if (!j.linear) {
            const double q = deg * kDeg;
            if (q < j.lo - 1e-10 || q > j.hi + 1e-10) return false;
            tmp[i] = static_cast<float>(std::min(j.hi, std::max(j.lo, q)) / kDeg);
            continue;
        }
        double mm = 0.0;
        if (!solve_linear(j.link, deg, &mm)) return false;
        tmp[i] = static_cast<float>(mm);
    }
    for (int i = 0; i < kJointNum; ++i) out[i] = tmp[i];
    return true;
}

bool actuatorToAnglesDeg(const float actuator[kJointNum], float angle_deg[kJointNum]) noexcept
{
    if (!actuator || !angle_deg) return false;
    float tmp[kJointNum];
    for (int i = 0; i < kJointNum; ++i) {
        const double raw = static_cast<double>(actuator[i]);
        if (!std::isfinite(raw)) return false;
        const Joint &j = kJoints[i];
        if (!j.linear) {
            const double q = raw * kDeg;
            if (q < j.lo - kFeedbackAngleTolRad || q > j.hi + kFeedbackAngleTolRad) return false;
            tmp[i] = static_cast<float>(std::min(j.hi, std::max(j.lo, q)) / kDeg);
            continue;
        }
        double deg = 0.0;
        if (!inverse_linear(j.link, raw, &deg)) return false;
        tmp[i] = static_cast<float>(deg);
    }
    for (int i = 0; i < kJointNum; ++i) angle_deg[i] = tmp[i];
    return true;
}

bool actuatorToHand(const float actuator[kJointNum], float self_out[kSelfMotorNum], float mit_out[kMitMotorNum]) noexcept
{
    if (!actuator || !self_out || !mit_out) return false;
    for (int i = 0; i < kJointNum; ++i) {
        if (!std::isfinite(static_cast<double>(actuator[i]))) return false;
        const int d = kJoints[i].dest;
        if (d >= 0) self_out[d] = actuator[i];
        else mit_out[-d - 1] = actuator[i];
    }
    return true;
}

}  // namespace a3
