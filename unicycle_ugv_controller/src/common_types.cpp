#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "unicycle_ugv_controller/common/types.h"
#include "xgc2_math/algebra/angle.hpp"

namespace unicycle_ugv_controller {
namespace {

constexpr int kResetSampleCount = 48;
constexpr double kOmegaDenomEps = 1.0e-8;
constexpr double kMinResetDuration = 0.05;

struct Vec2 {
    double x{0.0};
    double y{0.0};
};

double hypot2(double x, double y) {
    return std::hypot(x, y);
}

Vec2 bezierPoint(const UnicycleBezierPlan& plan, double s) {
    const double u = 1.0 - s;
    const double uu = u * u;
    const double uuu = uu * u;
    const double ss = s * s;
    const double sss = ss * s;
    Vec2 p;
    p.x = uuu * plan.p0x + 3.0 * uu * s * plan.p1x + 3.0 * u * ss * plan.p2x + sss * plan.p3x;
    p.y = uuu * plan.p0y + 3.0 * uu * s * plan.p1y + 3.0 * u * ss * plan.p2y + sss * plan.p3y;
    return p;
}

Vec2 bezierDeriv(const UnicycleBezierPlan& plan, double s) {
    const double u = 1.0 - s;
    Vec2 d;
    d.x = 3.0 * u * u * (plan.p1x - plan.p0x) + 6.0 * u * s * (plan.p2x - plan.p1x) +
          3.0 * s * s * (plan.p3x - plan.p2x);
    d.y = 3.0 * u * u * (plan.p1y - plan.p0y) + 6.0 * u * s * (plan.p2y - plan.p1y) +
          3.0 * s * s * (plan.p3y - plan.p2y);
    return d;
}

Vec2 bezierSecondDeriv(const UnicycleBezierPlan& plan, double s) {
    const double u = 1.0 - s;
    Vec2 d;
    d.x = 6.0 * u * (plan.p2x - 2.0 * plan.p1x + plan.p0x) +
          6.0 * s * (plan.p3x - 2.0 * plan.p2x + plan.p1x);
    d.y = 6.0 * u * (plan.p2y - 2.0 * plan.p1y + plan.p0y) +
          6.0 * s * (plan.p3y - 2.0 * plan.p2y + plan.p1y);
    return d;
}

void expm2x2(const double a00, const double a01, const double a10, const double a11, double& e00,
             double& e01, double& e10, double& e11) {
    double s00 = a00;
    double s01 = a01;
    double s10 = a10;
    double s11 = a11;
    int squares = 0;
    double norm = std::fabs(a00) + std::fabs(a01) + std::fabs(a10) + std::fabs(a11);
    while (norm > 0.5) {
        s00 *= 0.5;
        s01 *= 0.5;
        s10 *= 0.5;
        s11 *= 0.5;
        norm *= 0.5;
        ++squares;
    }
    e00 = 1.0;
    e01 = 0.0;
    e10 = 0.0;
    e11 = 1.0;
    double t00 = 1.0;
    double t01 = 0.0;
    double t10 = 0.0;
    double t11 = 1.0;
    double factorial = 1.0;
    for (int k = 1; k <= 12; ++k) {
        const double n00 = t00 * s00 + t01 * s10;
        const double n01 = t00 * s01 + t01 * s11;
        const double n10 = t10 * s00 + t11 * s10;
        const double n11 = t10 * s01 + t11 * s11;
        t00 = n00;
        t01 = n01;
        t10 = n10;
        t11 = n11;
        factorial *= static_cast<double>(k);
        e00 += t00 / factorial;
        e01 += t01 / factorial;
        e10 += t10 / factorial;
        e11 += t11 / factorial;
    }
    for (int i = 0; i < squares; ++i) {
        const double n00 = e00 * e00 + e01 * e10;
        const double n01 = e00 * e01 + e01 * e11;
        const double n10 = e10 * e00 + e11 * e10;
        const double n11 = e10 * e01 + e11 * e11;
        e00 = n00;
        e01 = n01;
        e10 = n10;
        e11 = n11;
    }
}

bool kinematicsAt(const UnicycleBezierPlan& plan, double s, double T, double& v, double& omega,
                  double& x, double& y, double& yaw) {
    if (!(T > 0.0) || !std::isfinite(s)) {
        return false;
    }
    const Vec2 p = bezierPoint(plan, s);
    const Vec2 dp = bezierDeriv(plan, s);
    const Vec2 ddp = bezierSecondDeriv(plan, s);
    const double vx = dp.x / T;
    const double vy = dp.y / T;
    const double ax = ddp.x / (T * T);
    const double ay = ddp.y / (T * T);
    const double speed = hypot2(vx, vy);
    const double speed_sq = vx * vx + vy * vy;
    x = p.x;
    y = p.y;
    if (speed_sq < kOmegaDenomEps) {
        return false;
    }
    omega = (vx * ay - vy * ax) / speed_sq;
    const double tangent_yaw = std::atan2(vy, vx);
    yaw = plan.reverse ? wrapAngle(tangent_yaw + M_PI) : wrapAngle(tangent_yaw);
    const double heading_x = std::cos(yaw);
    const double heading_y = std::sin(yaw);
    v = std::copysign(speed, vx * heading_x + vy * heading_y);
    return std::isfinite(v) && std::isfinite(omega) && std::isfinite(yaw);
}

bool planFeasible(const UnicycleBezierPlan& plan, double T, const ControllerConfig& config) {
    if (!(T > 0.0) || !std::isfinite(T)) {
        return false;
    }
    for (int i = 0; i <= kResetSampleCount; ++i) {
        const double s = static_cast<double>(i) / static_cast<double>(kResetSampleCount);
        double v = 0.0;
        double omega = 0.0;
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        if (!kinematicsAt(plan, s, T, v, omega, x, y, yaw)) {
            if (i == 0 || i == kResetSampleCount) {
                continue;
            }
            return false;
        }
        if (std::fabs(v) > config.chassis_max_linear_speed + 1.0e-9) {
            return false;
        }
        if (std::fabs(omega) > config.chassis_max_yaw_rate + 1.0e-9) {
            return false;
        }
    }
    return true;
}

bool pathInsideFence(const UnicycleBezierPlan& plan, const ControllerConfig& config) {
    UgvState probe;
    probe.received = true;
    for (int i = 0; i <= kResetSampleCount; ++i) {
        const double s = static_cast<double>(i) / static_cast<double>(kResetSampleCount);
        const Vec2 p = bezierPoint(plan, s);
        probe.x = p.x;
        probe.y = p.y;
        probe.yaw = 0.0;
        if (!insideFence(probe, config)) {
            return false;
        }
    }
    return true;
}

UnicycleBezierPlan makeCandidate(const UgvState& state, const ResetTarget& goal, double length,
                                 bool reverse) {
    UnicycleBezierPlan plan;
    const double start_yaw = reverse ? wrapAngle(state.yaw + M_PI) : state.yaw;
    const double goal_yaw = reverse ? wrapAngle(goal.yaw + M_PI) : goal.yaw;
    const double c0 = std::cos(start_yaw);
    const double s0 = std::sin(start_yaw);
    const double c3 = std::cos(goal_yaw);
    const double s3 = std::sin(goal_yaw);
    plan.p0x = state.x;
    plan.p0y = state.y;
    plan.p1x = state.x + length * c0;
    plan.p1y = state.y + length * s0;
    plan.p2x = goal.x - length * c3;
    plan.p2y = goal.y - length * s3;
    plan.p3x = goal.x;
    plan.p3y = goal.y;
    plan.reverse = reverse;
    return plan;
}

bool searchDuration(UnicycleBezierPlan& plan, const ControllerConfig& config) {
    const double dist = hypot2(plan.p3x - plan.p0x, plan.p3y - plan.p0y);
    double t_hi =
        std::max(kMinResetDuration, dist / std::max(config.chassis_max_linear_speed, 1.0e-6));
    const double t_cap = config.reset_timeout > 0.0 ? config.reset_timeout : 45.0;
    if (!planFeasible(plan, t_hi, config)) {
        bool found = false;
        while (t_hi < t_cap) {
            t_hi = std::min(t_hi * 2.0, t_cap);
            if (planFeasible(plan, t_hi, config)) {
                found = true;
                break;
            }
            if (t_hi >= t_cap) {
                break;
            }
        }
        if (!found) {
            return false;
        }
    }
    double t_lo = kMinResetDuration;
    for (int i = 0; i < 24; ++i) {
        const double mid = 0.5 * (t_lo + t_hi);
        if (planFeasible(plan, mid, config)) {
            t_hi = mid;
        } else {
            t_lo = mid;
        }
    }
    plan.T = t_hi;
    plan.valid = planFeasible(plan, plan.T, config);
    return plan.valid;
}

}  // namespace

double wrapAngle(double value) {
    return xgc2_math::normalizeAngle(value);
}

double yawFromQuaternion(double x, double y, double z, double w) {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return wrapAngle(std::atan2(siny_cosp, cosy_cosp));
}

bool tryYawFromQuaternion(double x, double y, double z, double w, double& yaw) {
    const double norm_squared = x * x + y * y + z * z + w * w;
    if (!std::isfinite(norm_squared) || norm_squared <= 1.0e-12) {
        return false;
    }
    const double inverse_norm = 1.0 / std::sqrt(norm_squared);
    yaw = yawFromQuaternion(x * inverse_norm, y * inverse_norm, z * inverse_norm, w * inverse_norm);
    return std::isfinite(yaw);
}

bool finitePose(const UgvState& state) {
    return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.yaw);
}

bool finiteState(const UgvState& state) {
    return finitePose(state) && std::isfinite(state.speed) && std::isfinite(state.yaw_rate);
}

bool stateFresh(const UgvState& state, const ros::Time& now, double timeout) {
    constexpr double kFutureStampTolerance = 0.05;
    const double age = (now - state.stamp).toSec();
    return state.received && finitePose(state) && timeout > 0.0 && std::isfinite(age) &&
           age >= -kFutureStampTolerance && age <= timeout;
}

bool insideFence(const UgvState& state, const ControllerConfig& config) {
    if (!std::isfinite(config.fence_x_min) || !std::isfinite(config.fence_x_max) ||
        !std::isfinite(config.fence_y_min) || !std::isfinite(config.fence_y_max) ||
        config.fence_x_min >= config.fence_x_max || config.fence_y_min >= config.fence_y_max) {
        return false;
    }
    return state.x >= config.fence_x_min && state.x <= config.fence_x_max &&
           state.y >= config.fence_y_min && state.y <= config.fence_y_max;
}

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

void boxSaturateUnicycle(double& linear_speed, double& angular_speed, double max_linear_speed,
                         double max_yaw_rate) {
    linear_speed = clamp(linear_speed, -max_linear_speed, max_linear_speed);
    angular_speed = clamp(angular_speed, -max_yaw_rate, max_yaw_rate);
}

bool usesPoseVelocityFilter(StateSource source) {
    return source != StateSource::STATE_ESTIMATOR;
}

double bodySpeedFromWorld(double yaw, double vx, double vy) {
    return std::cos(yaw) * vx + std::sin(yaw) * vy;
}

bool updatePoseVelocityFilter(PoseVelocityFilter& filter, double input, double dt, double wn,
                              double zeta, double dt_min, double dt_max) {
    if (!std::isfinite(input) || !std::isfinite(dt) || !std::isfinite(wn) || !std::isfinite(zeta) ||
        wn <= 0.0 || zeta <= 0.0 || dt <= dt_min || dt > dt_max) {
        return false;
    }
    if (!filter.initialized) {
        filter.x1 = input;
        filter.x2 = 0.0;
        filter.initialized = true;
        return false;
    }
    const double a00 = 0.0;
    const double a01 = 1.0;
    const double a10 = -wn * wn;
    const double a11 = -2.0 * zeta * wn;
    const double b0 = 0.0;
    const double b1 = wn * wn;
    double ad00 = 0.0;
    double ad01 = 0.0;
    double ad10 = 0.0;
    double ad11 = 0.0;
    expm2x2(a00 * dt, a01 * dt, a10 * dt, a11 * dt, ad00, ad01, ad10, ad11);
    const double det = a00 * a11 - a01 * a10;
    if (std::fabs(det) <= 1.0e-18) {
        return false;
    }
    const double inv00 = a11 / det;
    const double inv01 = -a01 / det;
    const double inv10 = -a10 / det;
    const double inv11 = a00 / det;
    const double d00 = ad00 - 1.0;
    const double d01 = ad01;
    const double d10 = ad10;
    const double d11 = ad11 - 1.0;
    const double ia00 = inv00 * d00 + inv01 * d10;
    const double ia01 = inv00 * d01 + inv01 * d11;
    const double ia10 = inv10 * d00 + inv11 * d10;
    const double ia11 = inv10 * d01 + inv11 * d11;
    const double bd0 = ia00 * b0 + ia01 * b1;
    const double bd1 = ia10 * b0 + ia11 * b1;
    const double x1 = ad00 * filter.x1 + ad01 * filter.x2 + bd0 * input;
    const double x2 = ad10 * filter.x1 + ad11 * filter.x2 + bd1 * input;
    if (!std::isfinite(x1) || !std::isfinite(x2)) {
        return false;
    }
    filter.x1 = x1;
    filter.x2 = x2;
    return true;
}

bool updatePoseVelocityEstimator(PoseVelocityEstimator& estimator, double stamp, double x, double y,
                                 double yaw, const ControllerConfig& config) {
    if (!std::isfinite(stamp) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
        estimator.velocity_valid = false;
        return false;
    }
    if (!estimator.have_pose) {
        estimator.have_pose = true;
        estimator.last_stamp = stamp;
        estimator.last_yaw = yaw;
        estimator.axis_x = PoseVelocityFilter{};
        estimator.axis_y = PoseVelocityFilter{};
        estimator.axis_yaw = PoseVelocityFilter{};
        estimator.axis_x.x1 = x;
        estimator.axis_y.x1 = y;
        estimator.axis_yaw.x1 = yaw;
        estimator.axis_x.initialized = true;
        estimator.axis_y.initialized = true;
        estimator.axis_yaw.initialized = true;
        estimator.velocity_valid = false;
        estimator.vx = 0.0;
        estimator.vy = 0.0;
        estimator.yaw_rate = 0.0;
        return false;
    }
    const double dt = stamp - estimator.last_stamp;
    const double unwrapped_yaw = estimator.last_yaw + wrapAngle(yaw - estimator.last_yaw);
    const bool ok_x =
        updatePoseVelocityFilter(estimator.axis_x, x, dt, config.filter_wn, config.filter_zeta,
                                 config.velocity_dt_min, config.velocity_dt_max);
    const bool ok_y =
        updatePoseVelocityFilter(estimator.axis_y, y, dt, config.filter_wn, config.filter_zeta,
                                 config.velocity_dt_min, config.velocity_dt_max);
    const bool ok_yaw = updatePoseVelocityFilter(estimator.axis_yaw, unwrapped_yaw, dt,
                                                 config.filter_wn, config.filter_zeta,
                                                 config.velocity_dt_min, config.velocity_dt_max);
    estimator.last_stamp = stamp;
    estimator.last_yaw = unwrapped_yaw;
    if (!ok_x || !ok_y || !ok_yaw) {
        estimator.velocity_valid = false;
        return false;
    }
    estimator.vx = estimator.axis_x.x2;
    estimator.vy = estimator.axis_y.x2;
    estimator.yaw_rate = estimator.axis_yaw.x2;
    estimator.velocity_valid = std::isfinite(estimator.vx) && std::isfinite(estimator.vy) &&
                               std::isfinite(estimator.yaw_rate);
    return estimator.velocity_valid;
}

WorldPvaReference liftWorldPva(const WorldPvaReference& sample, double now_sec) {
    WorldPvaReference out = sample;
    if (!sample.valid || !std::isfinite(now_sec) || !std::isfinite(sample.stamp.toSec())) {
        out.valid = false;
        return out;
    }
    const double tau = now_sec - sample.stamp.toSec();
    if (!std::isfinite(tau)) {
        out.valid = false;
        return out;
    }
    const double t = std::max(0.0, tau);
    out.x = sample.x + sample.vx * t + 0.5 * sample.ax * t * t;
    out.y = sample.y + sample.vy * t + 0.5 * sample.ay * t * t;
    out.vx = sample.vx + sample.ax * t;
    out.vy = sample.vy + sample.ay * t;
    out.ax = sample.ax;
    out.ay = sample.ay;
    return out;
}

bool worldPvaReady(const WorldPvaReference& sample) {
    return sample.valid && std::isfinite(sample.x) && std::isfinite(sample.y) &&
           std::isfinite(sample.vx) && std::isfinite(sample.vy) && std::isfinite(sample.ax) &&
           std::isfinite(sample.ay);
}

UnicycleBezierPlan planUnicycleReset(const UgvState& state, const ResetTarget& goal,
                                     const ControllerConfig& config) {
    UnicycleBezierPlan plan;
    if (!goal.valid || !finitePose(state) || !insideFence(state, config)) {
        plan.fence_failed = !insideFence(state, config) && finitePose(state);
        return plan;
    }
    const double dist = hypot2(goal.x - state.x, goal.y - state.y);
    if (dist <= config.reset_arrive_position) {
        plan.already_arrived = true;
        plan.valid = true;
        plan.p0x = state.x;
        plan.p0y = state.y;
        plan.p3x = goal.x;
        plan.p3y = goal.y;
        return plan;
    }
    double length = dist / 3.0;
    UnicycleBezierPlan best;
    best.T = config.reset_timeout > 0.0 ? config.reset_timeout + 1.0 : 1.0e9;
    bool any_feasible = false;
    for (int shorten = 0; shorten < 5; ++shorten) {
        for (const bool reverse : {false, true}) {
            UnicycleBezierPlan candidate = makeCandidate(state, goal, length, reverse);
            if (!pathInsideFence(candidate, config)) {
                continue;
            }
            if (!searchDuration(candidate, config)) {
                continue;
            }
            any_feasible = true;
            if (candidate.T < best.T) {
                best = candidate;
            }
        }
        if (any_feasible) {
            break;
        }
        length *= 0.5;
    }
    if (!any_feasible) {
        plan.fence_failed = true;
        return plan;
    }
    return best;
}

bool sampleUnicycleReset(const UnicycleBezierPlan& plan, double t_along,
                         UnicycleResetSample& sample) {
    sample = UnicycleResetSample{};
    if (!plan.valid || plan.already_arrived || !(plan.T > 0.0) || !std::isfinite(plan.T) ||
        !std::isfinite(t_along)) {
        return false;
    }
    const double t = std::max(0.0, t_along);
    const double s = clamp(t / plan.T, 0.0, 1.0);
    double v = 0.0;
    double omega = 0.0;
    if (!kinematicsAt(plan, s, plan.T, v, omega, sample.x, sample.y, sample.yaw)) {
        const Vec2 p = bezierPoint(plan, s);
        sample.x = p.x;
        sample.y = p.y;
        sample.yaw = 0.0;
        sample.linear_speed = 0.0;
        sample.angular_speed = 0.0;
        sample.valid = true;
        return true;
    }
    // Past the finite path, hold its endpoint rather than its last derivative.
    sample.linear_speed = t >= plan.T ? 0.0 : v;
    sample.angular_speed = t >= plan.T ? 0.0 : omega;
    sample.valid = true;
    return true;
}

UnicycleResetOutput trackUnicycleReset(const UgvState& state, const UnicycleBezierPlan& plan,
                                       double t_along, const ControllerConfig& config) {
    UnicycleResetOutput output;
    if (!finitePose(state) || !plan.valid) {
        return output;
    }
    const double dist_goal = hypot2(plan.p3x - state.x, plan.p3y - state.y);
    output.position_ok = dist_goal <= config.reset_arrive_position;
    if (output.position_ok || plan.already_arrived) {
        return output;
    }
    if (!std::isfinite(t_along) || !std::isfinite(plan.T) || !(plan.T > 0.0) ||
        !std::isfinite(dist_goal)) {
        return output;
    }
    if (t_along >= plan.T) {
        // Terminal XY approach stays inside Reset. A fixed tangent cannot remove
        // a lateral residual, and retaining terminal feedforward creates an offset
        // equilibrium. Point toward the goal, allowing reverse after overshoot.
        double heading_error =
            wrapAngle(std::atan2(plan.p3y - state.y, plan.p3x - state.x) - state.yaw);
        double direction = 1.0;
        if (std::fabs(heading_error) > 0.5 * M_PI) {
            direction = -1.0;
            heading_error = wrapAngle(heading_error - std::copysign(M_PI, heading_error));
        }
        output.linear_speed =
            direction * config.reset_kp_along * dist_goal * std::max(0.0, std::cos(heading_error));
        output.angular_speed = config.reset_kp_heading * heading_error;
        boxSaturateUnicycle(output.linear_speed, output.angular_speed,
                            config.chassis_max_linear_speed, config.chassis_max_yaw_rate);
        return output;
    }
    UnicycleResetSample sample;
    if (!sampleUnicycleReset(plan, t_along, sample) || !sample.valid) {
        return output;
    }
    const double ex = sample.x - state.x;
    const double ey = sample.y - state.y;
    const double heading_x = std::cos(sample.yaw);
    const double heading_y = std::sin(sample.yaw);
    output.linear_speed =
        sample.linear_speed + config.reset_kp_along * (ex * heading_x + ey * heading_y);
    output.angular_speed =
        sample.angular_speed + config.reset_kp_heading * wrapAngle(sample.yaw - state.yaw);
    boxSaturateUnicycle(output.linear_speed, output.angular_speed, config.chassis_max_linear_speed,
                        config.chassis_max_yaw_rate);
    return output;
}

FlatnessCommandOutput computeFlatnessCommand(const UgvState& state,
                                             const WorldPvaReference& reference, double body_speed,
                                             double dt, const ControllerConfig& config) {
    FlatnessCommandOutput output;
    if (!finitePose(state) || !reference.valid || !std::isfinite(body_speed) ||
        !std::isfinite(dt) || dt <= config.velocity_dt_min || dt > config.velocity_dt_max ||
        !state.velocity_valid) {
        return output;
    }
    const double ux = reference.ax + config.flatness_kv * (reference.vx - state.vx) +
                      config.flatness_kp * (reference.x - state.x);
    const double uy = reference.ay + config.flatness_kv * (reference.vy - state.vy) +
                      config.flatness_kp * (reference.y - state.y);
    const double c = std::cos(state.yaw);
    const double s = std::sin(state.yaw);
    output.accel = c * ux + s * uy;
    const double v_eps = std::max(config.flatness_v_eps, 1.0e-6);
    // A skid-steer chassis has lateral motion at the measured body origin while
    // turning. Fixed Cartesian derivative gain can feed that motion back with
    // positive yaw gain proportional to 1/v. Set transverse bandwidth by a
    // spatial response length; keep the measured world velocity and longitudinal
    // PVA feedback. This is one continuous tracking law, including at v=0.
    const double length = config.flatness_lateral_response_length;
    const double damping = config.flatness_lateral_damping;
    if (!std::isfinite(length) || length <= 0.0 || !std::isfinite(damping) || damping <= 0.0) {
        return output;
    }
    const double bandwidth = std::fabs(body_speed) / length;
    const double lateral_position_error = -s * (reference.x - state.x) + c * (reference.y - state.y);
    const double lateral_velocity_error = -s * (reference.vx - state.vx) + c * (reference.vy - state.vy);
    const double lateral_accel = -s * reference.ax + c * reference.ay +
        2.0 * damping * bandwidth * lateral_velocity_error +
        bandwidth * bandwidth * lateral_position_error;
    // Damped inverse of the dynamic-extension decoupling coefficient. Unlike a
    // signed epsilon denominator, this stays continuous during stop/reversal.
    // Exact transverse acceleration tracking is intentionally relaxed near rest.
    output.angular_speed = body_speed * lateral_accel / (body_speed * body_speed + v_eps * v_eps);
    output.linear_speed = body_speed + output.accel * dt;
    boxSaturateUnicycle(output.linear_speed, output.angular_speed, config.chassis_max_linear_speed,
                        config.chassis_max_yaw_rate);
    output.valid = std::isfinite(output.linear_speed) && std::isfinite(output.angular_speed);
    return output;
}

}  // namespace unicycle_ugv_controller
