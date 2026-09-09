#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "unicycle_ugv_controller/common/types.h"
#include "xgc2_math/algebra/angle.hpp"

namespace unicycle_ugv_controller {
namespace {

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
    const double lateral_position_error =
        -s * (reference.x - state.x) + c * (reference.y - state.y);
    const double lateral_velocity_error =
        -s * (reference.vx - state.vx) + c * (reference.vy - state.vy);
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
