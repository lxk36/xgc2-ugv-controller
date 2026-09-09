#include <algorithm>
#include <cmath>

#include "mecanum_ugv_controller/common/types.h"
#include "xgc2_math/algebra/angle.hpp"

namespace mecanum_ugv_controller {

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

void worldVelocityToBody(double yaw, double v_wx, double v_wy, double& v_bx, double& v_by) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    v_bx = c * v_wx + s * v_wy;
    v_by = -s * v_wx + c * v_wy;
}

void boxSaturateCommand(double& linear_x, double& linear_y, double& angular_z,
                        double max_linear_speed, double max_yaw_rate) {
    linear_x = clamp(linear_x, -max_linear_speed, max_linear_speed);
    linear_y = clamp(linear_y, -max_linear_speed, max_linear_speed);
    angular_z = clamp(angular_z, -max_yaw_rate, max_yaw_rate);
}

double headingRateToTarget(double yaw, double target_yaw, double kp_yaw, double max_yaw_rate) {
    const double error = xgc2_math::shortestAngularDistance(yaw, target_yaw);
    return clamp(kp_yaw * error, -max_yaw_rate, max_yaw_rate);
}

HolonomicTrackOutput computeHolonomicTrackCommand(const UgvState& state,
                                                  const WorldVelocityReference& reference,
                                                  const ControllerConfig& config) {
    HolonomicTrackOutput output;
    if (!reference.valid || !finitePose(state) || !std::isfinite(reference.vx) ||
        !std::isfinite(reference.vy)) {
        return output;
    }
    worldVelocityToBody(state.yaw, reference.vx, reference.vy, output.linear_x, output.linear_y);
    output.angular_z = headingRateToTarget(state.yaw, config.heading_target_yaw,
                                           config.track_kp_yaw, config.max_yaw_rate);
    boxSaturateCommand(output.linear_x, output.linear_y, output.angular_z, config.max_linear_speed,
                       config.max_yaw_rate);
    return output;
}

}  // namespace mecanum_ugv_controller
