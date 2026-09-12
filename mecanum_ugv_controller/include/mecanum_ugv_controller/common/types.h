#pragma once

#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#include <cstdint>
#include <limits>
#include <state_machine/state_machine.hpp>

namespace mecanum_ugv_controller {

struct ControllerConfig {
    double control_rate_hz{500.0};
    double state_timeout{0.2};
    double command_publish_rate_hz{30.0};
    double idle_cmd_rate_hz{5.0};
    double status_publish_rate_hz{5.0};
    bool auto_start_tracking{false};
    double heading_target_yaw{0.0};
    double track_kp_yaw{1.2};
    double reset_timeout{90.0};
    double max_linear_speed{1.0};  // FLU |vx|, |vy| after R(ψ)^T
    double max_yaw_rate{1.0};      // FLU |ω|
    double fence_x_min{-20.0};     // offset world ENU x
    double fence_x_max{20.0};
    double fence_y_min{-20.0};  // offset world ENU y
    double fence_y_max{20.0};
    double reset_initial_x{std::numeric_limits<double>::quiet_NaN()};
    double reset_initial_y{std::numeric_limits<double>::quiet_NaN()};
    double reset_initial_yaw{std::numeric_limits<double>::quiet_NaN()};
};

struct UgvState {
    ros::Time stamp;
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    bool received{false};
};

struct ControlCommand {
    ros::Time stamp;
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
    bool valid{false};
};

struct ResetTarget {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    bool valid{false};
};

struct WorldVelocityReference {
    ros::Time stamp;
    double vx{0.0};
    double vy{0.0};
    bool valid{false};
};

struct HolonomicTrackOutput {
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
};

namespace state_type {
constexpr uint32_t HealthMonitor = 100;
constexpr uint32_t SelfCheck = 1;
constexpr uint32_t Ready = 2;
constexpr uint32_t Custom1 = 3;
constexpr uint32_t Reset = 5;
}  // namespace state_type

namespace region_type {
constexpr uint32_t HEALTH = 1;
constexpr uint32_t CONTROL = 2;
}  // namespace region_type

namespace event_type {
constexpr uint32_t CUSTOM1_REQUESTED = 1;
constexpr uint32_t STOP_REQUESTED = 2;
constexpr uint32_t RESET_REQUESTED = 3;
constexpr uint32_t RESET_ARRIVED = 4;
constexpr uint32_t RESET_TIMEOUT = 5;
constexpr uint32_t RESET_REJECTED = 6;
constexpr uint32_t INPUT_STATE_UPDATED = 20;
constexpr uint32_t INPUT_REFERENCE_UPDATED = 21;
constexpr uint32_t HEALTH_READY = 40;
constexpr uint32_t HEALTH_UNHEALTHY = 41;
}  // namespace event_type

namespace output_event_type {
constexpr uint32_t PUBLISH_CMD_VEL = 10001;
constexpr uint32_t PUBLISH_ZERO_CMD_VEL = 10002;
}  // namespace output_event_type

namespace transition_priority {
constexpr int COMMAND = 50;
constexpr int AUTOMATIC = 20;
}  // namespace transition_priority

double wrapAngle(double value);
double yawFromQuaternion(double x, double y, double z, double w);
bool tryYawFromQuaternion(double x, double y, double z, double w, double& yaw);
bool finitePose(const UgvState& state);
bool stateFresh(const UgvState& state, const ros::Time& now, double timeout);
bool insideFence(const UgvState& state, const ControllerConfig& config);
double clamp(double value, double min_value, double max_value);
void worldVelocityToBody(double yaw, double v_wx, double v_wy, double& v_bx, double& v_by);
void boxSaturateCommand(double& linear_x, double& linear_y, double& angular_z,
                        double max_linear_speed, double max_yaw_rate);
double headingRateToTarget(double yaw, double target_yaw, double kp_yaw, double max_yaw_rate);

HolonomicTrackOutput computeHolonomicTrackCommand(const UgvState& state,
                                                  const WorldVelocityReference& reference,
                                                  const ControllerConfig& config);

}  // namespace mecanum_ugv_controller
