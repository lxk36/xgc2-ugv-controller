#pragma once

#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#include <Eigen/Dense>
#include <cstdint>
#include <state_machine/state_machine.hpp>

namespace unicycle_ugv_controller {

enum class StateSource {
    STATE_ESTIMATOR = 0,
    PLATFORM_POSE = 1,
};

enum class TrackingStrategy {
    NMPC = 0,
    FLATNESS = 1,
};

struct NmpcCostWeights {
    double position_x{20.0};
    double position_y{20.0};
    double yaw{8.0};
    double speed{4.0};
    double accel{0.4};
    double omega{10.0};
    double angular_accel{1.0};
    double terminal_position_x{60.0};
    double terminal_position_y{60.0};
    double terminal_yaw{20.0};
    double terminal_speed{10.0};
};

inline bool operator==(const NmpcCostWeights& lhs, const NmpcCostWeights& rhs) {
    return lhs.position_x == rhs.position_x && lhs.position_y == rhs.position_y &&
           lhs.yaw == rhs.yaw && lhs.speed == rhs.speed && lhs.accel == rhs.accel &&
           lhs.omega == rhs.omega && lhs.angular_accel == rhs.angular_accel &&
           lhs.terminal_position_x == rhs.terminal_position_x &&
           lhs.terminal_position_y == rhs.terminal_position_y &&
           lhs.terminal_yaw == rhs.terminal_yaw && lhs.terminal_speed == rhs.terminal_speed;
}

inline bool operator!=(const NmpcCostWeights& lhs, const NmpcCostWeights& rhs) {
    return !(lhs == rhs);
}

struct ControllerConfig {
    double control_rate_hz{500.0};
    double control_period{0.1};
    double prediction_horizon{1.0};
    double state_timeout{0.2};
    double solve_timeout{0.05};
    double result_timeout{0.1};
    double command_publish_rate_hz{50.0};
    double idle_cmd_rate_hz{5.0};
    double nmpc_request_rate_hz{100.0};
    bool auto_start_tracking{false};
    StateSource state_source{StateSource::STATE_ESTIMATOR};
    TrackingStrategy tracking_strategy{TrackingStrategy::NMPC};
    double reset_timeout{45.0};
    double reset_arrive_position{0.05};
    double reset_kp_along{0.8};
    double reset_kp_heading{1.2};
    double chassis_max_linear_speed{1.05};
    double chassis_max_yaw_rate{1.05};
    double max_linear_speed{3.0};
    double min_linear_speed{-1.5};
    double max_angular_speed{2.5};
    double max_linear_acceleration{2.0};
    double max_angular_acceleration{3.0};
    double flatness_kp{6.0};
    double flatness_kv{4.0};
    double flatness_v_eps{0.15};
    double flatness_lateral_response_length{0.8};
    double flatness_lateral_damping{1.0};
    double filter_zeta{0.7071067811865476};
    double filter_wn{31.41592653589793};
    double velocity_dt_min{1.0e-4};
    double velocity_dt_max{0.2};
    double fence_x_min{-20.0};
    double fence_x_max{20.0};
    double fence_y_min{-20.0};
    double fence_y_max{20.0};
    double reset_initial_x{0.0};
    double reset_initial_y{0.0};
    double reset_initial_yaw{0.0};
    NmpcCostWeights nmpc_weights{};
};

struct UgvState {
    ros::Time stamp;
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double vx{0.0};
    double vy{0.0};
    double speed{0.0};
    double yaw_rate{0.0};
    uint8_t estimator_state{0U};
    uint32_t estimator_flags{0U};
    bool received{false};
    bool velocity_valid{false};
};

struct ControlCommand {
    ros::Time stamp;
    double linear_speed{0.0};
    double angular_speed{0.0};
    bool valid{false};
};

struct ResetTarget {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    bool valid{false};
};

struct WorldPvaReference {
    ros::Time stamp;
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double vx{0.0};
    double vy{0.0};
    double ax{0.0};
    double ay{0.0};
    bool valid{false};
};

struct UnicycleBezierPlan {
    double p0x{0.0};
    double p0y{0.0};
    double p1x{0.0};
    double p1y{0.0};
    double p2x{0.0};
    double p2y{0.0};
    double p3x{0.0};
    double p3y{0.0};
    double T{0.0};
    bool reverse{false};
    bool valid{false};
    bool already_arrived{false};
    bool fence_failed{false};
};

struct UnicycleResetSample {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double linear_speed{0.0};
    double angular_speed{0.0};
    bool valid{false};
};

struct UnicycleResetOutput {
    double linear_speed{0.0};
    double angular_speed{0.0};
    bool position_ok{false};
};

struct FlatnessCommandOutput {
    double linear_speed{0.0};
    double angular_speed{0.0};
    double accel{0.0};
    bool valid{false};
};

struct PoseVelocityFilter {
    double x1{0.0};
    double x2{0.0};
    bool initialized{false};
};

struct PoseVelocityEstimator {
    PoseVelocityFilter axis_x;
    PoseVelocityFilter axis_y;
    PoseVelocityFilter axis_yaw;
    double last_stamp{0.0};
    double last_yaw{0.0};
    bool have_pose{false};
    bool velocity_valid{false};
    double vx{0.0};
    double vy{0.0};
    double yaw_rate{0.0};
};

struct NmpcSolveResult {
    uint64_t sequence{0U};
    ros::Time stamp;
    bool success{false};
    int solver_status{-1};
    double solve_time_ms{0.0};
    ControlCommand command;
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
constexpr uint32_t TRACKING_REQUESTED = CUSTOM1_REQUESTED;
constexpr uint32_t STOP_REQUESTED = 2;
constexpr uint32_t HOLD_REQUESTED = STOP_REQUESTED;
constexpr uint32_t RESET_REQUESTED = 3;
constexpr uint32_t RESET_ARRIVED = 4;
constexpr uint32_t RESET_TIMEOUT = 5;
constexpr uint32_t RESET_PLAN_FAILED = 6;
constexpr uint32_t INPUT_STATE_UPDATED = 20;
constexpr uint32_t INPUT_REFERENCE_UPDATED = 21;
constexpr uint32_t INPUT_NMPC_SOLVE_SUCCEEDED = 23;
constexpr uint32_t INPUT_NMPC_SOLVE_FAILED = 24;
constexpr uint32_t INPUT_RESET_TARGET_UPDATED = 25;
constexpr uint32_t HEALTH_READY = 40;
constexpr uint32_t HEALTH_UNHEALTHY = 41;
}  // namespace event_type

namespace output_event_type {
constexpr uint32_t REQUEST_NMPC_SOLVE = 10000;
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
bool finiteState(const UgvState& state);
bool stateFresh(const UgvState& state, const ros::Time& now, double timeout);
bool insideFence(const UgvState& state, const ControllerConfig& config);
double clamp(double value, double min_value, double max_value);
void boxSaturateUnicycle(double& linear_speed, double& angular_speed, double max_linear_speed,
                         double max_yaw_rate);
bool usesPoseVelocityFilter(StateSource source);
double bodySpeedFromWorld(double yaw, double vx, double vy);

bool updatePoseVelocityFilter(PoseVelocityFilter& filter, double input, double dt, double wn,
                              double zeta, double dt_min, double dt_max);
bool updatePoseVelocityEstimator(PoseVelocityEstimator& estimator, double stamp, double x, double y,
                                 double yaw, const ControllerConfig& config);

WorldPvaReference liftWorldPva(const WorldPvaReference& sample, double now_sec);
bool worldPvaReady(const WorldPvaReference& sample);

UnicycleBezierPlan planUnicycleReset(const UgvState& state, const ResetTarget& goal,
                                     const ControllerConfig& config);
bool sampleUnicycleReset(const UnicycleBezierPlan& plan, double t_along,
                         UnicycleResetSample& sample);
UnicycleResetOutput trackUnicycleReset(const UgvState& state, const UnicycleBezierPlan& plan,
                                       double t_along, const ControllerConfig& config);

FlatnessCommandOutput computeFlatnessCommand(const UgvState& state,
                                             const WorldPvaReference& reference, double body_speed,
                                             double dt, const ControllerConfig& config);

}  // namespace unicycle_ugv_controller
