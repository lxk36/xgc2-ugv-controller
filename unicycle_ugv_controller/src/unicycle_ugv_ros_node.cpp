#include "unicycle_ugv_controller/unicycle_ugv_ros_node.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include "unicycle_ugv_controller/output/cmd_vel_output_consumer.h"
#include "unicycle_ugv_controller/output/nmpc_output_consumer.h"

namespace unicycle_ugv_controller {
namespace {

constexpr uint32_t kMinQueueSize = 1U;

double finitePositiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

}  // namespace

UnicycleUgvRosNode::UnicycleUgvRosNode(ros::NodeHandle& nh)
    : nh_(nh),
      private_nh_("~"),
      controller_(state_),
      reset_client_(nh_, controller_.resetSession()),
      output_executor_(nh_) {
    loadParams();
    controller_.setConfig(config_);
    seedResetTarget();

    auto post_input_event = [this](::state_machine::Event event) {
        return controller_.postEvent(std::move(event));
    };

    output_dispatcher_.addConsumer(std::make_unique<CmdVelOutputConsumer>(
        nh_, output_executor_, controller_, cmd_vel_topic_, queue_size_));
    if (config_.tracking_strategy == TrackingStrategy::NMPC) {
        output_dispatcher_.addConsumer(
            std::make_unique<NmpcOutputConsumer>(nh_, controller_, post_input_event, queue_size_));
        reference_input_ = std::make_unique<ReferenceInputProducer>(
            nh_, controller_.referenceCache(), active_analytic_topic_, active_polynomial_topic_,
            active_sampled_topic_, post_input_event, queue_size_);
    } else {
        pva_reference_input_ = std::make_unique<PvaReferenceInputProducer>(
            nh_, controller_, pva_reference_topic_, post_input_event, queue_size_);
    }

    control_state_pub_ = nh_.advertise<std_msgs::String>(control_state_topic_, queue_size_);

    command_input_ = std::make_unique<CommandInputProducer>(nh_, post_input_event, queue_size_);
    state_input_ =
        std::make_unique<StateInputProducer>(nh_, state_, config_.state_source, state_topic_,
                                             platform_pose_topic_, post_input_event, queue_size_);
    reset_target_input_ = std::make_unique<ResetTargetInputProducer>(
        nh_, controller_, reset_pose_topic_, post_input_event, queue_size_);

    output_executor_.start();
    ROS_INFO(
        "[UnicycleUgvRosNode] Initialized: strategy=%s state_source=%s state=%s pose=%s "
        "pva=%s reset_pose=%s cmd_vel=%s",
        config_.tracking_strategy == TrackingStrategy::FLATNESS ? "flatness" : "nmpc",
        config_.state_source == StateSource::PLATFORM_POSE ? "platform_pose" : "state_estimator",
        state_topic_.c_str(), platform_pose_topic_.c_str(), pva_reference_topic_.c_str(),
        reset_pose_topic_.c_str(), cmd_vel_topic_.c_str());
}

UnicycleUgvRosNode::~UnicycleUgvRosNode() {
    output_executor_.stop();
}

void UnicycleUgvRosNode::run(double frequency_hz) {
    const double frequency = finitePositiveOr(frequency_hz, config_.control_rate_hz);
    ROS_INFO("[UnicycleUgvRosNode] Starting main loop at %.1f Hz", frequency);
    ros::WallRate rate(frequency);
    while (ros::ok()) {
        ros::spinOnce();
        updateOnce();
        rate.sleep();
    }
}

void UnicycleUgvRosNode::loadParams() {
    int queue_size = static_cast<int>(queue_size_);
    private_nh_.param("queue_size", queue_size, queue_size);
    queue_size_ = std::max(kMinQueueSize, static_cast<uint32_t>(std::max(1, queue_size)));
    private_nh_.param("state_estimate_topic", state_topic_, state_topic_);
    std::string state_source{"state_estimator"};
    private_nh_.param("state_source", state_source, state_source);
    if (state_source == "platform_pose" || state_source == "pose") {
        config_.state_source = StateSource::PLATFORM_POSE;
    } else if (state_source == "state_estimator" || state_source == "estimator") {
        config_.state_source = StateSource::STATE_ESTIMATOR;
    } else {
        ROS_WARN("[UnicycleUgvRosNode] Unknown state_source=%s; using state_estimator",
                 state_source.c_str());
        config_.state_source = StateSource::STATE_ESTIMATOR;
    }
    std::string strategy{"nmpc"};
    private_nh_.param("tracking_strategy", strategy, strategy);
    if (strategy == "flatness") {
        config_.tracking_strategy = TrackingStrategy::FLATNESS;
    } else if (strategy == "nmpc") {
        config_.tracking_strategy = TrackingStrategy::NMPC;
    } else {
        throw std::invalid_argument("Unknown tracking_strategy: " + strategy);
    }
    private_nh_.param("platform_pose_topic", platform_pose_topic_, platform_pose_topic_);
    private_nh_.param("reset_pose_topic", reset_pose_topic_, reset_pose_topic_);
    private_nh_.param("active_analytic_topic", active_analytic_topic_, active_analytic_topic_);
    private_nh_.param("active_polynomial_topic", active_polynomial_topic_,
                      active_polynomial_topic_);
    private_nh_.param("active_sampled_topic", active_sampled_topic_, active_sampled_topic_);
    private_nh_.param("pva_reference_topic", pva_reference_topic_, pva_reference_topic_);
    private_nh_.param("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);
    private_nh_.param("control_state_topic", control_state_topic_, control_state_topic_);
    private_nh_.param("status_publish_rate_hz", status_publish_rate_hz_, status_publish_rate_hz_);
    private_nh_.param("control_rate_hz", config_.control_rate_hz, config_.control_rate_hz);
    private_nh_.param("nmpc/control_period", config_.control_period, config_.control_period);
    private_nh_.param("nmpc/prediction_horizon", config_.prediction_horizon,
                      config_.prediction_horizon);
    private_nh_.param("state_timeout", config_.state_timeout, config_.state_timeout);
    private_nh_.param("nmpc/solve_timeout", config_.solve_timeout, config_.solve_timeout);
    private_nh_.param("nmpc/result_timeout", config_.result_timeout, config_.result_timeout);
    private_nh_.param("command_publish_rate_hz", config_.command_publish_rate_hz,
                      config_.command_publish_rate_hz);
    private_nh_.param("idle_cmd_rate_hz", config_.idle_cmd_rate_hz, config_.idle_cmd_rate_hz);
    private_nh_.param("auto_start_tracking", config_.auto_start_tracking,
                      config_.auto_start_tracking);
    private_nh_.param("reset/timeout", config_.reset_timeout, config_.reset_timeout);
    private_nh_.param("nmpc/request_rate_hz", config_.nmpc_request_rate_hz,
                      config_.nmpc_request_rate_hz);
    private_nh_.param("limits/max_linear_speed", config_.max_linear_speed,
                      config_.max_linear_speed);
    private_nh_.param("limits/min_linear_speed", config_.min_linear_speed,
                      config_.min_linear_speed);
    private_nh_.param("limits/max_angular_speed", config_.max_angular_speed,
                      config_.max_angular_speed);
    private_nh_.param("limits/max_linear_acceleration", config_.max_linear_acceleration,
                      config_.max_linear_acceleration);
    private_nh_.param("limits/max_angular_acceleration", config_.max_angular_acceleration,
                      config_.max_angular_acceleration);
    private_nh_.param("chassis/max_linear_speed", config_.chassis_max_linear_speed,
                      config_.chassis_max_linear_speed);
    private_nh_.param("chassis/max_yaw_rate", config_.chassis_max_yaw_rate,
                      config_.chassis_max_yaw_rate);
    private_nh_.param("flatness/kp", config_.flatness_kp, config_.flatness_kp);
    private_nh_.param("flatness/kv", config_.flatness_kv, config_.flatness_kv);
    private_nh_.param("flatness/lateral_response_length", config_.flatness_lateral_response_length,
                      config_.flatness_lateral_response_length);
    private_nh_.param("flatness/lateral_damping", config_.flatness_lateral_damping,
                      config_.flatness_lateral_damping);
    private_nh_.param("flatness/v_eps", config_.flatness_v_eps, config_.flatness_v_eps);
    private_nh_.param("filter/zeta", config_.filter_zeta, config_.filter_zeta);
    private_nh_.param("filter/wn", config_.filter_wn, config_.filter_wn);
    private_nh_.param("filter/dt_min", config_.velocity_dt_min, config_.velocity_dt_min);
    private_nh_.param("filter/dt_max", config_.velocity_dt_max, config_.velocity_dt_max);
    private_nh_.param("fence/x_min", config_.fence_x_min, config_.fence_x_min);
    private_nh_.param("fence/x_max", config_.fence_x_max, config_.fence_x_max);
    private_nh_.param("fence/y_min", config_.fence_y_min, config_.fence_y_min);
    private_nh_.param("fence/y_max", config_.fence_y_max, config_.fence_y_max);
    private_nh_.param("reset_initial_x", config_.reset_initial_x, config_.reset_initial_x);
    private_nh_.param("reset_initial_y", config_.reset_initial_y, config_.reset_initial_y);
    private_nh_.param("reset_initial_yaw", config_.reset_initial_yaw, config_.reset_initial_yaw);
    private_nh_.param("nmpc/weights/position_x", config_.nmpc_weights.position_x,
                      config_.nmpc_weights.position_x);
    private_nh_.param("nmpc/weights/position_y", config_.nmpc_weights.position_y,
                      config_.nmpc_weights.position_y);
    private_nh_.param("nmpc/weights/yaw", config_.nmpc_weights.yaw, config_.nmpc_weights.yaw);
    private_nh_.param("nmpc/weights/speed", config_.nmpc_weights.speed, config_.nmpc_weights.speed);
    private_nh_.param("nmpc/weights/accel", config_.nmpc_weights.accel, config_.nmpc_weights.accel);
    private_nh_.param("nmpc/weights/omega", config_.nmpc_weights.omega, config_.nmpc_weights.omega);
    private_nh_.param("nmpc/weights/angular_accel", config_.nmpc_weights.angular_accel,
                      config_.nmpc_weights.angular_accel);
    private_nh_.param("nmpc/weights/terminal_position_x", config_.nmpc_weights.terminal_position_x,
                      config_.nmpc_weights.terminal_position_x);
    private_nh_.param("nmpc/weights/terminal_position_y", config_.nmpc_weights.terminal_position_y,
                      config_.nmpc_weights.terminal_position_y);
    private_nh_.param("nmpc/weights/terminal_yaw", config_.nmpc_weights.terminal_yaw,
                      config_.nmpc_weights.terminal_yaw);
    private_nh_.param("nmpc/weights/terminal_speed", config_.nmpc_weights.terminal_speed,
                      config_.nmpc_weights.terminal_speed);

    config_.control_rate_hz = finitePositiveOr(config_.control_rate_hz, 500.0);
    config_.control_period = finitePositiveOr(config_.control_period, 0.1);
    config_.prediction_horizon = finitePositiveOr(config_.prediction_horizon, 1.0);
    config_.state_timeout = finitePositiveOr(config_.state_timeout, 0.2);
    config_.solve_timeout = finitePositiveOr(config_.solve_timeout, 0.05);
    config_.result_timeout = finitePositiveOr(config_.result_timeout, 0.1);
    config_.command_publish_rate_hz = finitePositiveOr(config_.command_publish_rate_hz, 30.0);
    config_.idle_cmd_rate_hz = finitePositiveOr(config_.idle_cmd_rate_hz, 5.0);
    config_.nmpc_request_rate_hz = finitePositiveOr(config_.nmpc_request_rate_hz, 100.0);
    config_.max_linear_speed = finitePositiveOr(config_.max_linear_speed, 3.0);
    if (!std::isfinite(config_.min_linear_speed) ||
        config_.min_linear_speed >= config_.max_linear_speed) {
        config_.min_linear_speed = -1.5;
    }
    config_.max_angular_speed = finitePositiveOr(config_.max_angular_speed, 2.5);
    config_.max_linear_acceleration = finitePositiveOr(config_.max_linear_acceleration, 2.0);
    config_.max_angular_acceleration = finitePositiveOr(config_.max_angular_acceleration, 3.0);
    config_.chassis_max_linear_speed = finitePositiveOr(config_.chassis_max_linear_speed, 1.05);
    config_.chassis_max_yaw_rate = finitePositiveOr(config_.chassis_max_yaw_rate, 1.05);
    if (!std::isfinite(config_.flatness_lateral_response_length) ||
        config_.flatness_lateral_response_length <= 0.0 ||
        !std::isfinite(config_.flatness_lateral_damping) ||
        config_.flatness_lateral_damping <= 0.0) {
        throw std::invalid_argument(
            "flatness lateral response length and damping must be positive and finite");
    }
    config_.flatness_kp = finitePositiveOr(config_.flatness_kp, 6.0);
    config_.flatness_kv = finitePositiveOr(config_.flatness_kv, 4.0);
    config_.flatness_v_eps = finitePositiveOr(config_.flatness_v_eps, 0.15);
    config_.filter_zeta = finitePositiveOr(config_.filter_zeta, 0.7071067811865476);
    config_.filter_wn = finitePositiveOr(config_.filter_wn, 31.41592653589793);
    config_.velocity_dt_min = finitePositiveOr(config_.velocity_dt_min, 1.0e-4);
    config_.velocity_dt_max = finitePositiveOr(config_.velocity_dt_max, 0.2);
    config_.nmpc_weights.position_x = finitePositiveOr(config_.nmpc_weights.position_x, 20.0);
    config_.nmpc_weights.position_y = finitePositiveOr(config_.nmpc_weights.position_y, 20.0);
    config_.nmpc_weights.yaw = finitePositiveOr(config_.nmpc_weights.yaw, 8.0);
    config_.nmpc_weights.speed = finitePositiveOr(config_.nmpc_weights.speed, 4.0);
    config_.nmpc_weights.accel = finitePositiveOr(config_.nmpc_weights.accel, 0.4);
    config_.nmpc_weights.omega = finitePositiveOr(config_.nmpc_weights.omega, 10.0);
    config_.nmpc_weights.angular_accel = finitePositiveOr(config_.nmpc_weights.angular_accel, 1.0);
    config_.nmpc_weights.terminal_position_x =
        finitePositiveOr(config_.nmpc_weights.terminal_position_x, 60.0);
    config_.nmpc_weights.terminal_position_y =
        finitePositiveOr(config_.nmpc_weights.terminal_position_y, 60.0);
    config_.nmpc_weights.terminal_yaw = finitePositiveOr(config_.nmpc_weights.terminal_yaw, 20.0);
    config_.nmpc_weights.terminal_speed =
        finitePositiveOr(config_.nmpc_weights.terminal_speed, 10.0);
    config_.reset_timeout = finitePositiveOr(config_.reset_timeout, 600.0);
    status_publish_rate_hz_ = finitePositiveOr(status_publish_rate_hz_, 5.0);
}

void UnicycleUgvRosNode::seedResetTarget() {
    ResetTarget target;
    target.x = config_.reset_initial_x;
    target.y = config_.reset_initial_y;
    target.yaw = wrapAngle(config_.reset_initial_yaw);
    target.valid = std::isfinite(target.x) && std::isfinite(target.y) && std::isfinite(target.yaw);
    if (target.valid) {
        controller_.setResetTarget(target);
    }
}

void UnicycleUgvRosNode::updateOnce() {
    controller_.update(ros::Time::now().toSec());
    if (!controller_.lastResetAdmissionMiss().empty() &&
        controller_.lastResetAdmissionMiss() != last_logged_reset_miss_) {
        last_logged_reset_miss_ = controller_.lastResetAdmissionMiss();
        ROS_ERROR("[UnicycleUgvRosNode] %s", last_logged_reset_miss_.c_str());
    }
    dispatchOutputEvents(controller_.stateMachine().currentOutputEvents());
    reset_client_.update({state_.x, state_.y, state_.yaw}, state_.stamp, controller_.healthReady());
    const auto control_state = controller_.stateMachine().currentState(region_type::CONTROL);
    const auto health_state = controller_.stateMachine().currentState(region_type::HEALTH);
    logStateChanges(control_state, health_state);
    publishStatusIfDue(ros::Time::now());
}

void UnicycleUgvRosNode::dispatchOutputEvents(const std::vector<::state_machine::Event>& events) {
    const auto result = output_dispatcher_.dispatch(events);
    for (const auto& event : result.unhandled_events) {
        ROS_WARN("[UnicycleUgvRosNode] Unhandled output event id: %u",
                 static_cast<unsigned>(event.id));
    }
    for (const auto& failure : result.failures) {
        ROS_WARN("[UnicycleUgvRosNode] Output consumer '%s' failed on event %u: %s",
                 failure.consumer_name.c_str(), static_cast<unsigned>(failure.event.id),
                 failure.message.c_str());
    }
}

void UnicycleUgvRosNode::publishStatusIfDue(const ros::Time& now) {
    if (now.isZero() || !control_state_pub_) {
        return;
    }
    const double period = 1.0 / status_publish_rate_hz_;
    if (!last_status_stamp_.isZero() && (now - last_status_stamp_).toSec() < period) {
        return;
    }
    last_status_stamp_ = now;
    std_msgs::String control_state;
    control_state.data = controller_.stateMachine().currentStateName(region_type::CONTROL);
    if (control_state.data.empty()) {
        control_state.data = "Unknown";
    }
    control_state_pub_.publish(control_state);
}

void UnicycleUgvRosNode::logStateChanges(::state_machine::StateId control_state,
                                         ::state_machine::StateId health_state) {
    if (control_state != last_logged_control_state_) {
        ROS_INFO("[UnicycleUgvRosNode] CONTROL state -> %s",
                 controller_.stateMachine().currentStateName(region_type::CONTROL).c_str());
        last_logged_control_state_ = control_state;
    }
    if (health_state != last_logged_health_state_) {
        ROS_INFO("[UnicycleUgvRosNode] HEALTH state -> %u", static_cast<unsigned>(health_state));
        last_logged_health_state_ = health_state;
    }
}

}  // namespace unicycle_ugv_controller
