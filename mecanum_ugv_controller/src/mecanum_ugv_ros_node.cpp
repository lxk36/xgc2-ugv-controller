#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ugv_reset_safety/reset_client.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>
#include <utility>

#include "mecanum_ugv_controller/mecanum_ugv_controller.h"
#include "mecanum_ugv_controller/state_machine/periodic_gate.h"

namespace mecanum_ugv_controller {
namespace {

constexpr uint32_t kMinQueueSize = 1U;

double finitePositiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

std::string normalize(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

}  // namespace

class MecanumUgvRosNode {
   public:
    explicit MecanumUgvRosNode(ros::NodeHandle& nh)
        : nh_(nh),
          private_nh_("~"),
          controller_(state_),
          reset_client_(nh_, controller_.resetSession()) {
        loadParams();
        controller_.setConfig(config_);
        seedResetTarget();
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
        control_state_pub_ = nh_.advertise<std_msgs::String>(control_state_topic_, queue_size_);
        command_sub_ =
            nh_.subscribe("command", queue_size_, &MecanumUgvRosNode::commandCallback, this);
        pose_sub_ = nh_.subscribe(pose_topic_, queue_size_, &MecanumUgvRosNode::poseCallback, this);
        reset_pose_sub_ = nh_.subscribe(reset_pose_topic_, queue_size_,
                                        &MecanumUgvRosNode::resetPoseCallback, this);
        reference_twist_sub_ = nh_.subscribe(reference_twist_topic_, queue_size_,
                                             &MecanumUgvRosNode::referenceTwistCallback, this);
        ROS_INFO("[MecanumUgvRosNode] pose=%s reset_pose=%s reference=%s cmd_vel=%s rate=%.1f Hz",
                 pose_topic_.c_str(), reset_pose_topic_.c_str(), reference_twist_topic_.c_str(),
                 cmd_vel_topic_.c_str(), config_.control_rate_hz);
    }

    void run() {
        ros::WallRate rate(finitePositiveOr(config_.control_rate_hz, 500.0));
        while (ros::ok()) {
            ros::spinOnce();
            const double now = ros::Time::now().toSec();
            controller_.update(now);
            for (const auto& event : controller_.stateMachine().currentOutputEvents()) {
                if (event.id == output_event_type::PUBLISH_CMD_VEL) {
                    const auto command = makeTwist(controller_.command());
                    cmd_vel_pub_.publish(command);
                    controller_.resetSession().noteApplied(
                        {command.linear.x, command.linear.y, command.angular.z},
                        ros::Time::now().toNSec());
                } else if (event.id == output_event_type::PUBLISH_ZERO_CMD_VEL) {
                    cmd_vel_pub_.publish(geometry_msgs::Twist{});
                    controller_.resetSession().noteApplied({}, ros::Time::now().toNSec());
                }
            }
            reset_client_.update({state_.x, state_.y, state_.yaw}, state_.stamp,
                                 controller_.healthReady());
            if (status_gate_.due(now, 1.0 / config_.status_publish_rate_hz)) {
                std_msgs::String status;
                status.data = controller_.stateMachine().currentStateName(region_type::CONTROL);
                if (status.data.empty()) {
                    status.data = "Unknown";
                }
                control_state_pub_.publish(status);
            }
            rate.sleep();
        }
    }

   private:
    void loadParams() {
        int queue_size = static_cast<int>(queue_size_);
        private_nh_.param("queue_size", queue_size, queue_size);
        queue_size_ = std::max(kMinQueueSize, static_cast<uint32_t>(std::max(1, queue_size)));
        private_nh_.param("pose_topic", pose_topic_, pose_topic_);
        private_nh_.param("reset_pose_topic", reset_pose_topic_, reset_pose_topic_);
        private_nh_.param("reference_twist_topic", reference_twist_topic_, reference_twist_topic_);
        private_nh_.param("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);
        private_nh_.param("control_state_topic", control_state_topic_, control_state_topic_);
        private_nh_.param("control_rate_hz", config_.control_rate_hz, config_.control_rate_hz);
        private_nh_.param("state_timeout", config_.state_timeout, config_.state_timeout);
        private_nh_.param("command_publish_rate_hz", config_.command_publish_rate_hz,
                          config_.command_publish_rate_hz);
        private_nh_.param("idle_cmd_rate_hz", config_.idle_cmd_rate_hz, config_.idle_cmd_rate_hz);
        private_nh_.param("status_publish_rate_hz", config_.status_publish_rate_hz,
                          config_.status_publish_rate_hz);
        private_nh_.param("auto_start_tracking", config_.auto_start_tracking,
                          config_.auto_start_tracking);
        private_nh_.param("heading_target_yaw", config_.heading_target_yaw,
                          config_.heading_target_yaw);
        private_nh_.param("track/kp_yaw", config_.track_kp_yaw, config_.track_kp_yaw);
        private_nh_.param("reset/timeout", config_.reset_timeout, config_.reset_timeout);
        private_nh_.param("max_linear_speed", config_.max_linear_speed, config_.max_linear_speed);
        private_nh_.param("max_yaw_rate", config_.max_yaw_rate, config_.max_yaw_rate);
        private_nh_.param("fence/x_min", config_.fence_x_min, config_.fence_x_min);
        private_nh_.param("fence/x_max", config_.fence_x_max, config_.fence_x_max);
        private_nh_.param("fence/y_min", config_.fence_y_min, config_.fence_y_min);
        private_nh_.param("fence/y_max", config_.fence_y_max, config_.fence_y_max);
        private_nh_.param("reset_initial_x", config_.reset_initial_x, config_.reset_initial_x);
        private_nh_.param("reset_initial_y", config_.reset_initial_y, config_.reset_initial_y);
        private_nh_.param("reset_initial_yaw", config_.reset_initial_yaw,
                          config_.reset_initial_yaw);
        config_.control_rate_hz = finitePositiveOr(config_.control_rate_hz, 500.0);
        config_.state_timeout = finitePositiveOr(config_.state_timeout, 0.2);
        config_.command_publish_rate_hz = finitePositiveOr(config_.command_publish_rate_hz, 50.0);
        config_.idle_cmd_rate_hz = finitePositiveOr(config_.idle_cmd_rate_hz, 5.0);
        config_.status_publish_rate_hz = finitePositiveOr(config_.status_publish_rate_hz, 5.0);
        config_.reset_timeout = finitePositiveOr(config_.reset_timeout, 600.0);
        config_.track_kp_yaw = finitePositiveOr(config_.track_kp_yaw, 1.2);
        config_.max_linear_speed = finitePositiveOr(config_.max_linear_speed, 1.0);
        config_.max_yaw_rate = finitePositiveOr(config_.max_yaw_rate, 1.0);
        if (!std::isfinite(config_.heading_target_yaw)) {
            config_.heading_target_yaw = 0.0;
        }
    }

    void seedResetTarget() {
        ResetTarget target;
        target.x = config_.reset_initial_x;
        target.y = config_.reset_initial_y;
        target.yaw = wrapAngle(config_.reset_initial_yaw);
        target.valid =
            std::isfinite(target.x) && std::isfinite(target.y) && std::isfinite(target.yaw);
        if (target.valid) {
            controller_.setResetTarget(target);
        }
    }

    void commandCallback(const std_msgs::String::ConstPtr& msg) {
        if (!msg || msg->data.empty()) {
            return;
        }
        const std::string command = normalize(msg->data);
        ::state_machine::EventId id = 0;
        if (command == "reset") {
            id = event_type::RESET_REQUESTED;
        } else if (command == "track" || command == "tracking" || command == "custom" ||
                   command == "custom1" || command == "start") {
            id = event_type::CUSTOM1_REQUESTED;
        } else if (command == "hold" || command == "stop") {
            id = event_type::STOP_REQUESTED;
        } else {
            return;
        }
        ::state_machine::Event event(id, ::state_machine::EventTimestamp{ros::Time::now().toSec()});
        event.source = "command";
        event.category = ::state_machine::EventCategory::kInput;
        (void)controller_.postEvent(std::move(event));
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!msg) {
            return;
        }
        double yaw = 0.0;
        if (!tryYawFromQuaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                                  msg->pose.orientation.z, msg->pose.orientation.w, yaw)) {
            return;
        }
        if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y)) {
            return;
        }
        state_.x = msg->pose.position.x;
        state_.y = msg->pose.position.y;
        state_.yaw = yaw;
        state_.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        state_.received = true;
        ::state_machine::Event event(event_type::INPUT_STATE_UPDATED,
                                     ::state_machine::EventTimestamp{state_.stamp.toSec()});
        event.source = "platform_pose";
        event.category = ::state_machine::EventCategory::kInput;
        (void)controller_.postEvent(std::move(event));
    }

    void resetPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        if (!msg || !std::isfinite(msg->x) || !std::isfinite(msg->y) ||
            !std::isfinite(msg->theta)) {
            return;
        }
        ResetTarget target;
        target.x = msg->x;
        target.y = msg->y;
        target.yaw = wrapAngle(msg->theta);
        target.valid = true;
        controller_.setResetTarget(target);
    }

    void referenceTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        if (!msg || !std::isfinite(msg->twist.linear.x) || !std::isfinite(msg->twist.linear.y)) {
            return;
        }
        WorldVelocityReference reference;
        reference.stamp = ros::Time::now();
        reference.vx = msg->twist.linear.x;
        reference.vy = msg->twist.linear.y;
        reference.valid = true;
        controller_.setWorldReference(reference);
        ::state_machine::Event event(event_type::INPUT_REFERENCE_UPDATED,
                                     ::state_machine::EventTimestamp{reference.stamp.toSec()});
        event.source = "reference_twist";
        event.category = ::state_machine::EventCategory::kInput;
        (void)controller_.postEvent(std::move(event));
    }

    geometry_msgs::Twist makeTwist(const ControlCommand& command) {
        geometry_msgs::Twist msg;
        if (controller_.stateMachine().currentState(region_type::CONTROL) == state_type::Reset) {
            const auto feedback = controller_.resetSession().feedback(
                ros::Time::now().toNSec(), ugv_reset_safety::monotonicSeconds());
            if (!feedback.valid || feedback.status != ugv_reset_safety::ResetSession::RUNNING ||
                std::abs(feedback.command.x) > config_.max_linear_speed ||
                std::abs(feedback.command.y) > config_.max_linear_speed ||
                std::abs(feedback.command.yaw) > config_.max_yaw_rate) {
                return msg;
            }
            msg.linear.x = feedback.command.x;
            msg.linear.y = feedback.command.y;
            msg.angular.z = feedback.command.yaw;
            return msg;
        }
        if (!command.valid) {
            return msg;
        }
        msg.linear.x = command.linear_x;
        msg.linear.y = command.linear_y;
        msg.angular.z = command.angular_z;
        return msg;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    UgvState state_;
    MecanumUgvController controller_;
    ugv_reset_safety::ResetClient reset_client_;
    ControllerConfig config_{};
    uint32_t queue_size_{10U};
    std::string pose_topic_{"pose"};
    std::string reset_pose_topic_{"reset_pose"};
    std::string reference_twist_topic_{"alg/reference/twist"};
    std::string cmd_vel_topic_{"cmd_vel"};
    std::string control_state_topic_{"custom/statustext"};
    ros::Publisher cmd_vel_pub_;
    ros::Publisher control_state_pub_;
    ros::Subscriber command_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber reset_pose_sub_;
    ros::Subscriber reference_twist_sub_;
    PeriodicGate status_gate_{};
};

}  // namespace mecanum_ugv_controller

int main(int argc, char** argv) {
    ros::init(argc, argv, "mecanum_ugv_controller");
    ros::NodeHandle nh;
    mecanum_ugv_controller::MecanumUgvRosNode node(nh);
    node.run();
    return 0;
}
