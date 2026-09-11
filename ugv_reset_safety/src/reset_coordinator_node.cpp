#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ugv_reset_safety/ResetRequest.h>
#include <ugv_reset_safety/ResetResponse.h>
#include <ugv_reset_safety/fleet_guidance.h>
#include <ugv_reset_safety/fleet_schedule.h>
#include <ugv_reset_safety/reset_guidance.h>
#include <ugv_reset_safety/scene_projection.h>
#include <xgc2_geometry_msgs/SceneConsumerStatus.h>
#include <xgc2_geometry_msgs/SceneState.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace ugv_reset_safety {
namespace {
double number(const XmlRpc::XmlRpcValue& v, const std::string& key) {
    if (!v.hasMember(key)) {
        throw std::invalid_argument("missing fleet field: " + key);
    }
    const auto& a = v[key];
    double result;
    if (a.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        result = static_cast<double>(a);
    } else if (a.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        result = static_cast<int>(a);
    } else {
        throw std::invalid_argument("numeric fleet field required: " + key);
    }
    if (!std::isfinite(result)) {
        throw std::invalid_argument("nonfinite fleet field: " + key);
    }
    return result;
}
Eigen::Quaterniond quaternion(const geometry_msgs::Quaternion& q) {
    Eigen::Quaterniond r(q.w, q.x, q.y, q.z);
    if (!r.coeffs().allFinite() || !std::isfinite(r.norm()) || r.norm() < 1e-6) {
        throw std::invalid_argument("invalid quaternion");
    }
    r.normalize();
    return r;
}
double yaw(const Eigen::Quaterniond& q) {
    const auto r = q.toRotationMatrix();
    return std::atan2(r(1, 0), r(0, 0));
}
bool finitePose(const geometry_msgs::Pose2D& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta);
}
}  // namespace

class Coordinator {
    struct Entry {
        Robot robot;
        ResetGuidance guidance;
        ros::Subscriber request_sub, pose_sub, state_sub;
        ros::Publisher response_pub;
        ResetRequest request;
        ros::WallTime request_wall, pose_wall, state_wall;
        ros::Time pose_stamp;
        Eigen::Vector2d measured_position = Eigen::Vector2d::Zero();
        double measured_yaw = 0, measured_speed = 0, measured_omega = 0;
        bool have_pose = false, have_request = false, have_generation = false, planned = false,
             rejected = false;
        uint32_t generation = 0;
        std::string state;
        geometry_msgs::Pose2D frozen_target;
        std::string reason;
    };

   public:
    Coordinator() : private_("~") {
        private_.param("frequency", frequency_, 50.0);
        private_.param("input_timeout", timeout_, 0.15);
        private_.param("world_frame", world_frame_, std::string("world"));
        private_.param("scene_namespace", scene_namespace_, std::string("/xgc/scene"));
        private_.param("clearance", filter_.clearance, 0.08);
        private_.param("uncertainty_margin", filter_.uncertainty_margin, 0.03);
        private_.param("barrier_gain", filter_.barrier_gain, 1.0);
        private_.param("velocity_uncertainty", filter_.velocity_uncertainty, 0.0);
        fence_.enabled = true;
        if (!private_.getParam("fence/x_min", fence_.xmin) ||
            !private_.getParam("fence/x_max", fence_.xmax) ||
            !private_.getParam("fence/y_min", fence_.ymin) ||
            !private_.getParam("fence/y_max", fence_.ymax)) {
            throw std::invalid_argument("explicit fence required");
        }
        if (!std::isfinite(frequency_) || frequency_ < 10 || !std::isfinite(timeout_) ||
            timeout_ <= 0 || timeout_ > 0.5) {
            throw std::invalid_argument("invalid timing configuration");
        }
        XmlRpc::XmlRpcValue roster;
        if (!private_.getParam("robots", roster) ||
            roster.getType() != XmlRpc::XmlRpcValue::TypeArray || roster.size() < 1) {
            throw std::invalid_argument("explicit complete robots roster required");
        }
        std::set<std::string> ids;
        for (int i = 0; i < roster.size(); ++i) {
            const auto& r = roster[i];
            auto e = std::make_unique<Entry>();
            e->robot.id = static_cast<std::string>(r["namespace"]);
            if (e->robot.id.empty() || e->robot.id.find("..") != std::string::npos ||
                !ids.insert(e->robot.id).second) {
                throw std::invalid_argument("invalid or duplicate robot namespace");
            }
            const std::string type = static_cast<std::string>(r["type"]);
            if (type == "scout") {
                e->robot.type = RobotType::Unicycle;
            } else if (type == "mecanum") {
                e->robot.type = RobotType::Mecanum;
            } else {
                throw std::invalid_argument("unknown robot type");
            }
            e->robot.lateral_velocity_per_yaw_bound = number(r, "lateral_velocity_per_yaw_bound");
            e->robot.half_length = number(r, "length") / 2;
            e->robot.half_width = number(r, "width") / 2;
            e->robot.body_center_offset = {number(r, "body_offset_x"), number(r, "body_offset_y")};
            e->robot.limits.max_vx = number(r, "max_vx");
            e->robot.limits.max_vy = number(r, "max_vy");
            e->robot.limits.max_omega = number(r, "max_omega");
            e->robot.limits.accel_vx = number(r, "accel_vx");
            e->robot.limits.accel_vy = number(r, "accel_vy");
            e->robot.limits.accel_omega = number(r, "accel_omega");
            if (e->robot.half_length <= 0 || e->robot.half_width <= 0) {
                throw std::invalid_argument("nonpositive robot footprint");
            }
            const std::size_t n = entries_.size();
            const std::string ns = "/" + e->robot.id;
            e->response_pub = nh_.advertise<ResetResponse>(ns + "/reset/response", 1);
            e->request_sub = nh_.subscribe<ResetRequest>(
                ns + "/reset/request", 1,
                [this, n](const ResetRequest::ConstPtr& m) { request(n, *m); });
            e->pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(
                ns + "/pose", 1,
                [this, n](const geometry_msgs::PoseStamped::ConstPtr& m) { pose(n, *m); });
            e->state_sub = nh_.subscribe<std_msgs::String>(
                ns + "/custom/statustext",
                1, [this, n](const std_msgs::String::ConstPtr& m) {
                    entries_[n]->state = m->data;
                    entries_[n]->state_wall = ros::WallTime::now();
                });
            entries_.push_back(std::move(e));
        }
        scene_sub_ = nh_.subscribe(scene_namespace_ + "/snapshot", 1, &Coordinator::scene, this);
        scene_state_sub_ =
            nh_.subscribe(scene_namespace_ + "/state", 1, &Coordinator::sceneState, this);
        scene_status_pub_ = nh_.advertise<xgc2_geometry_msgs::SceneConsumerStatus>(
            scene_namespace_ + "/consumer_status", 1, true);
        consumer_generation_ = static_cast<uint32_t>(ros::WallTime::now().toNSec() & 0xffffffffu);
        if (consumer_generation_ == 0) {
            consumer_generation_ = 1;
        }
    }
    void run() {
        ros::WallRate rate(frequency_);
        last_tick_ = ros::WallTime::now();
        last_ros_tick_ = ros::Time::now();
        while (ros::ok()) {
            ros::spinOnce();
            tick();
            rate.sleep();
        }
    }

   private:
    void request(std::size_t n, const ResetRequest& r) {
        auto& e = *entries_[n];
        const auto now = ros::Time::now();
        if (r.header.stamp.isZero() || r.pose_stamp.isZero() || !finitePose(r.pose) ||
            !finitePose(r.target) || (now - r.header.stamp).toSec() < 0 ||
            (now - r.header.stamp).toSec() > timeout_ || (now - r.pose_stamp).toSec() < 0 ||
            (now - r.pose_stamp).toSec() > timeout_) {
            return;
        }
        const auto& a = r.applied_command;
        Eigen::Vector3d applied(a.linear.x, a.linear.y, a.angular.z);
        if (!applied.allFinite() || a.linear.z != 0 || a.angular.x != 0 || a.angular.y != 0 ||
            (e.robot.type == RobotType::Unicycle && a.linear.y != 0)) {
            return;
        }
        if (r.header.frame_id != world_frame_ || r.applied_stamp.isZero() ||
            r.applied_stamp > r.header.stamp || (now - r.applied_stamp).toSec() > timeout_) {
            return;
        }
        if (std::abs(applied.x()) > e.robot.limits.max_vx + 1e-6 ||
            std::abs(applied.y()) > e.robot.limits.max_vy + 1e-6 ||
            std::abs(applied.z()) > e.robot.limits.max_omega + 1e-6) {
            return;
        }
        if (e.have_request && r.header.stamp <= e.request.header.stamp) {
            return;
        }
        if (!e.have_generation || r.generation != e.generation) {
            // Session identifiers are monotonically increasing within an owner.
            // Wall expiration alone never resets the command/rate state.
            e.generation = r.generation;
            e.have_generation = true;
            e.planned = false;
            e.rejected = false;
            e.reason.clear();
            e.frozen_target = r.target;
            e.guidance.clear();
            passing_.clear();
            schedule_ready_ = false;
            schedule_.clear();
            scheduled_requested_.clear();
            selected_.clear();
            completed_.assign(entries_.size(), false);
            bool others_collecting = false;
            for (const auto& peer : entries_) {
                if (peer.get() != &e && peer->have_request) {
                    others_collecting = true;
                }
            }
            if (!others_collecting) {
                last_admission_ = ros::WallTime::now();
            }
            const auto wall = ros::WallTime::now();
            if (!e.have_pose || (wall - e.pose_wall).toSec() > timeout_ ||
                (now - e.pose_stamp).toSec() < 0 || (now - e.pose_stamp).toSec() > timeout_ ||
                (wall - e.state_wall).toSec() > timeout_) {
                e.rejected = true;
                e.reason = "reset pose/state unavailable at request";
            }
        }
        if (r.target.x != e.frozen_target.x || r.target.y != e.frozen_target.y ||
            r.target.theta != e.frozen_target.theta) {
            e.rejected = true;
            e.reason = "target changed inside reset session";
        }
        e.robot.previous = applied;
        e.request = r;
        e.have_request = true;
        e.request_wall = ros::WallTime::now();
    }
    void pose(std::size_t n, const geometry_msgs::PoseStamped& p) {
        auto& e = *entries_[n];
        try {
            if (p.header.frame_id != world_frame_ || p.header.stamp.isZero() ||
                !std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y) ||
                p.header.stamp > ros::Time::now()) {
                return;
            }
            const auto heading = yaw(quaternion(p.pose.orientation));
            Eigen::Vector2d position(p.pose.position.x, p.pose.position.y);
            if (e.have_pose) {
                const double dt = (p.header.stamp - e.pose_stamp).toSec();
                if (dt <= 0) {
                    return;
                }
                e.measured_speed =
                    dt <= timeout_ ? (position - e.measured_position).norm() / dt : 1e9;
                e.measured_omega = dt > timeout_ ? 1e9
                                                 : std::atan2(std::sin(heading - e.measured_yaw),
                                                              std::cos(heading - e.measured_yaw)) /
                                                       dt;
            } else {
                e.measured_speed = 1e9;
                e.measured_omega = 1e9;
            }
            e.measured_position = position;
            e.measured_yaw = heading;
            e.pose_stamp = p.header.stamp;
            e.pose_wall = ros::WallTime::now();
            e.have_pose = true;
        } catch (const std::exception& ex) {
            ROS_WARN_THROTTLE(2, "Reset pose rejected: %s", ex.what());
        }
    }
    void sceneState(const xgc2_geometry_msgs::SceneState::ConstPtr& state) {
        if (snapshot_.epoch.empty() || !scene_parsed_) {
            return;
        }
        if (state->epoch != snapshot_.epoch || state->revision != snapshot_.revision ||
            state->header.frame_id != world_frame_ || state->header.stamp.isZero()) {
            return;
        }
        if (!last_state_stamp_.isZero() && state->header.stamp <= last_state_stamp_) {
            return;
        }
        try {
            auto geometry = scene_projection::live(snapshot_, *state, world_frame_);
            bool moved = geometry.live.size() != obstacles_.size();
            if (!moved) {
                for (std::size_t i = 0; i < geometry.live.size(); ++i) {
                    if (geometry.live[i].id != obstacles_[i].id ||
                        geometry.live[i].vertices.size() != obstacles_[i].vertices.size()) {
                        moved = true;
                        break;
                    }
                    for (std::size_t j = 0; j < geometry.live[i].vertices.size(); ++j) {
                        if ((geometry.live[i].vertices[j] - obstacles_[i].vertices[j]).norm() >
                            1e-3) {
                            moved = true;
                            break;
                        }
                    }
                }
            }
            obstacles_ = std::move(geometry.live);
            occupancy_ = std::move(geometry.occupancy);
            scene_state_ = *state;
            last_state_stamp_ = state->header.stamp;
            scene_state_wall_ = ros::WallTime::now();
            scene_valid_ = true;
            scene_error_.clear();
            scene_capability_ = "ok";
            if (moved) {
                for (auto& e : entries_) {
                    e->planned = false;
                }
            }
        } catch (const std::exception& e) {
            scene_valid_ = false;
            scene_error_ = e.what();
            classifyCapability(scene_error_);
            ROS_WARN_THROTTLE(2, "Reset live scene rejected: %s", e.what());
        }
        publishStatus();
    }
    void scene(const xgc2_geometry_msgs::SceneSnapshot::ConstPtr& snapshot) {
        if (snapshot->epoch == snapshot_.epoch && snapshot->revision < snapshot_.revision) {
            return;
        }
        const bool definition_changed =
            !snapshot_.epoch.empty() &&
            (snapshot->epoch != snapshot_.epoch || snapshot->revision != snapshot_.revision);
        snapshot_ = *snapshot;
        scene_parsed_ = false;
        scene_valid_ = false;
        scene_error_.clear();
        obstacles_.clear();
        occupancy_.clear();
        last_state_stamp_ = ros::Time();
        scene_state_wall_ = ros::WallTime();
        try {
            std::set<std::string> ids;
            for (const auto& obstacle : snapshot_.obstacles) {
                scene_projection::validateObstacle(obstacle, &ids);
            }
            if (snapshot_.epoch.empty() || snapshot_.header.frame_id != world_frame_) {
                throw std::invalid_argument("scene epoch/frame mismatch");
            }
            scene_parsed_ = true;
            scene_capability_ = "ok";
            if (definition_changed) {
                for (auto& e : entries_) {
                    if (e->have_request) {
                        e->rejected = true;
                        e->reason = "scene changed; request a new reset after stopping";
                    }
                }
            }
        } catch (const std::exception& e) {
            scene_error_ = e.what();
            classifyCapability(scene_error_);
            ROS_WARN("Reset scene rejected: %s", e.what());
        }
        publishStatus();
    }
    void classifyCapability(const std::string& error) {
        scene_capability_ = error.find("unsupported") != std::string::npos ? "unsupported" : "";
    }
    void publishStatus() {
        if (snapshot_.epoch.empty()) {
            return;
        }
        xgc2_geometry_msgs::SceneConsumerStatus status;
        status.header.stamp = ros::Time::now();
        status.header.frame_id = world_frame_;
        status.epoch = snapshot_.epoch;
        status.revision = snapshot_.revision;
        status.consumer = "ugv-reset";
        status.generation = consumer_generation_;
        status.applied = scene_parsed_;
        status.capability =
            scene_capability_.empty() ? (scene_parsed_ ? "ok" : "") : scene_capability_;
        status.operational = scene_parsed_ && scene_valid_ && status.capability != "unsupported" &&
                             !scene_state_wall_.isZero() &&
                             (ros::WallTime::now() - scene_state_wall_).toSec() <= 0.5;
        status.success = status.applied;  // derived publish of applied, not a second authority
        status.message =
            scene_parsed_
                ? (scene_valid_
                       ? "applied live planar projection with finite-horizon occupancy"
                       : (scene_error_.empty() ? "waiting for matching scene state" : scene_error_))
                : scene_error_;
        scene_status_pub_.publish(status);
    }
    void reply(Entry& e, uint8_t status, const Eigen::Vector3d& command,
               const std::string& reason) {
        ResetResponse r;
        r.header = e.request.header;
        r.generation = e.request.generation;
        r.status = status;
        r.reason = reason;
        r.command.linear.x = command.x();
        r.command.linear.y = command.y();
        r.command.angular.z = command.z();
        e.response_pub.publish(r);
    }
    void rejectActive(const std::string& reason) {
        for (auto& e : entries_) {
            if (e->have_request && e->robot.active) {
                e->rejected = true;
                e->reason = reason;
                reply(*e, ResetResponse::REJECTED, Eigen::Vector3d::Zero(), reason);
            }
        }
    }
    void tick() {
        const auto wall = ros::WallTime::now();
        const auto now = ros::Time::now();
        const double wall_dt = (wall - last_tick_).toSec();
        last_tick_ = wall;
        const double dt = (now - last_ros_tick_).toSec();
        last_ros_tick_ = now;
        bool active = false;
        for (auto& e : entries_) {
            e->robot.active =
                e->have_request && (wall - e->request_wall).toSec() <= timeout_ && e->state == "Reset";
            active = active || e->robot.active;
        }
        publishStatus();
        if (!active) {
            return;
        }
        if (!scene_parsed_) {
            rejectActive("scene unavailable: " + scene_error_);
            return;
        }
        for (auto& e : entries_) {
            if (e->robot.active && e->rejected) {
                rejectActive(e->reason);
                return;
            }
        }
        if (scene_state_wall_.isZero() || (wall - scene_state_wall_).toSec() > 0.5) {
            rejectActive("shared scene heartbeat expired");
            return;
        }
        if (!scene_valid_) {
            rejectActive("scene unavailable: " + scene_error_);
            return;
        }
        if (dt <= 0 || dt > timeout_ || wall_dt <= 0 || wall_dt > timeout_) {
            rejectActive("coordinator deadline missed");
            return;
        }
        bool cohort_incomplete = false;
        for (const auto& e : entries_) {
            if (e->state != "Reset") {
                cohort_incomplete = true;
                break;
            }
        }
        if ((wall - last_admission_).toSec() < timeout_ && cohort_incomplete) {
            for (auto& e : entries_) {
                if (e->robot.active) {
                    reply(*e, ResetResponse::RUNNING, Eigen::Vector3d::Zero(),
                          "collecting reset batch");
                }
            }
            return;
        }
        for (auto& e : entries_) {
            if (!e->have_pose || (wall - e->pose_wall).toSec() > timeout_ ||
                (now - e->pose_stamp).toSec() < 0 || (now - e->pose_stamp).toSec() > timeout_ ||
                (wall - e->state_wall).toSec() > timeout_) {
                rejectActive("fleet pose/state unavailable");
                return;
            }
            if (e->state != "SelfCheck" && e->state != "Ready" && e->state != "Reset") {
                rejectActive("fleet member is outside reset/stop states");
                return;
            }
            if (!e->robot.active &&
                (e->measured_speed > 0.03 || std::abs(e->measured_omega) > 0.05)) {
                rejectActive("uncontrolled moving fleet member");
                return;
            }
            e->robot.position = e->measured_position;
            e->robot.yaw = e->measured_yaw;
            if (!e->robot.active) {
                e->robot.previous.setZero();
            }
            if (e->robot.active && e->rejected) {
                rejectActive(e->reason);
                return;
            }
        }
        std::vector<Robot> robots;
        std::vector<ResetTarget> targets;
        std::vector<bool> requested;
        for (const auto& e : entries_) {
            robots.push_back(e->robot);
            ResetTarget target;
            target.position = {e->frozen_target.x, e->frozen_target.y};
            target.yaw = e->frozen_target.theta;
            targets.push_back(target);
            requested.push_back(e->robot.active);
        }
        if (!schedule_ready_) {
            schedule_ = FleetSchedule(filter_.clearance + filter_.uncertainty_margin);
            const auto initialized = schedule_.initialize(robots, targets);
            if (!initialized.ok()) {
                rejectActive("reset schedule: " + initialized.detail);
                return;
            }
            scheduled_requested_ = requested;
            completed_.assign(robots.size(), false);
            selected_.assign(robots.size(), false);
            schedule_ready_ = true;
        }
        for (std::size_t i = 0; i < robots.size(); ++i) {
            if (scheduled_requested_[i] != requested[i] && !completed_[i]) {
                rejectActive("Reset batch membership changed before arrival");
                return;
            }
            if (completed_[i] && (!withinTargetTolerance(robots[i], targets[i]) ||
                                  entries_[i]->measured_speed > 0.03 ||
                                  std::abs(entries_[i]->measured_omega) > 0.05)) {
                rejectActive("completed Reset member moved; stop fleet and retry");
                return;
            }
        }
        const auto group = schedule_.select(completed_);
        if (!group.ok()) {
            rejectActive("reset schedule: " + group.detail);
            return;
        }
        std::vector<bool> selected(robots.size(), false);
        for (const auto i : group.selected) {
            selected[i] = true;
        }
        if (selected != selected_) {
            selected_ = selected;
            passing_.clear();
            for (auto& e : entries_) {
                e->planned = false;
            }
        }
        auto guidance_obstacles = occupancy_.empty() ? obstacles_ : occupancy_;
        const auto parked = parkedPeerObstacles(robots, selected);
        guidance_obstacles.insert(guidance_obstacles.end(), parked.begin(), parked.end());
        std::vector<GuidanceStatus> statuses(robots.size(), GuidanceStatus::Uninitialized);
        for (std::size_t i = 0; i < robots.size(); ++i) {
            auto& e = *entries_[i];
            // Nonselected owners remain in Reset but receive an exact zero.
            // They are stationary obstacles in the QP, not free actuators.
            if (!selected[i] &&
                (e.measured_speed > 0.03 || std::abs(e.measured_omega) > 0.05 ||
                 robots[i].previous.cwiseAbs().maxCoeff() > filter_.feasibility_tolerance)) {
                rejectActive("parked Reset member is moving");
                return;
            }
            robots[i].active = selected[i];
            robots[i].stop_requested = false;
            robots[i].nominal.setZero();
            if (!selected[i]) {
                continue;
            }
            if (!e.planned) {
                const auto planned =
                    e.guidance.setGoal(robots[i], targets[i], guidance_obstacles, fence_);
                if (planned.status == GuidanceStatus::InvalidInput ||
                    planned.status == GuidanceStatus::NoRoute) {
                    rejectActive(planned.message);
                    return;
                }
                e.planned = true;
            }
            const auto g = e.guidance.step(robots[i]);
            if (g.status == GuidanceStatus::InvalidInput || g.status == GuidanceStatus::NoRoute) {
                rejectActive(g.message);
                return;
            }
            robots[i].nominal = g.nominal;
            statuses[i] = g.status;
            robots[i].stop_requested =
                g.status == GuidanceStatus::Reached &&
                robots[i].previous.cwiseAbs().maxCoeff() <= filter_.feasibility_tolerance &&
                e.measured_speed <= 0.03 && std::abs(e.measured_omega) <= 0.05;
        }
        passing_.apply(robots, guidance_obstacles, fence_, filter_);
        filter_.dt = dt;
        const auto filtered = solveSafetyFilter(robots, obstacles_, fence_, filter_);
        if (!filtered.ok()) {
            rejectActive("safety filter: " + filtered.detail);
            return;
        }
        if ((ros::WallTime::now() - wall).toSec() > 1.0 / frequency_) {
            rejectActive("safety solve deadline missed");
            return;
        }
        for (std::size_t i = 0; i < entries_.size(); ++i) {
            auto& e = *entries_[i];
            if (!e.robot.active) {
                continue;
            }
            const bool arrived =
                robots[i].stop_requested && filtered.commands[i].isZero(0.0) &&
                e.robot.previous.cwiseAbs().maxCoeff() <= filter_.feasibility_tolerance &&
                e.measured_speed <= 0.03 && std::abs(e.measured_omega) <= 0.05;
            if (arrived || completed_[i]) {
                completed_[i] = true;
                reply(e, ResetResponse::ARRIVED, Eigen::Vector3d::Zero(), "");
            } else {
                reply(e, ResetResponse::RUNNING, filtered.commands[i], "");
            }
        }
    }
    ros::NodeHandle nh_, private_;
    std::vector<std::unique_ptr<Entry>> entries_;
    ros::Subscriber scene_sub_, scene_state_sub_;
    ros::Publisher scene_status_pub_;
    xgc2_geometry_msgs::SceneSnapshot snapshot_;
    xgc2_geometry_msgs::SceneState scene_state_;
    ros::Time last_state_stamp_;
    ros::WallTime scene_state_wall_;
    bool scene_parsed_ = false;
    bool scene_valid_ = false;
    std::string world_frame_, scene_namespace_, scene_error_, scene_capability_;
    std::vector<ConvexObstacle> obstacles_;
    std::vector<ConvexObstacle> occupancy_;
    Fence fence_;
    FilterConfig filter_;
    FleetGuidance passing_;
    FleetSchedule schedule_;
    bool schedule_ready_ = false;
    std::vector<bool> scheduled_requested_, selected_, completed_;
    ros::WallTime last_admission_;
    double frequency_ = 50, timeout_ = .15;
    uint32_t consumer_generation_ = 1;
    ros::WallTime last_tick_;
    ros::Time last_ros_tick_;
};
}  // namespace ugv_reset_safety
int main(int argc, char** argv) {
    ros::init(argc, argv, "ugv_reset_coordinator");
    try {
        ugv_reset_safety::Coordinator node;
        node.run();
    } catch (const std::exception& e) {
        ROS_FATAL("Reset coordinator startup failed: %s", e.what());
        return 1;
    }
    return 0;
}
