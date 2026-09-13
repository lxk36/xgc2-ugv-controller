#pragma once

#include <Eigen/Core>
#include <limits>
#include <string>
#include <vector>

namespace ugv_reset_safety {

enum class RobotType { Unicycle, Mecanum };

struct Limits {
    double max_vx{0.4};
    double max_vy{0.4};
    double max_omega{0.5};
    double accel_vx{0.5};
    double accel_vy{0.5};
    double accel_omega{0.8};
};

struct Robot {
    std::string id;
    RobotType type{RobotType::Unicycle};
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    double yaw{0.0};
    double half_length{0.3};
    double half_width{0.3};
    Eigen::Vector2d body_center_offset{Eigen::Vector2d::Zero()};
    // Scout's uncommanded body lateral velocity may be delta * omega, with
    // |delta| <= this bound (metres);
    // no lateral-velocity measurement or lateral command is introduced.
    double lateral_velocity_per_yaw_bound{0.0};
    Limits limits;
    // Body FLU [vx, vy, omega]. previous is the last executed command, not
    // a previous proposal. Unicycle vy must be zero.
    Eigen::Vector3d command{Eigen::Vector3d::Zero()};
    Eigen::Vector3d previous{Eigen::Vector3d::Zero()};
    // false means observed stationary and not controlled by this solve.
    // Missing, stale, or uncontrolled moving peers cannot be marked inactive.
    bool active{true};
    // Arrived and measured stopped; retain this robot as a parked footprint.
    bool stop_requested{false};
    // The coordinator has accepted the pose and coast margin. Reach an applied
    // zero through the checked dynamic window before declaring this body parked.
    bool brake_requested{false};
    // DWA found an admissible sample this tick. False is fail-closed: zero and
    // reject, not an unrolled brake treated as a safe command.
    bool local_plan_feasible{true};
};

struct ConvexObstacle {
    std::string id;
    // Clockwise or counterclockwise, without a repeated closing vertex.
    std::vector<Eigen::Vector2d> vertices;
    // World-frame origin and twist of the parent body. Zero for static shapes.
    // The closest-point velocity is origin linear plus yaw rate cross the
    // planar lever arm; this is the instantaneous model, not a future path.
    Eigen::Vector2d origin{Eigen::Vector2d::Zero()};
    Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
    double omega{0.0};
};

struct Fence {
    bool enabled{true};
    double xmin{-20.0};
    double xmax{20.0};
    double ymin{-20.0};
    double ymax{20.0};
};

struct DwaConfig {
    double dt{0.02};
    double clearance{0.10};
    double uncertainty_margin{0.0};
    int disk_count{2};
    double feasibility_tolerance{1.0e-6};
};

struct FootprintDisk {
    Eigen::Vector2d center;
    double radius;
};

std::vector<FootprintDisk> coveringDisks(const Robot& robot, int count);

}  // namespace ugv_reset_safety
