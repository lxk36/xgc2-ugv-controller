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
    // |delta| <= this bound (metres). Every extreme is enforced in the QP;
    // no lateral-velocity measurement or lateral command is introduced.
    double lateral_velocity_per_yaw_bound{0.0};
    Limits limits;
    // Body FLU [vx, vy, omega]. previous is the last executed command, not
    // an independently limited nominal command. Unicycle vy must be zero.
    Eigen::Vector3d nominal{Eigen::Vector3d::Zero()};
    Eigen::Vector3d previous{Eigen::Vector3d::Zero()};
    // false means observed stationary and not controlled by this solve.
    // Missing, stale, or uncontrolled moving peers cannot be marked inactive.
    bool active{true};
    // Require an exact zero command while retaining this active robot's
    // disturbance barriers. The zero must also satisfy the slew bounds.
    bool stop_requested{false};
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

struct FilterConfig {
    double dt{0.02};
    double barrier_gain{1.0};
    double clearance{0.10};
    // Additional geometric reserve. This is not an inferred braking bound.
    double uncertainty_margin{0.0};
    // Assumed bound on each active covering disk's unmodelled world velocity.
    double velocity_uncertainty{0.0};
    double smoothing_weight{0.05};
    Eigen::Vector3d objective_weights{1.0, 1.0, 0.2};
    int disk_count{2};
    int max_iterations{4000};
    double solver_tolerance{1.0e-7};
    double feasibility_tolerance{1.0e-6};
};

enum class Status {
    Solved,
    InvalidInput,
    InvalidGeometry,
    UnsafeInitialState,
    Infeasible,
    SolverFailure,
    ResidualViolation,
};

struct FilterResult {
    Status status{Status::InvalidInput};
    // Empty on failure: no zero or nominal command is labelled safe.
    std::vector<Eigen::Vector3d> commands;
    std::string detail;
    double min_clearance{std::numeric_limits<double>::infinity()};
    double max_constraint_violation{std::numeric_limits<double>::infinity()};
    int solver_status{0};
    bool ok() const {
        return status == Status::Solved;
    }
};

struct FootprintDisk {
    Eigen::Vector2d center;
    Eigen::Matrix<double, 2, 3> velocity_map;
    double radius;
};

// N overlapping disks cover the entire rectangular footprint, including its
// corners. The velocity Jacobian includes rotation about the pose origin.
std::vector<FootprintDisk> coveringDisks(const Robot& robot, int count);

// Joint velocity CBF-QP. Hard constraints cover convex obstacles with known
// instantaneous body twist, every robot disk pair, the fence, velocity bounds,
// and command slew. Instantaneous obstacle velocity is a local Lie derivative
// term, not a certificate of an arbitrary future trajectory. Continuous-time
// safety requires initially safe geometry, feasible continuous enforcement, and
// the stated velocity model/residual bound. Sampled commands, transport delay,
// and physical stopping distance require additional validated reserve; this
// pointwise QP alone does not certify them. Slew constraints alone do not
// ensure recursive feasibility.
// A successful solve certifies the checked inequalities, not global arrival.
FilterResult solveSafetyFilter(const std::vector<Robot>& robots,
                               const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                               const FilterConfig& config);

}  // namespace ugv_reset_safety
