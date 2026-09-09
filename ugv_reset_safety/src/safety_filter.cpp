#include "ugv_reset_safety/safety_filter.h"

#include <osqp/osqp.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

namespace ugv_reset_safety {
namespace {

constexpr double kGeometryTolerance = 1.0e-10;

double cross(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() * b.y() - a.y() * b.x();
}

bool positive(double value) {
    return std::isfinite(value) && value > 0.0;
}
bool nonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool convexPolygon(const ConvexObstacle& obstacle) {
    const auto& p = obstacle.vertices;
    if (p.size() < 3) {
        return false;
    }
    double area = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        if (!p[i].allFinite()) {
            return false;
        }
        const auto edge = p[(i + 1) % p.size()] - p[i];
        if (!edge.allFinite() || edge.squaredNorm() <= kGeometryTolerance) {
            return false;
        }
        area += cross(p[i], p[(i + 1) % p.size()]);
    }
    if (!std::isfinite(area) || std::abs(area) <= kGeometryTolerance) {
        return false;
    }
    const double sign = area > 0.0 ? 1.0 : -1.0;
    // Checking every vertex against every edge also rejects self-crossing
    // polygons whose consecutive turns happen to have the same sign.
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto edge = p[(i + 1) % p.size()] - p[i];
        for (const auto& point : p) {
            if (sign * cross(edge, point - p[i]) < -kGeometryTolerance) {
                return false;
            }
        }
    }
    return true;
}

Eigen::Vector2d projectPolygon(const Eigen::Vector2d& point, const ConvexObstacle& obstacle) {
    const auto& p = obstacle.vertices;
    double area = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        area += cross(p[i], p[(i + 1) % p.size()]);
    }
    const double sign = area > 0.0 ? 1.0 : -1.0;
    bool inside = true;
    Eigen::Vector2d nearest = p.front();
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto edge = p[(i + 1) % p.size()] - p[i];
        if (sign * cross(edge, point - p[i]) < 0.0) {
            inside = false;
        }
        const double t = std::clamp((point - p[i]).dot(edge) / edge.squaredNorm(), 0.0, 1.0);
        const Eigen::Vector2d candidate = p[i] + t * edge;
        const double distance = (point - candidate).squaredNorm();
        if (distance < best) {
            best = distance;
            nearest = candidate;
        }
    }
    return inside ? point : nearest;
}

struct Constraint {
    Eigen::VectorXd row;
    double lower;
    double upper;
};

struct CscStorage {
    std::vector<c_int> columns;
    std::vector<c_int> rows;
    std::vector<c_float> values;
    csc matrix{};

    explicit CscStorage(const Eigen::MatrixXd& dense) {
        for (Eigen::Index j = 0; j < dense.cols(); ++j) {
            columns.push_back(static_cast<c_int>(values.size()));
            for (Eigen::Index i = 0; i < dense.rows(); ++i) {
                if (dense(i, j) != 0.0) {
                    rows.push_back(static_cast<c_int>(i));
                    values.push_back(dense(i, j));
                }
            }
        }
        columns.push_back(static_cast<c_int>(values.size()));
        matrix.nzmax = static_cast<c_int>(values.size());
        matrix.m = static_cast<c_int>(dense.rows());
        matrix.n = static_cast<c_int>(dense.cols());
        matrix.p = columns.data();
        matrix.i = rows.data();
        matrix.x = values.data();
        matrix.nz = -1;
    }
};

}  // namespace

std::vector<FootprintDisk> coveringDisks(const Robot& robot, int count) {
    std::vector<FootprintDisk> disks;
    if (count < 1 || !positive(robot.half_length) || !positive(robot.half_width) ||
        !std::isfinite(robot.yaw) || !robot.position.allFinite() ||
        !robot.body_center_offset.allFinite()) {
        return disks;
    }
    const double c = std::cos(robot.yaw);
    const double s = std::sin(robot.yaw);
    Eigen::Matrix2d rotation;
    rotation << c, -s, s, c;
    const double slice = robot.half_length / static_cast<double>(count);
    for (int k = 0; k < count; ++k) {
        const Eigen::Vector2d body_offset =
            robot.body_center_offset +
            Eigen::Vector2d(-robot.half_length + (2 * k + 1) * slice, 0.0);
        const Eigen::Vector2d offset = rotation * body_offset;
        FootprintDisk disk;
        disk.center = robot.position + offset;
        disk.radius = std::hypot(slice, robot.half_width);
        disk.velocity_map.setZero();
        disk.velocity_map.col(0) = rotation.col(0);
        if (robot.type == RobotType::Mecanum) {
            disk.velocity_map.col(1) = rotation.col(1);
        }
        disk.velocity_map.col(2) = Eigen::Vector2d(-offset.y(), offset.x());
        disks.push_back(disk);
    }
    return disks;
}

FilterResult solveSafetyFilter(const std::vector<Robot>& robots,
                               const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                               const FilterConfig& config) {
    FilterResult result;
    auto fail = [&](Status status, const std::string& detail) {
        result.status = status;
        result.detail = detail;
        result.commands.clear();
        return result;
    };
    if (robots.empty() || !positive(config.dt) || !positive(config.barrier_gain) ||
        !nonnegative(config.clearance) || !nonnegative(config.uncertainty_margin) ||
        !nonnegative(config.velocity_uncertainty) || !nonnegative(config.smoothing_weight) ||
        !config.objective_weights.allFinite() || (config.objective_weights.array() <= 0.0).any() ||
        config.disk_count < 2 || config.disk_count > 32 || config.max_iterations < 1 ||
        !positive(config.solver_tolerance) || !positive(config.feasibility_tolerance)) {
        return fail(Status::InvalidInput, "invalid filter configuration or empty robot roster");
    }
    if (fence.enabled &&
        (!std::isfinite(fence.xmin) || !std::isfinite(fence.xmax) || !std::isfinite(fence.ymin) ||
         !std::isfinite(fence.ymax) || fence.xmin >= fence.xmax || fence.ymin >= fence.ymax)) {
        return fail(Status::InvalidGeometry, "invalid fence");
    }
    for (const auto& obstacle : obstacles) {
        if (!convexPolygon(obstacle)) {
            return fail(Status::InvalidGeometry, "invalid convex obstacle: " + obstacle.id);
        }
    }
    const Eigen::Index n = static_cast<Eigen::Index>(3 * robots.size());
    std::vector<Constraint> constraints;
    std::vector<std::vector<FootprintDisk>> footprints;
    std::vector<std::vector<Eigen::Vector2d>> lateral_extremes;
    std::set<std::string> ids;
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd q(n);
    const double margin = config.clearance + config.uncertainty_margin;
    for (std::size_t i = 0; i < robots.size(); ++i) {
        const auto& robot = robots[i];
        const auto& limits = robot.limits;
        if (robot.id.empty() || !ids.insert(robot.id).second ||
            (robot.type != RobotType::Unicycle && robot.type != RobotType::Mecanum) ||
            !robot.position.allFinite() || !std::isfinite(robot.yaw) ||
            !robot.body_center_offset.allFinite() || !positive(robot.half_length) ||
            !positive(robot.half_width) || !robot.nominal.allFinite() ||
            !robot.previous.allFinite() || !nonnegative(robot.lateral_velocity_per_yaw_bound) ||
            (robot.type != RobotType::Unicycle && robot.lateral_velocity_per_yaw_bound != 0.0) ||
            !positive(limits.max_vx) || !nonnegative(limits.max_vy) ||
            !positive(limits.max_omega) || !positive(limits.accel_vx) ||
            !positive(limits.accel_vy) || !positive(limits.accel_omega) ||
            (!robot.active &&
             robot.previous.cwiseAbs().maxCoeff() > config.feasibility_tolerance) ||
            (robot.type == RobotType::Unicycle &&
             std::abs(robot.previous.y()) > config.feasibility_tolerance)) {
            return fail(Status::InvalidInput, "invalid robot or inactive moving peer: " + robot.id);
        }
        footprints.push_back(coveringDisks(robot, config.disk_count));
        if (robot.active && robot.lateral_velocity_per_yaw_bound > 0.0) {
            const Eigen::Vector2d lateral =
                robot.lateral_velocity_per_yaw_bound *
                Eigen::Vector2d(-std::sin(robot.yaw), std::cos(robot.yaw));
            lateral_extremes.push_back({lateral, -lateral});
        } else {
            lateral_extremes.push_back({Eigen::Vector2d::Zero()});
        }
        const Eigen::Vector3d caps(limits.max_vx,
                                   robot.type == RobotType::Unicycle ? 0.0 : limits.max_vy,
                                   limits.max_omega);
        const Eigen::Vector3d accelerations(limits.accel_vx, limits.accel_vy, limits.accel_omega);
        for (int axis = 0; axis < 3; ++axis) {
            const Eigen::Index column = static_cast<Eigen::Index>(3 * i + axis);
            const double weight = config.objective_weights[axis];
            P(column, column) = 2.0 * weight * (1.0 + config.smoothing_weight);
            q[column] = -2.0 * weight *
                        (robot.nominal[axis] + config.smoothing_weight * robot.previous[axis]);
            double lower =
                std::max(-caps[axis], robot.previous[axis] - accelerations[axis] * config.dt);
            double upper =
                std::min(caps[axis], robot.previous[axis] + accelerations[axis] * config.dt);
            if (!robot.active || robot.stop_requested) {
                lower = std::max(lower, 0.0);
                upper = std::min(upper, 0.0);
            }
            if (lower > upper) {
                return fail(Status::Infeasible,
                            "velocity and slew limits have empty intersection: " + robot.id);
            }
            Eigen::VectorXd row = Eigen::VectorXd::Zero(n);
            row[column] = 1.0;
            constraints.push_back({std::move(row), lower, upper});
        }
    }
    auto addBarrier = [&](Eigen::VectorXd row, double h, double clearance, double drift) {
        result.min_clearance = std::min(result.min_clearance, clearance);
        if (clearance < -config.feasibility_tolerance || !row.allFinite() || !std::isfinite(h) ||
            !std::isfinite(drift)) {
            return false;
        }
        constraints.push_back({std::move(row), -config.barrier_gain * h + drift, OSQP_INFTY});
        return true;
    };
    for (std::size_t i = 0; i < robots.size(); ++i) {
        const double drift = robots[i].active ? config.velocity_uncertainty : 0.0;
        for (const auto& disk : footprints[i]) {
            for (const auto& obstacle : obstacles) {
                const Eigen::Vector2d delta = disk.center - projectPolygon(disk.center, obstacle);
                const double distance = delta.norm();
                const double radius = disk.radius + margin;
                for (const auto& lateral : lateral_extremes[i]) {
                    Eigen::VectorXd row = Eigen::VectorXd::Zero(n);
                    row.segment<3>(3 * i) = 2.0 * disk.velocity_map.transpose() * delta;
                    row[3 * i + 2] += 2.0 * delta.dot(lateral);
                    if (!addBarrier(std::move(row), delta.squaredNorm() - radius * radius,
                                    distance - radius, 2.0 * distance * drift)) {
                        return fail(Status::UnsafeInitialState,
                                    "footprint intersects obstacle clearance: " + robots[i].id +
                                        "/" + obstacle.id);
                    }
                }
            }
            if (fence.enabled) {
                const double radius = disk.radius + margin;
                const Eigen::Vector2d normals[] = {
                    {1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}};
                const double distances[] = {
                    disk.center.x() - fence.xmin - radius, fence.xmax - disk.center.x() - radius,
                    disk.center.y() - fence.ymin - radius, fence.ymax - disk.center.y() - radius};
                for (int edge = 0; edge < 4; ++edge) {
                    for (const auto& lateral : lateral_extremes[i]) {
                        Eigen::VectorXd row = Eigen::VectorXd::Zero(n);
                        row.segment<3>(3 * i) = disk.velocity_map.transpose() * normals[edge];
                        row[3 * i + 2] += normals[edge].dot(lateral);
                        if (!addBarrier(std::move(row), distances[edge], distances[edge], drift)) {
                            return fail(Status::UnsafeInitialState,
                                        "footprint intersects fence clearance: " + robots[i].id);
                        }
                    }
                }
            }
        }
        for (std::size_t j = i + 1; j < robots.size(); ++j) {
            const double pair_drift =
                drift + (robots[j].active ? config.velocity_uncertainty : 0.0);
            for (const auto& first : footprints[i]) {
                for (const auto& second : footprints[j]) {
                    const Eigen::Vector2d delta = first.center - second.center;
                    const double distance = delta.norm();
                    const double radius = first.radius + second.radius + margin;
                    for (const auto& first_lateral : lateral_extremes[i]) {
                        for (const auto& second_lateral : lateral_extremes[j]) {
                            Eigen::VectorXd row = Eigen::VectorXd::Zero(n);
                            row.segment<3>(3 * i) = 2.0 * first.velocity_map.transpose() * delta;
                            row.segment<3>(3 * j) = -2.0 * second.velocity_map.transpose() * delta;
                            row[3 * i + 2] += 2.0 * delta.dot(first_lateral);
                            row[3 * j + 2] -= 2.0 * delta.dot(second_lateral);
                            if (!addBarrier(std::move(row), delta.squaredNorm() - radius * radius,
                                            distance - radius, 2.0 * distance * pair_drift)) {
                                return fail(Status::UnsafeInitialState,
                                            "robot footprint clearances overlap: " + robots[i].id +
                                                "/" + robots[j].id);
                            }
                        }
                    }
                }
            }
        }
    }
    const Eigen::Index m = static_cast<Eigen::Index>(constraints.size());
    Eigen::MatrixXd A(m, n);
    std::vector<c_float> lower(static_cast<std::size_t>(m));
    std::vector<c_float> upper(static_cast<std::size_t>(m));
    std::vector<c_float> linear(static_cast<std::size_t>(n));
    for (Eigen::Index i = 0; i < m; ++i) {
        A.row(i) = constraints[i].row.transpose();
        lower[i] = constraints[i].lower;
        upper[i] = constraints[i].upper;
    }
    for (Eigen::Index i = 0; i < n; ++i) {
        linear[i] = q[i];
    }
    if (!P.allFinite() || !A.allFinite() || !q.allFinite()) {
        return fail(Status::InvalidInput, "non-finite QP coefficients");
    }
    CscStorage sparse_p(P);
    CscStorage sparse_a(A);
    OSQPData data{};
    data.n = static_cast<c_int>(n);
    data.m = static_cast<c_int>(m);
    data.P = &sparse_p.matrix;
    data.A = &sparse_a.matrix;
    data.q = linear.data();
    data.l = lower.data();
    data.u = upper.data();
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.polish = 1;
    settings.eps_abs = config.solver_tolerance;
    settings.eps_rel = config.solver_tolerance;
    settings.max_iter = config.max_iterations;
    OSQPWorkspace* workspace = nullptr;
    const c_int setup_status = osqp_setup(&workspace, &data, &settings);
    if (setup_status != 0 || !workspace) {
        if (workspace) {
            osqp_cleanup(workspace);
        }
        return fail(Status::SolverFailure, "OSQP setup failed");
    }
    const c_int solve_status = osqp_solve(workspace);
    result.solver_status = static_cast<int>(workspace->info->status_val);
    if (solve_status != 0 || workspace->info->status_val != OSQP_SOLVED) {
        const bool infeasible = workspace->info->status_val == OSQP_PRIMAL_INFEASIBLE ||
                                workspace->info->status_val == OSQP_PRIMAL_INFEASIBLE_INACCURATE;
        const std::string detail = std::string("OSQP: ") + workspace->info->status;
        osqp_cleanup(workspace);
        return fail(infeasible ? Status::Infeasible : Status::SolverFailure, detail);
    }
    Eigen::VectorXd solution(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        solution[i] = workspace->solution->x[i];
    }
    osqp_cleanup(workspace);
    if (!solution.allFinite()) {
        return fail(Status::ResidualViolation, "non-finite solver output");
    }
    // The first n rows are the exact componentwise velocity/slew intersections,
    // including fixed zero channels. Native owners require strict box bounds.
    // Correct only tolerance-sized solver residue, then check every original
    // inequality on these exact published values; projection can alter a CBF.
    for (Eigen::Index i = 0; i < n; ++i) {
        const auto& bounds = constraints[static_cast<std::size_t>(i)];
        const double bounded = std::clamp(solution[i], bounds.lower, bounds.upper);
        const double correction = std::abs(solution[i] - bounded);
        if (correction > config.feasibility_tolerance) {
            result.max_constraint_violation = correction;
            return fail(Status::ResidualViolation,
                        "solver output exceeds a velocity or slew box beyond tolerance");
        }
        solution[i] = bounded;
    }
    result.max_constraint_violation = 0.0;
    for (const auto& constraint : constraints) {
        const double value = constraint.row.dot(solution);
        result.max_constraint_violation = std::max(
            {result.max_constraint_violation, constraint.lower - value, value - constraint.upper});
    }
    if (result.max_constraint_violation > config.feasibility_tolerance) {
        return fail(Status::ResidualViolation,
                    "solver output violates an original hard constraint");
    }
    result.commands.reserve(robots.size());
    for (std::size_t i = 0; i < robots.size(); ++i) {
        result.commands.push_back(solution.segment<3>(3 * i));
    }
    result.status = Status::Solved;
    result.detail = "all original CBF, velocity and slew inequalities checked";
    return result;
}

}  // namespace ugv_reset_safety
