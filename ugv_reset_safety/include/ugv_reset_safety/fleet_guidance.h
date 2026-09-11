#pragma once
#include <ugv_reset_safety/reset_guidance.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ugv_reset_safety {

// FleetGuidance is the online local-planning layer between geometric route
// guidance and the hard joint CBF-QP. The geometric planner decides where a
// robot should go; this class decides which immediately reachable body twist is
// worth proposing over the next short horizon. Every proposal is rolled out
// with the chassis kinematics, full covering-disk footprint, static/dynamic
// scene occupancy supplied by the coordinator, peer motion, fence limits, and
// an explicit braking tail. The final CBF-QP remains authoritative.
class FleetGuidance {
    struct Passage {
        ResetGuidance guidance;
        std::string partner;
        bool complete = false;
    };

    struct RolloutResult {
        bool safe = false;
        double min_clearance = std::numeric_limits<double>::infinity();
    };

    std::map<std::string, Passage> passages_;
    std::set<std::pair<std::string, std::string>> encounters_;
    std::size_t priority_cursor_ = 0;

    static constexpr double kRolloutHorizon = 1.0;
    static constexpr double kRolloutStep = 0.10;
    static constexpr double kGeometryEpsilon = 1.0e-9;

    static double clamp(double value, double low, double high) {
        return std::max(low, std::min(high, value));
    }

    static double approachZero(double value, double delta) {
        if (value > delta) {
            return value - delta;
        }
        if (value < -delta) {
            return value + delta;
        }
        return 0.0;
    }

    static std::vector<double> axisSamples(double previous, double desired, double limit,
                                           double acceleration, double dt) {
        if (limit <= 0.0) {
            return {0.0};
        }
        const double reach = std::max(0.0, acceleration) * std::max(0.0, dt);
        const double low = std::max(-limit, previous - reach);
        const double high = std::min(limit, previous + reach);
        std::vector<double> values{clamp(desired, low, high), previous, low, high,
                                   0.5 * (low + high)};
        if (low <= 0.0 && high >= 0.0) {
            values.push_back(0.0);
        }
        std::sort(values.begin(), values.end());
        values.erase(std::unique(values.begin(), values.end(),
                                 [](double a, double b) { return std::abs(a - b) < 1.0e-9; }),
                     values.end());
        return values;
    }

    static void integrate(Robot* robot, const Eigen::Vector3d& command, double dt) {
        const double omega = command.z();
        const double mid_yaw = robot->yaw + 0.5 * omega * dt;
        const double c = std::cos(mid_yaw), s = std::sin(mid_yaw);
        const double vy = robot->type == RobotType::Unicycle ? 0.0 : command.y();
        robot->position.x() += (c * command.x() - s * vy) * dt;
        robot->position.y() += (s * command.x() + c * vy) * dt;
        robot->yaw =
            std::atan2(std::sin(robot->yaw + omega * dt), std::cos(robot->yaw + omega * dt));
    }

    static Robot predictConstant(const Robot& source, const Eigen::Vector3d& command, double time) {
        Robot result = source;
        if (time <= 0.0) {
            return result;
        }
        const double vy = source.type == RobotType::Unicycle ? 0.0 : command.y();
        const double omega = command.z();
        if (std::abs(omega) < 1.0e-8) {
            const double c = std::cos(source.yaw), s = std::sin(source.yaw);
            result.position.x() += (c * command.x() - s * vy) * time;
            result.position.y() += (s * command.x() + c * vy) * time;
        } else {
            const double phase = omega * time;
            const double a = std::sin(phase) / omega;
            const double b = (1.0 - std::cos(phase)) / omega;
            const Eigen::Vector2d body_delta(a * command.x() - b * vy, b * command.x() + a * vy);
            const double c = std::cos(source.yaw), s = std::sin(source.yaw);
            result.position.x() += c * body_delta.x() - s * body_delta.y();
            result.position.y() += s * body_delta.x() + c * body_delta.y();
        }
        result.yaw =
            std::atan2(std::sin(source.yaw + omega * time), std::cos(source.yaw + omega * time));
        return result;
    }

    static double pointSegmentDistance(const Eigen::Vector2d& point, const Eigen::Vector2d& a,
                                       const Eigen::Vector2d& b) {
        const Eigen::Vector2d edge = b - a;
        const double length_squared = edge.squaredNorm();
        if (length_squared <= kGeometryEpsilon * kGeometryEpsilon) {
            return (point - a).norm();
        }
        const double t = clamp((point - a).dot(edge) / length_squared, 0.0, 1.0);
        return (point - (a + t * edge)).norm();
    }

    static bool pointInside(const Eigen::Vector2d& point,
                            const std::vector<Eigen::Vector2d>& polygon) {
        bool inside = false;
        if (polygon.size() < 3) {
            return false;
        }
        for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
            const auto& a = polygon[i];
            const auto& b = polygon[j];
            const bool crosses =
                ((a.y() > point.y()) != (b.y() > point.y())) &&
                (point.x() < (b.x() - a.x()) * (point.y() - a.y()) / (b.y() - a.y()) + a.x());
            if (crosses) {
                inside = !inside;
            }
        }
        return inside;
    }

    static double pointPolygonDistance(const Eigen::Vector2d& point,
                                       const std::vector<Eigen::Vector2d>& polygon) {
        if (polygon.size() < 3 || pointInside(point, polygon)) {
            return 0.0;
        }
        double distance = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < polygon.size(); ++i) {
            distance = std::min(distance, pointSegmentDistance(point, polygon[i],
                                                               polygon[(i + 1) % polygon.size()]));
        }
        return distance;
    }

    static double diskReserve(const Robot&, const Eigen::Vector3d&, double,
                              const FilterConfig& config) {
        // Scene occupancy already carries the configured future-motion envelope,
        // while the downstream CBF accounts for velocity disturbance every tick.
        // Accumulating that velocity bound again as unbounded position error makes
        // the local rollout stop near valid goals and can deadlock a fleet.
        return config.clearance + config.uncertainty_margin;
    }

    static bool poseSafe(std::size_t index, const Robot& self, const Eigen::Vector3d& self_command,
                         const std::vector<Robot>& initial_robots,
                         const std::vector<Eigen::Vector3d>& peer_commands,
                         const std::vector<bool>& committed,
                         const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                         const FilterConfig& config, double time, double* min_clearance) {
        const auto self_disks = coveringDisks(self, config.disk_count);
        const double self_reserve = diskReserve(self, self_command, time, config);
        for (const auto& disk : self_disks) {
            if (fence.enabled) {
                const double margin = disk.radius + self_reserve;
                if (disk.center.x() < fence.xmin + margin ||
                    disk.center.x() > fence.xmax - margin ||
                    disk.center.y() < fence.ymin + margin ||
                    disk.center.y() > fence.ymax - margin) {
                    return false;
                }
            }
            for (const auto& obstacle : obstacles) {
                const double separation = pointPolygonDistance(disk.center, obstacle.vertices) -
                                          disk.radius - self_reserve;
                *min_clearance = std::min(*min_clearance, separation);
                if (separation <= 0.0) {
                    return false;
                }
            }
        }

        for (std::size_t peer_index = 0; peer_index < initial_robots.size(); ++peer_index) {
            if (peer_index == index) {
                continue;
            }
            const auto& initial_peer = initial_robots[peer_index];
            Eigen::Vector3d peer_command = Eigen::Vector3d::Zero();
            if (initial_peer.active) {
                peer_command =
                    committed[peer_index] ? peer_commands[peer_index] : initial_peer.previous;
            }
            Robot peer = predictConstant(initial_peer, peer_command, time);
            const auto peer_disks = coveringDisks(peer, config.disk_count);
            const double peer_reserve = diskReserve(peer, peer_command, time, config);
            for (const auto& a : self_disks) {
                for (const auto& b : peer_disks) {
                    // Inter-vehicle clearance is a pairwise margin, not one
                    // independent static margin per robot. Applying it twice made
                    // otherwise valid cooperative passages disappear.
                    const double separation = (a.center - b.center).norm() - a.radius - b.radius -
                                              std::max(self_reserve, peer_reserve);
                    *min_clearance = std::min(*min_clearance, separation);
                    if (separation <= 0.0) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    static RolloutResult rollout(std::size_t index, const Eigen::Vector3d& command,
                                 const std::vector<Robot>& robots,
                                 const std::vector<Eigen::Vector3d>& peer_commands,
                                 const std::vector<bool>& committed,
                                 const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                                 const FilterConfig& config) {
        RolloutResult result;
        Robot state = robots[index];
        double time = 0.0;
        if (!poseSafe(index, state, command, robots, peer_commands, committed, obstacles, fence,
                      config, time, &result.min_clearance)) {
            return result;
        }

        while (time + 1.0e-9 < kRolloutHorizon) {
            const double step = std::min(kRolloutStep, kRolloutHorizon - time);
            integrate(&state, command, step);
            time += step;
            if (!poseSafe(index, state, command, robots, peer_commands, committed, obstacles, fence,
                          config, time, &result.min_clearance)) {
                return result;
            }
        }

        // Dynamic-window admissibility includes a braking tail. This is still a
        // kinematic/acceleration-envelope check rather than a hardware brake
        // certificate, but it prevents selecting a trajectory that is safe only
        // while assuming an instantaneous stop at the end of the horizon.
        Eigen::Vector3d braking = command;
        for (int iteration = 0; iteration < 200 && braking.cwiseAbs().maxCoeff() > 1.0e-6;
             ++iteration) {
            Eigen::Vector3d next = braking;
            next.x() = approachZero(braking.x(), state.limits.accel_vx * kRolloutStep);
            next.y() = state.type == RobotType::Unicycle
                           ? 0.0
                           : approachZero(braking.y(), state.limits.accel_vy * kRolloutStep);
            next.z() = approachZero(braking.z(), state.limits.accel_omega * kRolloutStep);
            const Eigen::Vector3d average = 0.5 * (braking + next);
            integrate(&state, average, kRolloutStep);
            time += kRolloutStep;
            braking = next;
            if (!poseSafe(index, state, braking, robots, peer_commands, committed, obstacles, fence,
                          config, time, &result.min_clearance)) {
                return result;
            }
        }
        result.safe = braking.cwiseAbs().maxCoeff() <= 1.0e-6;
        return result;
    }

    static double commandScore(const Robot& robot, const Eigen::Vector3d& command,
                               const Eigen::Vector3d& desired) {
        const double sx = std::max(robot.limits.max_vx, 1.0e-6);
        const double sy = std::max(robot.limits.max_vy, 1.0e-6);
        const double sw = std::max(robot.limits.max_omega, 1.0e-6);
        double score = std::pow((command.x() - desired.x()) / sx, 2) +
                       std::pow((command.z() - desired.z()) / sw, 2);
        if (robot.type == RobotType::Mecanum) {
            score += std::pow((command.y() - desired.y()) / sy, 2);
        }
        // Clearance is a hard admissibility condition and a tie-break below.
        // It must not buy enough negative cost to override a feasible nominal
        // tracking command, which previously caused asymptotic near-zero motion.
        return score;
    }

    static Eigen::Vector3d brakingCommand(const Robot& robot, double dt) {
        Eigen::Vector3d result = robot.previous;
        result.x() = approachZero(result.x(), robot.limits.accel_vx * dt);
        result.y() = robot.type == RobotType::Unicycle
                         ? 0.0
                         : approachZero(result.y(), robot.limits.accel_vy * dt);
        result.z() = approachZero(result.z(), robot.limits.accel_omega * dt);
        return result;
    }

    void applyDynamicWindow(std::vector<Robot>& robots,
                            const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                            const FilterConfig& config) {
        std::vector<std::size_t> order;
        for (std::size_t i = 0; i < robots.size(); ++i) {
            if (robots[i].active && !robots[i].stop_requested) {
                order.push_back(i);
            }
        }
        std::sort(order.begin(), order.end(),
                  [&](std::size_t a, std::size_t b) { return robots[a].id < robots[b].id; });
        if (!order.empty()) {
            const std::size_t shift = priority_cursor_ % order.size();
            std::rotate(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(shift),
                        order.end());
            ++priority_cursor_;
        }

        std::vector<Eigen::Vector3d> committed_commands(robots.size(), Eigen::Vector3d::Zero());
        std::vector<bool> committed(robots.size(), false);
        const double dt = std::max(1.0e-3, std::min(0.1, config.dt));
        for (const auto index : order) {
            auto& robot = robots[index];
            const Eigen::Vector3d desired = robot.nominal;
            const auto xs = axisSamples(robot.previous.x(), desired.x(), robot.limits.max_vx,
                                        robot.limits.accel_vx, dt);
            const auto ys = robot.type == RobotType::Unicycle
                                ? std::vector<double>{0.0}
                                : axisSamples(robot.previous.y(), desired.y(), robot.limits.max_vy,
                                              robot.limits.accel_vy, dt);
            const auto ws = axisSamples(robot.previous.z(), desired.z(), robot.limits.max_omega,
                                        robot.limits.accel_omega, dt);

            bool found = false;
            double best_score = std::numeric_limits<double>::infinity();
            double best_clearance = -std::numeric_limits<double>::infinity();
            Eigen::Vector3d best = brakingCommand(robot, dt);
            for (const double vx : xs) {
                for (const double vy : ys) {
                    for (const double omega : ws) {
                        const Eigen::Vector3d candidate(vx, vy, omega);
                        const auto checked = rollout(index, candidate, robots, committed_commands,
                                                     committed, obstacles, fence, config);
                        if (!checked.safe) {
                            continue;
                        }
                        const double score = commandScore(robot, candidate, desired);
                        if (!found || score < best_score - 1.0e-12 ||
                            (std::abs(score - best_score) <= 1.0e-12 &&
                             checked.min_clearance > best_clearance)) {
                            found = true;
                            best_score = score;
                            best_clearance = checked.min_clearance;
                            best = candidate;
                        }
                    }
                }
            }
            // If every short-horizon candidate is blocked, request the fastest
            // dynamically reachable brake. The downstream joint CBF-QP still
            // decides whether that command is instantaneously feasible; failure
            // therefore stops/rejects rather than silently driving through.
            robot.nominal = found ? best : brakingCommand(robot, dt);
            committed_commands[index] = robot.nominal;
            committed[index] = true;
        }
    }

   public:
    void clear() {
        passages_.clear();
        encounters_.clear();
        priority_cursor_ = 0;
    }

    void apply(std::vector<Robot>& robots, const std::vector<ConvexObstacle>& obstacles,
               const Fence& fence, const FilterConfig& config = FilterConfig()) {
        std::vector<Eigen::Vector2d> velocity;
        velocity.reserve(robots.size());
        for (const auto& r : robots) {
            const double c = std::cos(r.yaw), s = std::sin(r.yaw);
            velocity.emplace_back(c * r.nominal.x() - s * r.nominal.y(),
                                  s * r.nominal.x() + c * r.nominal.y());
        }
        for (std::size_t i = 0; i < robots.size(); ++i) {
            for (std::size_t j = i + 1; j < robots.size(); ++j) {
                const auto& a = robots[i];
                const auto& b = robots[j];
                if (!a.active || !b.active || a.stop_requested || b.stop_requested) {
                    continue;
                }
                const Eigen::Vector2d d = b.position - a.position;
                const double distance = d.norm();
                if (distance < 1e-8) {
                    continue;
                }
                auto envelope = [&](const Robot& robot) {
                    double result = 0.0;
                    for (const auto& disk : coveringDisks(robot, config.disk_count)) {
                        result =
                            std::max(result, (disk.center - robot.position).norm() + disk.radius);
                    }
                    return result;
                };
                const double body_radius =
                    envelope(a) + envelope(b) + config.clearance + config.uncertainty_margin;
                const double gain = std::max(config.barrier_gain, 1.0e-6);
                const double drift = (2 * config.velocity_uncertainty +
                                      a.lateral_velocity_per_yaw_bound * a.limits.max_omega +
                                      b.lateral_velocity_per_yaw_bound * b.limits.max_omega) /
                                     gain;
                const double radius = drift + std::hypot(drift, body_radius) +
                                      0.5 * GuidanceOptions().lookahead_distance;
                const double turn_room =
                    a.limits.max_vx / a.limits.max_omega + b.limits.max_vx / b.limits.max_omega +
                    a.lateral_velocity_per_yaw_bound + b.lateral_velocity_per_yaw_bound;
                const auto pair = std::minmax(a.id, b.id);
                const std::pair<std::string, std::string> key(pair.first, pair.second);
                if (distance > radius + turn_room) {
                    encounters_.erase(key);
                }
                if (encounters_.count(key)) {
                    continue;
                }
                const Eigen::Vector2d relative = velocity[i] - velocity[j];
                const double closing = d.dot(relative);
                const double time = closing / std::max(relative.squaredNorm(), 1e-8);
                if (closing <= 0 || distance <= radius || (d - time * relative).norm() > radius ||
                    distance > radius + turn_room || distance <= radius + 1.0e-6) {
                    continue;
                }
                const Eigen::Vector2d axis = d / distance;
                const Eigen::Vector2d right(axis.y(), -axis.x());
                const Eigen::Vector2d middle = 0.5 * (a.position + b.position);
                const double denominator =
                    std::sqrt(std::max(1.0e-9, distance * distance - radius * radius));
                const double lateral = radius * distance / (2 * denominator);
                bool admitted = false;
                for (int side : {0, 1}) {
                    const auto& r = side == 0 ? a : b;
                    const auto existing = passages_.find(r.id);
                    if (!r.active || (existing != passages_.end() && !existing->second.complete) ||
                        r.nominal.head<2>().norm() < 1e-6) {
                        continue;
                    }
                    const double sign = side == 0 ? 1.0 : -1.0;
                    ResetTarget target;
                    target.position = middle + sign * lateral * right;
                    target.yaw = std::atan2(sign * axis.y(), sign * axis.x());
                    Passage passage;
                    passage.partner = side == 0 ? b.id : a.id;
                    const auto result = passage.guidance.setGoal(r, target, obstacles, fence);
                    if (result.status == GuidanceStatus::InvalidInput ||
                        result.status == GuidanceStatus::NoRoute) {
                        continue;
                    }
                    passages_[r.id] = std::move(passage);
                    admitted = true;
                }
                if (admitted) {
                    encounters_.insert(key);
                }
            }
        }

        for (auto& r : robots) {
            auto it = passages_.find(r.id);
            if (!r.active || it == passages_.end() || it->second.complete) {
                continue;
            }
            const auto result = it->second.guidance.step(r);
            if (result.status == GuidanceStatus::Reached) {
                it->second.complete = true;
                continue;
            }
            if (result.status == GuidanceStatus::Moving) {
                r.nominal = result.nominal;
            } else {
                r.nominal.setZero();
            }
        }

        applyDynamicWindow(robots, obstacles, fence, config);
    }
};

}  // namespace ugv_reset_safety
