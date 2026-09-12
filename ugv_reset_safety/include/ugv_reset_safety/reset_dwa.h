#pragma once
#include <ugv_reset_safety/reset_path.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace ugv_reset_safety {

// ResetDwa is the online local-planning layer after geometric route
// guidance. The geometric planner decides where a robot should go; this class
// decides which immediately reachable body twist to execute over the next
// short horizon. Every proposal is rolled out with the chassis kinematics,
// covering-disk footprint, scene occupancy, peer motion, fence limits, and an
// explicit braking tail. DWA is the Reset safety authority: no feasible sample
// is fail-closed (zero + infeasible), not an unrolled brake treated as safe.
class ResetDwa {
    struct RolloutResult {
        bool safe = false;
        double min_clearance = std::numeric_limits<double>::infinity();
        Robot endpoint;
    };

    struct MotionObservation {
        Eigen::Vector2d position;
        double yaw;
        double lateral_offset = 0.0;
    };
    std::map<std::string, MotionObservation> observations_;

    void observeMotion(const std::vector<Robot>& robots, double dt) {
        for (const auto& robot : robots) {
            auto found = observations_.find(robot.id);
            if (found == observations_.end()) {
                observations_.emplace(robot.id, MotionObservation{robot.position, robot.yaw});
                continue;
            }
            auto& observation = found->second;
            const double turn = std::atan2(std::sin(robot.yaw - observation.yaw),
                                           std::cos(robot.yaw - observation.yaw));
            if (robot.type == RobotType::Unicycle && std::abs(turn) > 1.0e-4) {
                // Integrated lateral displacement / heading change estimates the
                // kinematic body-origin offset. No twist topic or lateral
                // actuator is introduced. Small turns are excluded as ill-conditioned.
                const double midpoint = observation.yaw + 0.5 * turn;
                const double lateral =
                    (robot.position - observation.position)
                        .dot(Eigen::Vector2d(-std::sin(midpoint), std::cos(midpoint)));
                const double sample = std::clamp(-lateral / (2.0 * std::sin(0.5 * turn)),
                                                 -robot.lateral_velocity_per_yaw_bound,
                                                 robot.lateral_velocity_per_yaw_bound);
                observation.lateral_offset +=
                    (1.0 - std::exp(-dt / 0.1)) * (sample - observation.lateral_offset);
            }
            observation.position = robot.position;
            observation.yaw = robot.yaw;
        }
    }

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

    static std::vector<double> axisSamples(double previous, double limit, double acceleration,
                                           double dt) {
        if (limit <= 0.0) {
            return {0.0};
        }
        const double reach = std::max(0.0, acceleration) * std::max(0.0, dt);
        const double low = std::max(-limit, previous - reach);
        const double high = std::min(limit, previous + reach);
        std::vector<double> values{
            low,  low + 0.25 * (high - low), 0.5 * (low + high), low + 0.75 * (high - low),
            high, clamp(previous, low, high)};
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

    static std::vector<Eigen::Vector2d> bodyRectangle(const Robot& robot) {
        std::vector<Eigen::Vector2d> result;
        result.reserve(4);
        Eigen::Matrix2d rotation;
        rotation << std::cos(robot.yaw), -std::sin(robot.yaw), std::sin(robot.yaw),
            std::cos(robot.yaw);
        const Eigen::Vector2d corners[4] = {{-robot.half_length, -robot.half_width},
                                            {robot.half_length, -robot.half_width},
                                            {robot.half_length, robot.half_width},
                                            {-robot.half_length, robot.half_width}};
        for (const auto& corner : corners) {
            result.push_back(robot.position + rotation * (corner + robot.body_center_offset));
        }
        return result;
    }

    static double convexClearance(const std::vector<Eigen::Vector2d>& first,
                                  const std::vector<Eigen::Vector2d>& second) {
        bool separated = false;
        double penetration = std::numeric_limits<double>::infinity();
        for (const auto* polygon : {&first, &second}) {
            for (std::size_t edge = 0; edge < polygon->size(); ++edge) {
                const Eigen::Vector2d tangent =
                    (*polygon)[(edge + 1) % polygon->size()] - (*polygon)[edge];
                const double length = tangent.norm();
                if (length <= kGeometryEpsilon) {
                    continue;
                }
                const Eigen::Vector2d normal(-tangent.y() / length, tangent.x() / length);
                double amin = std::numeric_limits<double>::infinity(), amax = -amin;
                double bmin = amin, bmax = -amin;
                for (const auto& point : first) {
                    amin = std::min(amin, normal.dot(point));
                    amax = std::max(amax, normal.dot(point));
                }
                for (const auto& point : second) {
                    bmin = std::min(bmin, normal.dot(point));
                    bmax = std::max(bmax, normal.dot(point));
                }
                const double overlap = std::min(amax, bmax) - std::max(amin, bmin);
                if (overlap < 0.0) {
                    separated = true;
                }
                penetration = std::min(penetration, overlap);
            }
        }
        if (!separated) {
            return -penetration;
        }
        double gap = std::numeric_limits<double>::infinity();
        for (const auto& point : first) {
            gap = std::min(gap, pointPolygonDistance(point, second));
        }
        for (const auto& point : second) {
            gap = std::min(gap, pointPolygonDistance(point, first));
        }
        return gap;
    }

    static bool poseSafeModel(std::size_t index, const Robot& self,
                              const Eigen::Vector3d& self_command,
                              const std::vector<Robot>& initial_robots,
                              const std::vector<Eigen::Vector3d>& peer_commands,
                              const std::vector<bool>& committed,
                              const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                              const DwaConfig& config, double time, double* min_clearance,
                              double peer_lateral_scale) {
        const auto self_disks = coveringDisks(self, config.disk_count);
        const double self_reserve = config.clearance;
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

        const auto self_body = bodyRectangle(self);
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
            if (peer_lateral_scale != 0.0 && peer.type == RobotType::Unicycle) {
                peer.position += peer_lateral_scale * initial_peer.lateral_velocity_per_yaw_bound *
                                 Eigen::Vector2d(std::cos(initial_peer.yaw) - std::cos(peer.yaw),
                                                 std::sin(initial_peer.yaw) - std::sin(peer.yaw));
            }
            const double gap = convexClearance(self_body, bodyRectangle(peer));
            *min_clearance = std::min(*min_clearance, gap);
            // t=0 / stay-put: only reject actual rectangle penetration so a
            // still-legal pose is not an absorbing set. Future times keep a
            // 2 cm contact margin for plant lag and intersample motion.
            const bool staying = self_command.cwiseAbs().maxCoeff() <= 1.0e-6 &&
                                 peer_command.cwiseAbs().maxCoeff() <= 1.0e-6;
            const double margin = (time <= 1.0e-9 || staying) ? 0.0 : 0.02;
            if (gap <= margin) {
                return false;
            }
        }
        return true;
    }

    static bool poseSafe(std::size_t index, const Robot& self, const Eigen::Vector3d& command,
                         const std::vector<Robot>& robots,
                         const std::vector<Eigen::Vector3d>& peer_commands,
                         const std::vector<bool>& committed,
                         const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                         const DwaConfig& config, double time, double* clearance) {
        for (double self_lateral_scale : {0.0, -1.0, 1.0}) {
            if (self_lateral_scale != 0.0 &&
                (self.type != RobotType::Unicycle || self.lateral_velocity_per_yaw_bound == 0.0)) {
                continue;
            }
            Robot predicted = self;
            if (self_lateral_scale != 0.0) {
                predicted.position +=
                    self_lateral_scale * self.lateral_velocity_per_yaw_bound *
                    Eigen::Vector2d(std::cos(robots[index].yaw) - std::cos(self.yaw),
                                    std::sin(robots[index].yaw) - std::sin(self.yaw));
            }
            for (double peer_lateral_scale : {0.0, -1.0, 1.0}) {
                if (!poseSafeModel(index, predicted, command, robots, peer_commands, committed,
                                   obstacles, fence, config, time, clearance, peer_lateral_scale)) {
                    return false;
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
                                 const DwaConfig& config, double horizon = kRolloutHorizon) {
        RolloutResult result;
        Robot state = robots[index];
        double time = 0.0;
        if (!poseSafe(index, state, command, robots, peer_commands, committed, obstacles, fence,
                      config, time, &result.min_clearance)) {
            return result;
        }

        while (time + 1.0e-9 < horizon) {
            const double step = std::min(kRolloutStep, horizon - time);
            integrate(&state, command, step);
            time += step;
            if (!poseSafe(index, state, command, robots, peer_commands, committed, obstacles, fence,
                          config, time, &result.min_clearance)) {
                return result;
            }
        }

        result.endpoint = state;

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

    static double trajectoryScore(const Robot& robot, const Robot& endpoint,
                                  const Eigen::Vector3d& command, const ResetPath& path,
                                  bool encounter, double lateral_offset) {
        // Inside the accepted Scout pose region keep the same rotation-center
        // position objective, so stopping is not biased to its boundary.
        // Heading remains accepted within the user's tolerance.
        if (path.reached(endpoint)) {
            return (robot.type == RobotType::Unicycle
                        ? 2.0 * path.unicycleGoalCost(endpoint, lateral_offset)
                        : 0.0) +
                   0.2 * command.squaredNorm();
        }
        if (robot.type == RobotType::Unicycle &&
            path.project(robot.position).remaining <= path.lookahead()) {
            return 2.0 * path.unicycleGoalCost(endpoint, lateral_offset) +
                   0.2 * command.squaredNorm();
        }
        double score =
            2.0 * (endpoint.position - path.pointAhead(robot.position, path.lookahead())).norm() +
            0.2 * command.squaredNorm();
        if (robot.type == RobotType::Mecanum) {
            const double yaw_error = std::atan2(std::sin(path.target().yaw - endpoint.yaw),
                                                std::cos(path.target().yaw - endpoint.yaw));
            score += 0.25 * yaw_error * yaw_error;
        } else if (!path.reached(endpoint)) {
            const Eigen::Vector2d ahead = path.pointAhead(robot.position, path.lookahead()) -
                                          (encounter ? robot.position : endpoint.position);
            if (ahead.norm() > 1.0e-9) {
                const double bearing = std::atan2(ahead.y(), ahead.x());
                const double forward =
                    std::atan2(std::sin(bearing - endpoint.yaw), std::cos(bearing - endpoint.yaw));
                const double reverse =
                    std::atan2(std::sin(bearing - endpoint.yaw - 3.14159265358979323846),
                               std::cos(bearing - endpoint.yaw - 3.14159265358979323846));
                const double error =
                    command.x() > 1.0e-9
                        ? forward
                        : command.x() < -1.0e-9
                              ? reverse
                              : std::abs(forward) < std::abs(reverse) ? forward : reverse;
                score += 0.5 *
                         std::min(1.0, (robot.position - path.target().position).squaredNorm() /
                                           (path.lookahead() * path.lookahead())) *
                         error * error;
            }
        }
        return score;
    }

    static double encounterScore(std::size_t index, const Robot& endpoint,
                                 const Eigen::Vector3d& command, const std::vector<Robot>& robots,
                                 const std::vector<ResetPath>& paths) {
        const auto& self = robots[index];
        const Eigen::Vector2d self_heading(std::cos(endpoint.yaw), std::sin(endpoint.yaw));
        const double self_direction =
            std::abs(command.x()) > 1.0e-9
                ? (command.x() > 0 ? 1.0 : -1.0)
                : (self_heading.dot(
                       paths[index].pointAhead(self.position, paths[index].lookahead()) -
                       self.position) >= 0
                       ? 1.0
                       : -1.0);
        const Eigen::Vector2d velocity =
            self.type == RobotType::Unicycle
                ? Eigen::Vector2d(self_direction * self.limits.max_vx * self_heading)
                : self.limits.max_vx * paths[index].project(endpoint.position).tangent;
        double score = 0.0;
        for (std::size_t j = 0; j < robots.size(); ++j) {
            if (j == index || !robots[j].active || robots[j].stop_requested) {
                continue;
            }
            const auto& peer = robots[j];
            const Eigen::Vector2d peer_heading(std::cos(peer.yaw), std::sin(peer.yaw));
            const double peer_direction =
                std::abs(peer.previous.x()) > 1.0e-9
                    ? (peer.previous.x() > 0 ? 1.0 : -1.0)
                    : (peer_heading.dot(paths[j].pointAhead(peer.position, paths[j].lookahead()) -
                                        peer.position) >= 0
                           ? 1.0
                           : -1.0);
            const Eigen::Vector2d peer_velocity =
                peer.type == RobotType::Unicycle
                    ? Eigen::Vector2d(peer_direction * peer.limits.max_vx * peer_heading)
                    : peer.limits.max_vx * paths[j].project(peer.position).tangent;
            const Eigen::Vector2d delta = peer.position - self.position;
            const Eigen::Vector2d relative = velocity - peer_velocity;
            const double closing = delta.dot(relative);
            if (closing <= 0.0 || relative.squaredNorm() < 1.0e-9) {
                continue;
            }
            const double time = closing / relative.squaredNorm();
            const double turn_room = self.limits.max_vx / self.limits.max_omega +
                                     peer.limits.max_vx / peer.limits.max_omega;
            const double radius = std::hypot(self.half_length, self.half_width) +
                                  std::hypot(peer.half_length, peer.half_width) + 0.1 +
                                  2.0 * self.lateral_velocity_per_yaw_bound *
                                      std::sin(0.5 * self.limits.max_omega * kRolloutHorizon) +
                                  2.0 * peer.lateral_velocity_per_yaw_bound *
                                      std::sin(0.5 * peer.limits.max_omega * kRolloutHorizon);
            if (delta.norm() > radius + 2.0 * turn_room) {
                continue;
            }
            const double miss = (delta - time * relative).norm();
            if (miss > radius) {
                continue;
            }
            const double bearing = std::atan2(delta.y(), delta.x());
            const double travel = std::atan2(relative.y(), relative.x());
            const double error = std::atan2(std::sin(travel - bearing), std::cos(travel - bearing));
            const double cone = std::asin(std::min(1.0, radius / std::max(delta.norm(), radius)));
            // A common starboard preference breaks a symmetric head-on tie.
            // This scores this candidate's heading; it creates no alternate goal.
            if (std::abs(error) < cone) {
                score += 8.0 * (cone + error) * (cone + error);
            }
        }
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

    void applyDynamicWindow(std::vector<Robot>& robots, const std::vector<ResetPath>& paths,
                            const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                            const DwaConfig& config) {
        for (auto& robot : robots) {
            if (!robot.active || robot.stop_requested) {
                robot.command.setZero();
                robot.local_plan_feasible = true;
            }
        }
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
            bool encounter = false;
            for (std::size_t j = 0; j < robots.size(); ++j) {
                if (j == index || !robots[j].active || robots[j].stop_requested) {
                    continue;
                }
                const auto& peer = robots[j];
                const double range = 2.0 * (robot.limits.max_vx / robot.limits.max_omega +
                                            peer.limits.max_vx / peer.limits.max_omega) +
                                     std::hypot(robot.half_length, robot.half_width) +
                                     std::hypot(peer.half_length, peer.half_width);
                encounter = encounter || (peer.position - robot.position).norm() < range;
            }
            encounter = encounter && (robot.position - paths[index].target().position).norm() >
                                         2.0 * paths[index].lookahead();
            const auto xs =
                axisSamples(robot.previous.x(), robot.limits.max_vx, robot.limits.accel_vx, dt);
            const auto ys = robot.type == RobotType::Unicycle
                                ? std::vector<double>{0.0}
                                : axisSamples(robot.previous.y(), robot.limits.max_vy,
                                              robot.limits.accel_vy, dt);
            const auto ws = axisSamples(robot.previous.z(), robot.limits.max_omega,
                                        robot.limits.accel_omega, dt);

            bool found = false;
            double best_score = std::numeric_limits<double>::infinity();
            double best_clearance = -std::numeric_limits<double>::infinity();
            Eigen::Vector3d best = brakingCommand(robot, dt);
            struct Candidate {
                Eigen::Vector3d command;
                double score;
            };
            std::vector<Candidate> candidates;
            candidates.reserve(xs.size() * ys.size() * ws.size());
            for (const double vx : xs) {
                for (const double vy : ys) {
                    for (const double omega : ws) {
                        const Eigen::Vector3d candidate(vx, vy, omega);
                        // Scoring does not depend on collision clearance. Rank
                        // cheap endpoints first; only admissibility needs the
                        // full footprint rollout and braking tail.
                        Robot endpoint = robot;
                        double time = 0.0;
                        while (time + 1.0e-9 < kRolloutHorizon) {
                            const double step = std::min(kRolloutStep, kRolloutHorizon - time);
                            integrate(&endpoint, candidate, step);
                            time += step;
                        }
                        Robot predicted = endpoint;
                        if (robot.type == RobotType::Unicycle) {
                            predicted.position +=
                                observations_.at(robot.id).lateral_offset *
                                Eigen::Vector2d(std::cos(robot.yaw) - std::cos(predicted.yaw),
                                                std::sin(robot.yaw) - std::sin(predicted.yaw));
                        }
                        const double score =
                            trajectoryScore(robot, predicted, candidate, paths[index], encounter,
                                            observations_.at(robot.id).lateral_offset) +
                            encounterScore(index, endpoint, candidate, robots, paths);
                        candidates.push_back({candidate, score});
                    }
                }
            }
            std::stable_sort(
                candidates.begin(), candidates.end(),
                [](const Candidate& a, const Candidate& b) { return a.score < b.score; });
            for (const auto& candidate : candidates) {
                if (found && candidate.score > best_score + 1.0e-12) {
                    break;
                }
                const auto checked = rollout(index, candidate.command, robots, committed_commands,
                                             committed, obstacles, fence, config);
                if (!checked.safe) {
                    continue;
                }
                if (!found || candidate.score < best_score - 1.0e-12 ||
                    (std::abs(candidate.score - best_score) <= 1.0e-12 &&
                     checked.min_clearance > best_clearance)) {
                    found = true;
                    best_score = candidate.score;
                    best_clearance = checked.min_clearance;
                    best = candidate.command;
                }
            }
            if (found) {
                robot.command = best;
                robot.local_plan_feasible = true;
            } else {
                // Fail-closed: no rolled-out sample is admissible. Command
                // exact zero; do not publish an unrolled brake as safety.
                robot.command.setZero();
                robot.local_plan_feasible = false;
            }
            committed_commands[index] = robot.command;
            committed[index] = true;
        }
    }

   public:
    static constexpr double predictionHorizon() {
        return kRolloutHorizon;
    }

    void clear() {
        priority_cursor_ = 0;
        observations_.clear();
    }

    void apply(std::vector<Robot>& robots, const std::vector<ResetPath>& paths,
               const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
               const DwaConfig& config = DwaConfig()) {
        bool valid = paths.size() == robots.size() && std::isfinite(config.dt) && config.dt > 0.0 &&
                     config.dt <= 0.1 && config.disk_count > 0 && std::isfinite(config.clearance) &&
                     config.clearance >= 0.0 && std::isfinite(config.uncertainty_margin) &&
                     config.uncertainty_margin >= 0.0 &&
                     std::isfinite(config.feasibility_tolerance) &&
                     config.feasibility_tolerance > 0.0;
        if (fence.enabled) {
            valid = valid && std::isfinite(fence.xmin) && std::isfinite(fence.xmax) &&
                    std::isfinite(fence.ymin) && std::isfinite(fence.ymax) &&
                    fence.xmin < fence.xmax && fence.ymin < fence.ymax;
        }
        for (const auto& obstacle : obstacles) {
            valid = valid && validConvexObstacle(obstacle);
        }
        for (std::size_t i = 0; valid && i < robots.size(); ++i) {
            const auto& robot = robots[i];
            const auto status = paths[i].step(robot).status;
            valid =
                robot.position.allFinite() && std::isfinite(robot.yaw) &&
                robot.body_center_offset.allFinite() && std::isfinite(robot.half_length) &&
                robot.half_length > 0.0 && std::isfinite(robot.half_width) &&
                robot.half_width > 0.0 && robot.previous.allFinite() &&
                std::isfinite(robot.lateral_velocity_per_yaw_bound) &&
                robot.lateral_velocity_per_yaw_bound >= 0.0 &&
                ((robot.active && !robot.stop_requested) ||
                 robot.previous.cwiseAbs().maxCoeff() <= config.feasibility_tolerance) &&
                std::isfinite(robot.limits.max_vx) && robot.limits.max_vx > 0.0 &&
                std::isfinite(robot.limits.max_vy) && robot.limits.max_vy >= 0.0 &&
                std::isfinite(robot.limits.max_omega) && robot.limits.max_omega > 0.0 &&
                std::abs(robot.previous.x()) <= robot.limits.max_vx + 1.0e-9 &&
                std::abs(robot.previous.y()) <= robot.limits.max_vy + 1.0e-9 &&
                std::abs(robot.previous.z()) <= robot.limits.max_omega + 1.0e-9 &&
                (robot.type != RobotType::Unicycle || std::abs(robot.previous.y()) <= 1.0e-9) &&
                std::isfinite(robot.limits.accel_vx) && robot.limits.accel_vx > 0.0 &&
                std::isfinite(robot.limits.accel_vy) && robot.limits.accel_vy > 0.0 &&
                std::isfinite(robot.limits.accel_omega) && robot.limits.accel_omega > 0.0 &&
                (!robot.active || status == PathStatus::Moving || status == PathStatus::Reached);
        }
        if (!valid) {
            for (auto& robot : robots) {
                robot.command.setZero();
                robot.local_plan_feasible = false;
            }
            return;
        }
        observeMotion(robots, config.dt);
        const auto routes = planPeerPaths(robots, paths, obstacles, fence);
        applyDynamicWindow(robots, routes, obstacles, fence, config);
    }
};

}  // namespace ugv_reset_safety
