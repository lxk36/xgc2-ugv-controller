#include <ugv_reset_safety/reset_guidance.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace ugv_reset_safety {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kGeometryEpsilon = 1e-9;
using Polygon = std::vector<Eigen::Vector2d>;

double cross(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() * b.y() - a.y() * b.x();
}

double wrap(double angle) {
    const double result = std::atan2(std::sin(angle), std::cos(angle));
    return std::abs(std::abs(result) - kPi) < 1e-12 ? kPi : result;
}

double clamp(double value, double limit) {
    return std::max(-limit, std::min(limit, value));
}

bool validRobot(const Robot& robot) {
    return robot.position.allFinite() && std::isfinite(robot.yaw) &&
           robot.body_center_offset.allFinite() &&
           std::isfinite(robot.lateral_velocity_per_yaw_bound) &&
           robot.lateral_velocity_per_yaw_bound >= 0.0 && std::isfinite(robot.half_length) &&
           robot.half_length > 0.0 && std::isfinite(robot.half_width) && robot.half_width > 0.0 &&
           std::isfinite(robot.limits.max_vx) && robot.limits.max_vx > 0.0 &&
           std::isfinite(robot.limits.max_vy) && robot.limits.max_vy >= 0.0 &&
           (robot.type != RobotType::Mecanum || robot.limits.max_vy > 0.0) &&
           std::isfinite(robot.limits.max_omega) && robot.limits.max_omega > 0.0;
}

bool validOptions(const GuidanceOptions& options) {
    return std::isfinite(options.position_gain) && options.position_gain > 0.0 &&
           std::isfinite(options.heading_gain) && options.heading_gain > options.position_gain &&
           std::isfinite(options.terminal_yaw_gain) && options.terminal_yaw_gain > 0.0 &&
           std::isfinite(options.position_tolerance) && options.position_tolerance > 0.0 &&
           std::isfinite(options.waypoint_tolerance) && options.waypoint_tolerance > 0.0 &&
           std::isfinite(options.lookahead_distance) && options.lookahead_distance > 0.0 &&
           std::isfinite(options.terminal_approach_distance) &&
           options.terminal_approach_distance > 0.0 && std::isfinite(options.path_clearance) &&
           options.path_clearance > 2.0 * options.waypoint_tolerance;
}

// Validate cyclic convex input before removing redundant collinear vertices.
// Taking an arbitrary convex hull here would silently repair malformed scene
// geometry; invalid maps instead produce an explicit rejected plan.
bool normalizePolygon(const Polygon& input, Polygon* output) {
    if (input.size() < 3) {
        return false;
    }
    *output = input;
    double area = 0.0;
    for (std::size_t i = 0; i < output->size(); ++i) {
        const auto& a = (*output)[i];
        const auto& b = (*output)[(i + 1) % output->size()];
        if (!a.allFinite() || (b - a).norm() < kGeometryEpsilon) {
            return false;
        }
        area += cross(a, b);
    }
    if (!std::isfinite(area) || std::abs(area) < kGeometryEpsilon) {
        return false;
    }
    if (area < 0.0) {
        std::reverse(output->begin(), output->end());
    }
    for (std::size_t i = 0; i < output->size(); ++i) {
        const Eigen::Vector2d edge = (*output)[(i + 1) % output->size()] - (*output)[i];
        for (const auto& vertex : *output) {
            if (cross(edge, vertex - (*output)[i]) / edge.norm() < -kGeometryEpsilon) {
                return false;
            }
        }
    }
    bool changed = true;
    while (changed && output->size() > 3) {
        changed = false;
        for (std::size_t i = 0; i < output->size(); ++i) {
            const Eigen::Vector2d previous =
                (*output)[i] - (*output)[(i + output->size() - 1) % output->size()];
            const Eigen::Vector2d next = (*output)[(i + 1) % output->size()] - (*output)[i];
            if (std::abs(cross(previous, next)) <=
                kGeometryEpsilon * previous.norm() * next.norm()) {
                output->erase(output->begin() + static_cast<std::ptrdiff_t>(i));
                changed = true;
                break;
            }
        }
    }
    return true;
}

// Intersect outward-offset support lines. This miter polygon contains the
// Euclidean disk Minkowski sum (it can be more conservative at acute corners).
bool inflate(const Polygon& input, double distance, Polygon* output) {
    Polygon polygon;
    if (!normalizePolygon(input, &polygon)) {
        return false;
    }
    output->clear();
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Eigen::Vector2d incoming =
            polygon[i] - polygon[(i + polygon.size() - 1) % polygon.size()];
        const Eigen::Vector2d outgoing = polygon[(i + 1) % polygon.size()] - polygon[i];
        const Eigen::Vector2d n1(incoming.y() / incoming.norm(), -incoming.x() / incoming.norm());
        const Eigen::Vector2d n2(outgoing.y() / outgoing.norm(), -outgoing.x() / outgoing.norm());
        const double determinant = cross(n1, n2);
        if (std::abs(determinant) < 1e-12) {
            return false;
        }
        const double b1 = n1.dot(polygon[i]) + distance;
        const double b2 = n2.dot(polygon[i]) + distance;
        Eigen::Vector2d vertex((b1 * n2.y() - n1.y() * b2) / determinant,
                               (n1.x() * b2 - b1 * n2.x()) / determinant);
        if (!vertex.allFinite()) {
            return false;
        }
        output->push_back(vertex);
    }
    return true;
}

bool insideInterior(const Eigen::Vector2d& point, const Polygon& polygon) {
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Eigen::Vector2d edge = polygon[(i + 1) % polygon.size()] - polygon[i];
        if (cross(edge, point - polygon[i]) / edge.norm() <= kGeometryEpsilon) {
            return false;
        }
    }
    return true;
}

bool inFence(const Eigen::Vector2d& point, const Fence& fence, double erosion) {
    return !fence.enabled || (point.x() >= fence.xmin + erosion - kGeometryEpsilon &&
                              point.x() <= fence.xmax - erosion + kGeometryEpsilon &&
                              point.y() >= fence.ymin + erosion - kGeometryEpsilon &&
                              point.y() <= fence.ymax - erosion + kGeometryEpsilon);
}

// Clip the segment against the strict interior halfspaces. Boundary segments
// of the expanded obstacle are permitted; the underlying body remains clear.
bool entersInterior(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Polygon& polygon) {
    double lower = 0.0;
    double upper = 1.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Eigen::Vector2d edge = polygon[(i + 1) % polygon.size()] - polygon[i];
        const double initial = cross(edge, a - polygon[i]) / edge.norm();
        const double slope = cross(edge, b - a) / edge.norm();
        if (std::abs(slope) < 1e-14) {
            if (initial <= kGeometryEpsilon) {
                return false;
            }
        } else {
            const double boundary = (kGeometryEpsilon - initial) / slope;
            if (slope > 0.0) {
                lower = std::max(lower, boundary);
            } else {
                upper = std::min(upper, boundary);
            }
        }
        if (lower >= upper - 1e-12) {
            return false;
        }
    }
    return lower < upper - 1e-12;
}

}  // namespace

VisibilityPath planVisibilityPath(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                                  const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                                  double radius, double clearance) {
    VisibilityPath result;
    if (!start.allFinite() || !goal.allFinite() || !std::isfinite(radius) || radius < 0.0 ||
        !std::isfinite(clearance) || clearance < 0.0 ||
        (fence.enabled &&
         (!std::isfinite(fence.xmin) || !std::isfinite(fence.xmax) || !std::isfinite(fence.ymin) ||
          !std::isfinite(fence.ymax) || fence.xmin >= fence.xmax || fence.ymin >= fence.ymax))) {
        result.invalid_input = true;
        result.message = "Invalid pose, footprint, clearance, or fence";
        return result;
    }
    const double expansion = radius + clearance;
    std::vector<Polygon> expanded;
    std::size_t vertex_count = 2;
    for (const auto& obstacle : obstacles) {
        Polygon polygon;
        vertex_count += obstacle.vertices.size();
        if (vertex_count > 1024 || !inflate(obstacle.vertices, expansion, &polygon)) {
            result.invalid_input = true;
            result.message =
                "Invalid convex obstacle or visibility graph capacity exceeded: " + obstacle.id;
            return result;
        }
        expanded.push_back(std::move(polygon));
    }
    auto free_point = [&](const Eigen::Vector2d& point) {
        if (!inFence(point, fence, expansion)) {
            return false;
        }
        for (const auto& polygon : expanded) {
            if (insideInterior(point, polygon)) {
                return false;
            }
        }
        return true;
    };
    if (!free_point(start) || !free_point(goal)) {
        result.message = "Start or target lacks required footprint clearance";
        return result;
    }
    std::vector<Eigen::Vector2d> nodes{start, goal};
    for (const auto& polygon : expanded) {
        for (const auto& point : polygon) {
            if (free_point(point)) {
                nodes.push_back(point);
            }
        }
    }

    const double infinity = std::numeric_limits<double>::infinity();
    const std::size_t count = nodes.size();
    std::vector<double> distances(count, infinity);
    std::vector<std::size_t> previous(count, count);
    std::vector<bool> visited(count, false);
    distances[0] = 0.0;
    for (std::size_t iteration = 0; iteration < count; ++iteration) {
        std::size_t current = count;
        for (std::size_t i = 0; i < count; ++i) {
            if (!visited[i] && (current == count || distances[i] < distances[current])) {
                current = i;
            }
        }
        if (current == count || !std::isfinite(distances[current])) {
            break;
        }
        if (current == 1) {
            break;
        }
        visited[current] = true;
        for (std::size_t next = 0; next < count; ++next) {
            if (visited[next] || next == current) {
                continue;
            }
            const double candidate = distances[current] + (nodes[next] - nodes[current]).norm();
            if (candidate >= distances[next]) {
                continue;
            }
            bool visible = true;
            for (const auto& polygon : expanded) {
                if (entersInterior(nodes[current], nodes[next], polygon)) {
                    visible = false;
                    break;
                }
            }
            if (visible) {
                distances[next] = candidate;
                previous[next] = current;
            }
        }
    }
    if (!std::isfinite(distances[1])) {
        result.message = "No route within footprint clearance and fence";
        return result;
    }
    for (std::size_t vertex = 1; vertex != count; vertex = previous[vertex]) {
        result.points.push_back(nodes[vertex]);
        if (vertex == 0) {
            break;
        }
    }
    std::reverse(result.points.begin(), result.points.end());
    result.valid = true;
    return result;
}

ResetGuidance::ResetGuidance(const GuidanceOptions& options) : options_(options) {}

void ResetGuidance::clear() {
    path_.clear();
    waypoint_ = 0;
    direction_ = 1;
    status_ = GuidanceStatus::Uninitialized;
    message_.clear();
}

GuidanceResult ResetGuidance::setGoal(const Robot& robot, const ResetTarget& target,
                                      const std::vector<ConvexObstacle>& obstacles,
                                      const Fence& fence) {
    clear();
    if (!validRobot(robot) || !validOptions(options_) || !target.position.allFinite() ||
        !std::isfinite(target.yaw)) {
        status_ = GuidanceStatus::InvalidInput;
        message_ = "Invalid robot, target, or guidance options";
        return step(robot);
    }
    target_ = target;
    target_.yaw = wrap(target.yaw);
    if ((target_.position - robot.position).norm() <= options_.position_tolerance) {
        path_ = {robot.position, target_.position};
        waypoint_ = 1;
        status_ = GuidanceStatus::Reached;
        chooseDirection(robot);
        return step(robot);
    }
    const double radius =
        std::hypot(robot.half_length, robot.half_width) + robot.body_center_offset.norm();
    auto plan = planVisibilityPath(robot.position, target.position, obstacles, fence, radius,
                                   options_.path_clearance + 0.5 * options_.lookahead_distance);
    if (!plan.valid) {
        status_ = plan.invalid_input ? GuidanceStatus::InvalidInput : GuidanceStatus::NoRoute;
        message_ = plan.message;
        return step(robot);
    }
    path_ = std::move(plan.points);
    if (robot.type == RobotType::Unicycle &&
        (target.position - robot.position).norm() > 2.0 * options_.position_tolerance) {
        const Eigen::Vector2d body_heading(std::cos(target_.yaw), std::sin(target_.yaw));
        const Eigen::Vector2d final_travel = -body_heading;
        const Eigen::Vector2d original_last = path_.back() - path_[path_.size() - 2];
        bool admitted = false;
        // Soft heading may bend an already compatible approach. Do not add a
        // turnaround loop solely to enforce a heading that is not an arrival gate.
        if (original_last.dot(final_travel) > 0.0) {
            const double tangent_length = std::min(options_.terminal_approach_distance,
                                                   0.5 * (target.position - robot.position).norm());
            const Eigen::Vector2d approach = target.position + tangent_length * body_heading;
            const double clearance = options_.path_clearance + 0.5 * options_.lookahead_distance;
            auto tangent =
                planVisibilityPath(approach, target.position, obstacles, fence, radius, clearance);
            auto prefix =
                planVisibilityPath(robot.position, approach, obstacles, fence, radius, clearance);
            if (tangent.valid && tangent.points.size() == 2 && prefix.valid &&
                prefix.points.size() >= 2) {
                const Eigen::Vector2d incoming =
                    prefix.points.back() - prefix.points[prefix.points.size() - 2];
                if (incoming.dot(final_travel) > 0.0) {
                    path_ = std::move(prefix.points);
                    path_.push_back(target.position);
                    admitted = true;
                }
            }
        }
        if (!admitted) {
            message_ = "Route has no compatible reverse tangent; final heading is unconstrained";
        }
    }
    waypoint_ = 1;
    status_ = GuidanceStatus::Moving;
    chooseDirection(robot);
    return step(robot);
}

void ResetGuidance::chooseDirection(const Robot& robot) {
    // Scout's measured lateral coupling is v_y=-ell*omega (ell>=0). With a
    // reverse travel frame this is +ell*omega; unlike a forward point approach,
    // its local yaw feedback does not invert as range decreases below ell.
    // This fixed physical direction is not an online +/-90 degree branch.
    direction_ = robot.type == RobotType::Unicycle ? -1 : 1;
}

GuidanceResult ResetGuidance::step(const Robot& robot) {
    GuidanceResult result;
    result.message = message_;
    if (status_ == GuidanceStatus::InvalidInput || status_ == GuidanceStatus::NoRoute ||
        status_ == GuidanceStatus::Uninitialized) {
        result.status = status_;
        result.message = message_;
        return result;
    }
    if (!validRobot(robot)) {
        status_ = GuidanceStatus::InvalidInput;
        message_ = "Non-finite pose or invalid robot limits during reset";
        result.status = status_;
        result.message = message_;
        return result;
    }
    const double goal_distance = (target_.position - robot.position).norm();
    const double yaw_error = wrap(target_.yaw - robot.yaw);
    // Polyline progress is geometric, not a timed reference running away from
    // a yielding vehicle. Crossing a segment's end plane advances progress;
    // tracking never demands convergence to an isolated obstacle corner.
    while (waypoint_ + 1 < path_.size()) {
        const Eigen::Vector2d segment = path_[waypoint_] - path_[waypoint_ - 1];
        bool advance = segment.norm() <= kGeometryEpsilon ||
                       (robot.position - path_[waypoint_]).dot(segment) >= 0.0 ||
                       (robot.position - path_[waypoint_]).norm() <= options_.waypoint_tolerance;
        if (!advance) {
            const Eigen::Vector2d outgoing = path_[waypoint_ + 1] - path_[waypoint_];
            const double outgoing_length = outgoing.squaredNorm();
            if (outgoing_length > kGeometryEpsilon * kGeometryEpsilon) {
                const double outgoing_projection =
                    (robot.position - path_[waypoint_]).dot(outgoing) / outgoing_length;
                if (outgoing_projection > 0.0) {
                    const double incoming_projection = std::max(
                        0.0, std::min(1.0, (robot.position - path_[waypoint_ - 1]).dot(segment) /
                                               segment.squaredNorm()));
                    const Eigen::Vector2d nearest_incoming =
                        path_[waypoint_ - 1] + incoming_projection * segment;
                    const Eigen::Vector2d nearest_outgoing =
                        path_[waypoint_] + std::min(1.0, outgoing_projection) * outgoing;
                    // At a sharp turn the lookahead point can already be on the
                    // outgoing segment before the incoming end plane is crossed.
                    // Transfer progress to that adjacent segment when it is closer;
                    // otherwise a stationary carrot can trap the robot off the corner.
                    advance = (robot.position - nearest_outgoing).squaredNorm() + 1e-12 <
                              (robot.position - nearest_incoming).squaredNorm();
                }
            }
        }
        if (!advance) {
            break;
        }
        ++waypoint_;
    }
    // The reset completion contract is XY tolerance. Do not add a final
    // rotate-in-place maneuver: an offset/slipping pose point can translate
    // while turning and create a perpetual position/yaw correction cycle.
    if (goal_distance <= options_.position_tolerance) {
        status_ = GuidanceStatus::Reached;
        result.status = status_;
        return result;
    }

    status_ = GuidanceStatus::Moving;
    const Eigen::Vector2d segment = path_[waypoint_] - path_[waypoint_ - 1];
    const double length_squared = segment.squaredNorm();
    const double projection =
        length_squared > kGeometryEpsilon * kGeometryEpsilon
            ? std::max(0.0, std::min(1.0, (robot.position - path_[waypoint_ - 1]).dot(segment) /
                                              length_squared))
            : 1.0;
    Eigen::Vector2d carrot = path_[waypoint_ - 1] + projection * segment;
    double remaining = options_.lookahead_distance;
    for (std::size_t vertex = waypoint_; vertex < path_.size(); ++vertex) {
        const Eigen::Vector2d delta = path_[vertex] - carrot;
        const double length = delta.norm();
        if (length >= remaining && length > kGeometryEpsilon) {
            carrot += remaining / length * delta;
            break;
        }
        carrot = path_[vertex];
        remaining -= length;
    }
    const Eigen::Vector2d error = carrot - robot.position;
    const double distance = error.norm();
    if (robot.type == RobotType::Mecanum) {
        const double c = std::cos(robot.yaw);
        const double s = std::sin(robot.yaw);
        const Eigen::Vector2d world_velocity = options_.position_gain * error;
        Eigen::Vector2d body_velocity(c * world_velocity.x() + s * world_velocity.y(),
                                      -s * world_velocity.x() + c * world_velocity.y());
        const double scale =
            std::max(1.0, std::max(std::abs(body_velocity.x()) / robot.limits.max_vx,
                                   std::abs(body_velocity.y()) / robot.limits.max_vy));
        body_velocity /= scale;
        result.nominal.head<2>() = body_velocity;
        result.nominal.z() = clamp(options_.terminal_yaw_gain * yaw_error, robot.limits.max_omega);
    } else if (distance > kGeometryEpsilon) {
        const double alpha =
            wrap(std::atan2(error.y(), error.x()) - robot.yaw - (direction_ < 0 ? kPi : 0.0));
        // Polar position regulation with a fixed reverse travel direction.
        // rho_dot=-speed*cos(alpha) on the ideal model: initial
        // motion may increase range while the chassis turns. A zero-translation
        // gate at |alpha|>=pi/2 traps a near target inside the rotating offset
        // pose circle, so translation is not suppressed at that boundary.
        // No lateral-velocity estimate, acceleration inversion, or 1/v appears.
        // Angular saturation and the safety QP invalidate any unrestricted global
        // convergence claim; they are checked separately in closed-loop tests.
        // If k_rho >= omega_max, saturated steering can admit an alpha=pi/2
        // circular orbit. Reserve angular authority relative to the radial gain.
        const double radial_gain = std::min(options_.position_gain, 0.5 * robot.limits.max_omega);
        // A common positive time scaling preserves the nominal geometric input
        // ratio while bounding the ell/rho feedback bandwidth near a slipping
        // pose point. It uses the declared uncertainty envelope, not measured vy.
        const double time_scale = distance / (distance + robot.lateral_velocity_per_yaw_bound);
        result.nominal.x() =
            time_scale * direction_ * std::min(robot.limits.max_vx, radial_gain * distance);
        result.nominal.z() =
            time_scale * clamp(options_.heading_gain * alpha, robot.limits.max_omega);
    }
    result.status = status_;
    return result;
}

}  // namespace ugv_reset_safety
