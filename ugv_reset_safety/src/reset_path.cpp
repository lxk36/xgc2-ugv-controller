#include <ugv_reset_safety/reset_path.h>

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

bool validOptions(const PathOptions& options) {
    return std::isfinite(options.position_tolerance) && options.position_tolerance > 0.0 &&
           std::isfinite(options.mecanum_yaw_tolerance) && options.mecanum_yaw_tolerance > 0.0 &&
           options.mecanum_yaw_tolerance <= kPi && std::isfinite(options.path_clearance) &&
           options.path_clearance > 0.0 && std::isfinite(options.lookahead) &&
           options.lookahead > 0.0;
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

bool validConvexObstacle(const ConvexObstacle& obstacle) {
    Polygon normalized;
    return normalizePolygon(obstacle.vertices, &normalized);
}

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
            // Break equal-length detours to the robot's right, independent of
            // polygon winding and the peer's opposite direction of travel.
            const double tie =
                current == 0 && cross(goal - start, nodes[next] - start) > 0.0 ? 1.0e-6 : 0.0;
            const double candidate =
                distances[current] + (nodes[next] - nodes[current]).norm() + tie;
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

bool withinTargetTolerance(const Robot& robot, const ResetTarget& target,
                           const PathOptions& options) {
    return (robot.position - target.position).norm() <= options.position_tolerance &&
           (robot.type != RobotType::Mecanum ||
            std::abs(wrap(target.yaw - robot.yaw)) <= options.mecanum_yaw_tolerance);
}

void ResetPath::clear() {
    path_.clear();
    status_ = PathStatus::Uninitialized;
    message_.clear();
}

PathResult ResetPath::setGoal(const Robot& robot, const ResetTarget& target,
                              const std::vector<ConvexObstacle>& obstacles, const Fence& fence) {
    clear();
    if (!validRobot(robot) || !validOptions(options_) || !target.position.allFinite() ||
        !std::isfinite(target.yaw)) {
        status_ = PathStatus::InvalidInput;
        message_ = "Invalid robot, target, or path options";
        return step(robot);
    }
    target_ = target;
    const double radius =
        std::hypot(robot.half_length, robot.half_width) + robot.body_center_offset.norm();
    auto result = planVisibilityPath(robot.position, target.position, obstacles, fence, radius,
                                     planningClearance());
    if (!result.valid && !result.invalid_input) {
        // Lookahead adds preferred cornering room, not a second admission
        // boundary. A parked robot may already be inside that extra reserve.
        // Retain the full footprint and configured path clearance; DWA still
        // checks every selected trajectory against its unchanged hard margins.
        result = planVisibilityPath(robot.position, target.position, obstacles, fence, radius,
                                    options_.path_clearance);
    }
    if (!result.valid) {
        status_ = result.invalid_input ? PathStatus::InvalidInput : PathStatus::NoRoute;
        message_ = result.message;
        return step(robot);
    }
    path_ = std::move(result.points);
    status_ = PathStatus::Moving;
    return step(robot);
}

PathResult ResetPath::step(const Robot& robot) const {
    if (status_ != PathStatus::Moving) {
        return {status_, message_};
    }
    if (!validRobot(robot)) {
        return {PathStatus::InvalidInput, "Invalid robot pose or limits"};
    }
    return {
        withinTargetTolerance(robot, target_, options_) ? PathStatus::Reached : PathStatus::Moving,
        message_};
}

ResetPath::Projection ResetPath::project(const Eigen::Vector2d& position) const {
    Projection result;
    result.distance = std::numeric_limits<double>::infinity();
    double remaining = 0.0;
    for (std::size_t i = path_.size(); i > 1; --i) {
        const Eigen::Vector2d segment = path_[i - 1] - path_[i - 2];
        const double length = segment.norm();
        const double t =
            length > kGeometryEpsilon
                ? std::clamp((position - path_[i - 2]).dot(segment) / (length * length), 0.0, 1.0)
                : 1.0;
        const Eigen::Vector2d nearest = path_[i - 2] + t * segment;
        const double distance = (position - nearest).norm();
        if (distance < result.distance) {
            result.distance = distance;
            result.remaining = remaining + (1.0 - t) * length;
            result.point = nearest;
            result.tangent = length > kGeometryEpsilon ? Eigen::Vector2d(segment / length)
                                                       : Eigen::Vector2d::Zero();
        }
        remaining += length;
    }
    if (path_.size() == 1) {
        result.point = path_.front();
        result.distance = (position - result.point).norm();
    }
    return result;
}

Eigen::Vector2d ResetPath::pointAhead(const Eigen::Vector2d& position, double distance) const {
    double from_goal = std::max(0.0, project(position).remaining - distance);
    for (std::size_t i = path_.size(); i > 1; --i) {
        const Eigen::Vector2d segment = path_[i - 1] - path_[i - 2];
        const double length = segment.norm();
        if (length > kGeometryEpsilon && from_goal <= length) {
            return path_[i - 1] - from_goal / length * segment;
        }
        from_goal -= length;
    }
    return target_.position;
}

std::vector<ResetPath> planPeerPaths(const std::vector<Robot>& robots,
                                     const std::vector<ResetPath>& paths,
                                     const std::vector<ConvexObstacle>& obstacles,
                                     const Fence& fence) {
    if (robots.size() != paths.size()) {
        return {};
    }
    auto result = paths;
    for (std::size_t i = 0; i < robots.size(); ++i) {
        const auto& self = robots[i];
        if (!self.active || self.stop_requested) {
            continue;
        }
        auto occupancy = obstacles;
        for (std::size_t j = 0; j < robots.size(); ++j) {
            if (i == j) {
                continue;
            }
            const auto& peer = robots[j];
            const double radius = std::hypot(self.half_length, self.half_width) +
                                  self.body_center_offset.norm() +
                                  std::hypot(peer.half_length, peer.half_width) +
                                  peer.body_center_offset.norm() + paths[i].planningClearance();
            // Nearby peers may not fit the conservative path inflation.
            // Active and arrived peers keep the same hard DWA body constraints.
            if ((peer.position - paths[i].target().position).norm() <= radius ||
                (peer.position - self.position).norm() <= radius) {
                continue;
            }
            ConvexObstacle body;
            body.id = "fleet_peer/" + peer.id;
            Eigen::Matrix2d rotation;
            rotation << std::cos(peer.yaw), -std::sin(peer.yaw), std::sin(peer.yaw),
                std::cos(peer.yaw);
            for (const auto& corner :
                 std::vector<Eigen::Vector2d>{{-peer.half_length, -peer.half_width},
                                              {peer.half_length, -peer.half_width},
                                              {peer.half_length, peer.half_width},
                                              {-peer.half_length, peer.half_width}}) {
                body.vertices.push_back(peer.position +
                                        rotation * (corner + peer.body_center_offset));
            }
            occupancy.push_back(std::move(body));
        }
        if (occupancy.size() == obstacles.size()) {
            continue;
        }
        ResetPath candidate = paths[i];
        const auto status = candidate.setGoal(self, paths[i].target(), occupancy, fence).status;
        if (status == PathStatus::Moving || status == PathStatus::Reached) {
            result[i] = std::move(candidate);
        }
    }
    return result;
}

}  // namespace ugv_reset_safety
