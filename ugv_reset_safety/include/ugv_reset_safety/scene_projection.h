#pragma once
#include <ugv_reset_safety/safety_filter.h>
#include <xgc2_geometry_msgs/SceneSnapshot.h>
#include <xgc2_geometry_msgs/SceneState.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
namespace ugv_reset_safety {
namespace scene_projection {
inline constexpr double occupancyHorizon() {
    return 2.0;
}
inline bool supportedMotion(const std::string& type) {
    return type == "hold" || type == "constant_twist" || type == "ping_pong" || type == "circle";
}
inline std::string motionType(const xgc2_geometry_msgs::SceneObstacle& obstacle) {
    if (obstacle.motion_type.empty()) {
        return obstacle.dynamic ? std::string() : std::string("hold");
    }
    return obstacle.motion_type;
}
inline Eigen::Quaterniond rotation(const geometry_msgs::Quaternion& q) {
    Eigen::Quaterniond r(q.w, q.x, q.y, q.z);
    if (!r.coeffs().allFinite() || r.norm() < 1e-9)
        throw std::invalid_argument("invalid scene rotation");
    return r.normalized();
}
inline Eigen::Vector3d point(const geometry_msgs::Point& p) {
    Eigen::Vector3d v(p.x, p.y, p.z);
    if (!v.allFinite())
        throw std::invalid_argument("nonfinite scene position");
    return v;
}
inline Eigen::Vector3d vector(const geometry_msgs::Vector3& v) {
    Eigen::Vector3d value(v.x, v.y, v.z);
    if (!value.allFinite())
        throw std::invalid_argument("nonfinite scene twist");
    return value;
}
inline double support(const xgc2_geometry_msgs::SceneGeometry& g, const Eigen::Vector3d& d) {
    const auto positive = [](double v) {
        if (!std::isfinite(v) || v <= 0)
            throw std::invalid_argument("invalid geometry dimension");
    };
    if (g.type == "box") {
        positive(g.size.x);
        positive(g.size.y);
        positive(g.size.z);
        return .5 * (std::abs(d.x()) * g.size.x + std::abs(d.y()) * g.size.y +
                     std::abs(d.z()) * g.size.z);
    }
    if (g.type == "sphere" || g.type == "cylinder" || g.type == "capsule") {
        positive(g.radius);
        if (g.type == "sphere")
            return g.radius * d.norm();
        if (!std::isfinite(g.height) || g.height < 0 || (g.type == "cylinder" && g.height == 0))
            throw std::invalid_argument("invalid axial height");
        return .5 * g.height * std::abs(d.z()) +
               g.radius * (g.type == "capsule" ? d.norm() : d.head<2>().norm());
    }
    if (g.type == "convex") {
        if (g.vertices.size() < 4)
            throw std::invalid_argument("incomplete convex geometry");
        double h = -std::numeric_limits<double>::infinity();
        for (const auto& v : g.vertices)
            h = std::max(h, point(v).dot(d));
        return h;
    }
    throw std::invalid_argument("unsupported geometry: " + g.type);
}
struct BodyState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
};
inline ConvexObstacle projectPart(const xgc2_geometry_msgs::SceneObstacle& obstacle,
                                  const xgc2_geometry_msgs::ScenePart& part, const BodyState& body,
                                  double extra) {
    if (part.id.empty())
        throw std::invalid_argument("duplicate/empty scene part id");
    const Eigen::Quaterniond orientation = body.orientation * rotation(part.pose.orientation);
    const Eigen::Vector3d center = body.position + body.orientation * point(part.pose.position);
    constexpr int count = 32;
    constexpr double pi = 3.14159265358979323846;
    std::vector<Eigen::Vector2d> normals;
    std::vector<double> offsets;
    for (int i = 0; i < count; ++i) {
        const double angle = 2 * pi * i / count;
        const Eigen::Vector3d direction(std::cos(angle), std::sin(angle), 0);
        normals.push_back(direction.head<2>());
        offsets.push_back(center.dot(direction) +
                          support(part.geometry, orientation.conjugate() * direction) + extra);
    }
    ConvexObstacle projected;
    projected.id = std::to_string(obstacle.id.size()) + ":" + obstacle.id + part.id;
    projected.origin = body.position.head<2>();
    projected.velocity = body.linear.head<2>();
    projected.omega = body.angular.z();
    for (int i = 0; i < count; ++i) {
        const int j = (i + 1) % count;
        Eigen::Matrix2d matrix;
        matrix.row(0) = normals[i];
        matrix.row(1) = normals[j];
        const Eigen::Vector2d vertex = matrix.inverse() * Eigen::Vector2d(offsets[i], offsets[j]);
        if (!vertex.allFinite())
            throw std::invalid_argument("scene projection overflow");
        if (projected.vertices.empty() || (projected.vertices.back() - vertex).norm() > 1e-9)
            projected.vertices.push_back(vertex);
    }
    if (projected.vertices.size() > 1 &&
        (projected.vertices.front() - projected.vertices.back()).norm() < 1e-9)
        projected.vertices.pop_back();
    if (projected.vertices.size() < 3)
        throw std::invalid_argument("degenerate scene projection");
    return projected;
}
inline void validateObstacle(const xgc2_geometry_msgs::SceneObstacle& obstacle,
                             std::set<std::string>* obstacle_ids) {
    if (obstacle.id.empty() || !obstacle_ids->insert(obstacle.id).second || obstacle.parts.empty())
        throw std::invalid_argument("invalid obstacle identity or empty parts");
    const std::string type = motionType(obstacle);
    if (!supportedMotion(type))
        throw std::invalid_argument("unsupported motion type: " +
                                    (type.empty() ? std::string("<empty>") : type));
    if (obstacle.dynamic != (type != "hold"))
        throw std::invalid_argument("dynamic flag disagrees with declared motion type");
    std::set<std::string> part_ids;
    for (const auto& part : obstacle.parts) {
        if (part.id.empty() || !part_ids.insert(part.id).second)
            throw std::invalid_argument("duplicate/empty scene part id");
        support(part.geometry, Eigen::Vector3d::UnitX());
        rotation(part.pose.orientation);
        point(part.pose.position);
    }
}
inline BodyState snapshotBody(const xgc2_geometry_msgs::SceneObstacle& obstacle) {
    BodyState body;
    body.orientation = rotation(obstacle.pose.orientation);
    body.position = point(obstacle.pose.position);
    return body;
}
inline BodyState liveBody(const xgc2_geometry_msgs::SceneObstacleState& state) {
    BodyState body;
    body.orientation = rotation(state.pose.orientation);
    body.position = point(state.pose.position);
    body.linear = vector(state.twist.linear);
    body.angular = vector(state.twist.angular);
    return body;
}
inline double radiusBound(const ConvexObstacle& projected) {
    double rho = 0.0;
    for (const auto& vertex : projected.vertices)
        rho = std::max(rho, (vertex - projected.origin).norm());
    return rho;
}
inline std::vector<ConvexObstacle> projectBodies(const xgc2_geometry_msgs::SceneSnapshot& scene,
                                                 const std::map<std::string, BodyState>& bodies,
                                                 const std::string& frame, bool occupancy) {
    if (scene.epoch.empty() || scene.header.frame_id != frame)
        throw std::invalid_argument("scene epoch/frame mismatch");
    std::vector<ConvexObstacle> result;
    std::set<std::string> obstacle_ids;
    for (const auto& obstacle : scene.obstacles) {
        validateObstacle(obstacle, &obstacle_ids);
        const auto found = bodies.find(obstacle.id);
        if (found == bodies.end())
            throw std::invalid_argument("scene state missing obstacle " + obstacle.id);
        const BodyState& body = found->second;
        const double speed = body.linear.head<2>().norm();
        const double yaw_rate = std::abs(body.angular.z());
        for (const auto& part : obstacle.parts) {
            auto live = projectPart(obstacle, part, body, 0.0);
            const double extra =
                occupancy ? (speed + yaw_rate * radiusBound(live)) * occupancyHorizon() : 0.0;
            result.push_back(extra > 0.0 ? projectPart(obstacle, part, body, extra)
                                         : std::move(live));
        }
    }
    if (bodies.size() != obstacle_ids.size())
        throw std::invalid_argument("scene state has extra obstacle identities");
    std::sort(result.begin(), result.end(),
              [](const auto& a, const auto& b) { return a.id < b.id; });
    return result;
}
inline std::map<std::string, BodyState> snapshotBodies(
    const xgc2_geometry_msgs::SceneSnapshot& scene) {
    std::map<std::string, BodyState> bodies;
    for (const auto& obstacle : scene.obstacles)
        bodies.emplace(obstacle.id, snapshotBody(obstacle));
    return bodies;
}
inline std::map<std::string, BodyState> liveBodies(const xgc2_geometry_msgs::SceneSnapshot& scene,
                                                   const xgc2_geometry_msgs::SceneState& state) {
    if (state.epoch != scene.epoch || state.revision != scene.revision)
        throw std::invalid_argument("scene state epoch/revision mismatch");
    if (state.header.frame_id != scene.header.frame_id)
        throw std::invalid_argument("scene state frame mismatch");
    if (!std::isfinite(state.scene_time) || state.scene_time < 0.0)
        throw std::invalid_argument("invalid scene time");
    std::map<std::string, BodyState> bodies;
    std::set<std::string> ids;
    for (const auto& obstacle : state.obstacles) {
        if (obstacle.id.empty() || !ids.insert(obstacle.id).second)
            throw std::invalid_argument("invalid obstacle identity or empty parts");
        bodies.emplace(obstacle.id, liveBody(obstacle));
    }
    return bodies;
}
// Rest-pose projection of a hold-only snapshot. Dynamic motion requires live().
inline std::vector<ConvexObstacle> project(const xgc2_geometry_msgs::SceneSnapshot& scene,
                                           const std::string& frame) {
    std::set<std::string> ids;
    for (const auto& obstacle : scene.obstacles) {
        validateObstacle(obstacle, &ids);
        if (motionType(obstacle) != "hold")
            throw std::invalid_argument("live SceneState required for motion type " +
                                        motionType(obstacle));
    }
    return projectBodies(scene, snapshotBodies(scene), frame, false);
}
struct SceneGeometry {
    std::vector<ConvexObstacle> live;
    std::vector<ConvexObstacle> occupancy;
};
inline SceneGeometry live(const xgc2_geometry_msgs::SceneSnapshot& scene,
                          const xgc2_geometry_msgs::SceneState& state, const std::string& frame) {
    if (scene.epoch.empty() || scene.header.frame_id != frame)
        throw std::invalid_argument("scene epoch/frame mismatch");
    const auto bodies = liveBodies(scene, state);
    SceneGeometry result;
    result.live = projectBodies(scene, bodies, frame, false);
    result.occupancy = projectBodies(scene, bodies, frame, true);
    return result;
}
}  // namespace scene_projection
}  // namespace ugv_reset_safety
