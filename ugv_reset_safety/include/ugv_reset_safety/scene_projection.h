#pragma once
#include <ugv_reset_safety/safety_filter.h>
#include <xgc2_geometry_msgs/SceneSnapshot.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
namespace ugv_reset_safety {
namespace scene_projection {
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
// Explicit conservative 2D projection for the existing planar Reset solver:
// intersect 32 supporting halfspaces of EACH part separately. This encloses
// curved surfaces (unlike an inscribed vertex ring), and never fills a
// compound's opening by taking one hull across all parts.
inline std::vector<ConvexObstacle> project(const xgc2_geometry_msgs::SceneSnapshot& scene,
                                           const std::string& frame) {
    if (scene.epoch.empty() || scene.header.frame_id != frame)
        throw std::invalid_argument("scene epoch/frame mismatch");
    constexpr int count = 32;
    constexpr double pi = 3.14159265358979323846;
    std::vector<ConvexObstacle> result;
    std::set<std::string> obstacle_ids;
    for (const auto& obstacle : scene.obstacles) {
        if (obstacle.id.empty() || !obstacle_ids.insert(obstacle.id).second ||
            obstacle.parts.empty())
            throw std::invalid_argument("invalid obstacle identity or empty parts");
        if (obstacle.dynamic)
            throw std::invalid_argument("dynamic obstacle unsupported by static Reset safety");
        const auto parent_rotation = rotation(obstacle.pose.orientation);
        const auto parent_position = point(obstacle.pose.position);
        std::set<std::string> part_ids;
        for (const auto& part : obstacle.parts) {
            if (part.id.empty() || !part_ids.insert(part.id).second)
                throw std::invalid_argument("duplicate/empty scene part id");
            const Eigen::Quaterniond orientation =
                parent_rotation * rotation(part.pose.orientation);
            const Eigen::Vector3d center =
                parent_position + parent_rotation * point(part.pose.position);
            std::vector<Eigen::Vector2d> normals;
            std::vector<double> offsets;
            for (int i = 0; i < count; ++i) {
                const double angle = 2 * pi * i / count;
                const Eigen::Vector3d direction(std::cos(angle), std::sin(angle), 0);
                normals.push_back(direction.head<2>());
                offsets.push_back(center.dot(direction) +
                                  support(part.geometry, orientation.conjugate() * direction));
            }
            ConvexObstacle projected;
            projected.id = std::to_string(obstacle.id.size()) + ":" + obstacle.id + part.id;
            for (int i = 0; i < count; ++i) {
                const int j = (i + 1) % count;
                Eigen::Matrix2d matrix;
                matrix.row(0) = normals[i];
                matrix.row(1) = normals[j];
                const Eigen::Vector2d vertex =
                    matrix.inverse() * Eigen::Vector2d(offsets[i], offsets[j]);
                if (!vertex.allFinite())
                    throw std::invalid_argument("scene projection overflow");
                // Zero curvature can create duplicate adjacent intersections.
                if (projected.vertices.empty() ||
                    (projected.vertices.back() - vertex).norm() > 1e-9)
                    projected.vertices.push_back(vertex);
            }
            if (projected.vertices.size() > 1 &&
                (projected.vertices.front() - projected.vertices.back()).norm() < 1e-9)
                projected.vertices.pop_back();
            if (projected.vertices.size() < 3)
                throw std::invalid_argument("degenerate scene projection");
            result.push_back(std::move(projected));
        }
    }
    std::sort(result.begin(), result.end(),
              [](const auto& a, const auto& b) { return a.id < b.id; });
    return result;
}
}  // namespace scene_projection
}  // namespace ugv_reset_safety
