#pragma once
#include <ugv_reset_safety/reset_guidance.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <vector>
namespace ugv_reset_safety {
// Persistent right-hand passage targets break the symmetric CBF stand-off.
// Targets use the same obstacle-checked visibility guidance as the main goal.
// They are an objective preference, not a global completeness certificate.
class FleetGuidance {
    struct Passage {
        ResetGuidance guidance;
        std::string partner;
        bool complete = false;
    };
    std::map<std::string, Passage> passages_;
    std::set<std::pair<std::string, std::string>> encounters_;

   public:
    void clear() {
        passages_.clear();
        encounters_.clear();
    }
    void apply(std::vector<Robot>& robots, const std::vector<ConvexObstacle>& obstacles,
               const Fence& fence, const FilterConfig& config = FilterConfig()) {
        std::vector<Eigen::Vector2d> velocity;
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
                // Circumscribe the actual covering-disk union, not just the
                // chassis rectangle. Include the active velocity uncertainty
                // in the zero-relative-speed barrier equilibrium distance.
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
                const double drift = (2 * config.velocity_uncertainty +
                                      a.lateral_velocity_per_yaw_bound * a.limits.max_omega +
                                      b.lateral_velocity_per_yaw_bound * b.limits.max_omega) /
                                     config.barrier_gain;
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
                const double closing = d.dot(relative),
                             time = closing / std::max(relative.squaredNorm(), 1e-8);
                if (closing <= 0 || distance <= radius || (d - time * relative).norm() > radius ||
                    distance > radius + turn_room) {
                    continue;
                }
                const Eigen::Vector2d axis = d / distance, right(axis.y(), -axis.x()),
                                      middle = .5 * (a.position + b.position);
                // For starts separated by D and opposite midpoint-normal
                // targets at +/-b, the incoming segment separation is
                // D*b/sqrt((D/2)^2+b^2). Solve it for the required radius.
                const double lateral =
                    radius * distance / (2 * std::sqrt(distance * distance - radius * radius));
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
    }
};
}  // namespace ugv_reset_safety
