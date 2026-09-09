#pragma once

#include <ugv_reset_safety/safety_filter.h>

#include <cstddef>
#include <string>
#include <vector>

namespace ugv_reset_safety {

struct ResetTarget {
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    double yaw = 0.0;
};

struct GuidanceOptions {
    double position_gain = 0.7;
    double heading_gain = 2.0;
    double terminal_yaw_gain = 1.5;
    double position_tolerance = 0.05;
    // Mecanum can hold XY while aligning yaw; Scout arrival remains XY only.
    double mecanum_yaw_tolerance = 0.05;
    double waypoint_tolerance = 0.015;
    // Arc-length carrot on the polyline; reserve half this distance in the
    // inflated geometry for turn anticipation. CBF remains the safety layer.
    double lookahead_distance = 0.5;
    // Soft Scout reverse tangent, admitted only with clear geometry and no
    // reversing cusp. Heading is never added to the XY arrival condition.
    double terminal_approach_distance = 0.35;
    // Clearance added to the circumscribed body; include the safety filter's
    // required clearance here. Half the lookahead is reserved separately.
    double path_clearance = 0.18;
};

enum class GuidanceStatus { Uninitialized, Moving, Reached, NoRoute, InvalidInput };

struct GuidanceResult {
    GuidanceStatus status = GuidanceStatus::Uninitialized;
    // Body FLU vx, vy, yaw rate; unicycle vy is always zero.
    Eigen::Vector3d nominal = Eigen::Vector3d::Zero();
    std::string message;
};

struct VisibilityPath {
    bool valid = false;
    bool invalid_input = false;
    std::vector<Eigen::Vector2d> points;
    std::string message;
};

// Goal geometry only; arrival still requires a measured stop and a zero
// command certified by the safety filter before releasing a Reset session.
bool withinTargetTolerance(const Robot& robot, const ResetTarget& target,
                           const GuidanceOptions& options = GuidanceOptions());

// Visibility graph and Dijkstra on convex polygons expanded by a circumscribed
// disk. Polygon input must be cyclically ordered and convex (either winding).
// The returned path is geometric guidance, not a timed collision certificate.
VisibilityPath planVisibilityPath(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                                  const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                                  double radius, double clearance);

class ResetGuidance {
   public:
    explicit ResetGuidance(const GuidanceOptions& options = GuidanceOptions());

    // Call once per reset request or after an explicit scene revision/replan.
    // A rejected plan clears the previous route; there is no direct-goal fallback.
    GuidanceResult setGoal(const Robot& robot, const ResetTarget& target,
                           const std::vector<ConvexObstacle>& obstacles, const Fence& fence);
    GuidanceResult step(const Robot& robot);
    void clear();

    const std::vector<Eigen::Vector2d>& path() const {
        return path_;
    }
    std::size_t waypointIndex() const {
        return waypoint_;
    }
    int travelDirection() const {
        return direction_;
    }
    const ResetTarget& target() const {
        return target_;
    }

   private:
    void chooseDirection(const Robot& robot);
    GuidanceOptions options_;
    ResetTarget target_;
    std::vector<Eigen::Vector2d> path_;
    std::size_t waypoint_ = 0;
    int direction_ = 1;
    GuidanceStatus status_ = GuidanceStatus::Uninitialized;
    std::string message_;
};

}  // namespace ugv_reset_safety
