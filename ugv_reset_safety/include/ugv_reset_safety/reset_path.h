#pragma once

#include <ugv_reset_safety/reset_geometry.h>

#include <cstddef>
#include <string>
#include <vector>

namespace ugv_reset_safety {

struct ResetTarget {
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    double yaw = 0.0;
};

struct PathOptions {
    double position_tolerance = 0.05;
    double yaw_tolerance = 0.17453292519943295;  // 10 degrees, shortest angle
    double path_clearance = 0.18;
    double lookahead = 0.5;
};

enum class PathStatus { Uninitialized, Moving, Reached, NoRoute, InvalidInput };

struct PathResult {
    PathStatus status = PathStatus::Uninitialized;
    std::string message;
};

struct VisibilityPath {
    bool valid = false;
    bool invalid_input = false;
    std::vector<Eigen::Vector2d> points;
    std::string message;
};

bool validConvexObstacle(const ConvexObstacle& obstacle);

bool withinTargetTolerance(const Robot& robot, const ResetTarget& target,
                           const PathOptions& options = PathOptions());

double unicyclePoseDistance(const Robot& robot, const ResetTarget& target, double lateral_offset);

VisibilityPath planVisibilityPath(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                                  const std::vector<ConvexObstacle>& obstacles, const Fence& fence,
                                  double radius, double clearance);

// Geometric route only. Velocity selection belongs to ResetDwa.
class ResetPath {
   public:
    struct Projection {
        double distance = 0.0;
        double remaining = 0.0;
        Eigen::Vector2d tangent = Eigen::Vector2d::Zero();
        Eigen::Vector2d point = Eigen::Vector2d::Zero();
    };

    explicit ResetPath(const PathOptions& options = PathOptions()) : options_(options) {}
    PathResult setGoal(const Robot& robot, const ResetTarget& target,
                       const std::vector<ConvexObstacle>& obstacles, const Fence& fence);
    PathResult step(const Robot& robot) const;
    Projection project(const Eigen::Vector2d& position) const;
    Eigen::Vector2d pointAhead(const Eigen::Vector2d& position, double distance) const;
    void clear();
    const std::vector<Eigen::Vector2d>& path() const {
        return path_;
    }
    const ResetTarget& target() const {
        return target_;
    }
    double planningClearance() const {
        return options_.path_clearance + 0.5 * options_.lookahead;
    }
    double lookahead() const {
        return options_.lookahead;
    }
    bool reached(const Robot& robot) const {
        return withinTargetTolerance(robot, target_, options_);
    }

   private:
    PathOptions options_;
    ResetTarget target_;
    std::vector<Eigen::Vector2d> path_;
    PathStatus status_ = PathStatus::Uninitialized;
    std::string message_;
};

// Replan the complete route to the unchanged target around current peer
// footprints. DWA remains responsible for their future motion.
std::vector<ResetPath> planPeerPaths(const std::vector<Robot>& robots,
                                     const std::vector<ResetPath>& paths,
                                     const std::vector<ConvexObstacle>& obstacles,
                                     const Fence& fence);

}  // namespace ugv_reset_safety
