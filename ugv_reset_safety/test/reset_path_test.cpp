#include <gtest/gtest.h>
#include <ugv_reset_safety/reset_path.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>

namespace ugv_reset_safety {
namespace {
constexpr double kPi = 3.14159265358979323846;

Robot makeRobot(RobotType type = RobotType::Unicycle) {
    Robot robot;
    robot.id = "robot";
    robot.type = type;
    robot.lateral_velocity_per_yaw_bound = type == RobotType::Unicycle ? 0.25 : 0.0;
    robot.half_length = 0.24;
    robot.half_width = 0.18;
    robot.limits.max_vx = 0.45;
    robot.limits.max_vy = type == RobotType::Mecanum ? 0.35 : 0.0;
    robot.limits.max_omega = 0.75;
    return robot;
}

ConvexObstacle box(double xmin, double xmax, double ymin, double ymax) {
    ConvexObstacle obstacle;
    obstacle.id = "box";
    obstacle.vertices = {{xmin, ymin}, {xmax, ymin}, {xmax, ymax}, {xmin, ymax}};
    return obstacle;
}

double distanceToBox(const Eigen::Vector2d& point, double xmin, double xmax, double ymin,
                     double ymax) {
    const double dx = std::max({xmin - point.x(), 0.0, point.x() - xmax});
    const double dy = std::max({ymin - point.y(), 0.0, point.y() - ymax});
    return std::hypot(dx, dy);
}

TEST(SafetyFootprint, CoversRectangleIncludingCornersAfterPoseTransform) {
    Robot value = makeRobot();
    value.position = Eigen::Vector2d(1.1, -2.3);
    value.body_center_offset = Eigen::Vector2d(0.13, -0.04);
    for (double yaw : {-kPi, -2.1, -0.1, 0.0, 0.8, kPi}) {
        value.yaw = yaw;
        const auto disks = coveringDisks(value, 2);
        ASSERT_EQ(disks.size(), 2U);
        Eigen::Matrix2d rotation;
        rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
        for (int x = 0; x <= 10; ++x) {
            for (int y = 0; y <= 10; ++y) {
                const Eigen::Vector2d body(-value.half_length + 2.0 * value.half_length * x / 10.0,
                                           -value.half_width + 2.0 * value.half_width * y / 10.0);
                const Eigen::Vector2d point =
                    value.position + rotation * (body + value.body_center_offset);
                double gap = std::numeric_limits<double>::infinity();
                for (const auto& disk : disks) {
                    gap = std::min(gap, (point - disk.center).norm() - disk.radius);
                }
                EXPECT_LE(gap, 1.0e-12);
            }
        }
    }
}

TEST(ResetPath, VisibilityGraphRoutesAroundBlockingBox) {
    const auto obstacle = box(-0.4, 0.4, -0.7, 0.7);
    const auto path = planVisibilityPath({-2.0, 0.0}, {2.0, 0.0}, {obstacle}, Fence(), 0.3, 0.18);
    ASSERT_TRUE(path.valid) << path.message;
    ASSERT_GE(path.points.size(), 4U);
    for (std::size_t i = 1; i < path.points.size(); ++i) {
        for (int sample = 0; sample <= 100; ++sample) {
            const double ratio = sample / 100.0;
            const Eigen::Vector2d point =
                (1.0 - ratio) * path.points[i - 1] + ratio * path.points[i];
            EXPECT_GE(distanceToBox(point, -0.4, 0.4, -0.7, 0.7), 0.48 - 1e-8);
        }
    }
}

TEST(ResetPath, ImpossibleRouteRejectsAndClearsPreviousRoute) {
    Robot robot = makeRobot();
    robot.position = Eigen::Vector2d(-2.0, 0.0);
    ResetTarget target;
    target.position = Eigen::Vector2d(2.0, 0.0);
    Fence fence;
    fence.xmin = -3.0;
    fence.xmax = 3.0;
    fence.ymin = -2.0;
    fence.ymax = 2.0;
    ResetPath guidance;
    ASSERT_EQ(guidance.setGoal(robot, target, {}, fence).status, PathStatus::Moving);
    const auto rejected = guidance.setGoal(robot, target, {box(-0.2, 0.2, -3.0, 3.0)}, fence);
    EXPECT_EQ(rejected.status, PathStatus::NoRoute);
    EXPECT_TRUE(guidance.path().empty());
}

TEST(ResetPath, RejectsInvalidGeometryAndOccupiedTarget) {
    const auto occupied =
        planVisibilityPath({-2.0, 0.0}, {0.0, 0.0}, {box(-0.5, 0.5, -0.5, 0.5)}, Fence(), 0.3, 0.1);
    EXPECT_FALSE(occupied.valid);
    EXPECT_FALSE(occupied.invalid_input);
    ConvexObstacle concave;
    concave.vertices = {{0, 0}, {1, 0}, {0.3, 0.3}, {1, 1}, {0, 1}};
    EXPECT_TRUE(planVisibilityPath({-2, 0}, {2, 0}, {concave}, Fence(), 0.3, 0.1).invalid_input);
    auto duplicate = box(-0.5, 0.5, -0.5, 0.5);
    duplicate.vertices.push_back(duplicate.vertices.front());
    EXPECT_TRUE(planVisibilityPath({-2, 0}, {2, 0}, {duplicate}, Fence(), 0.3, 0.1).invalid_input);
    EXPECT_TRUE(planVisibilityPath({std::numeric_limits<double>::quiet_NaN(), 0}, {2, 0}, {},
                                   Fence(), 0.3, 0.1)
                    .invalid_input);
}

TEST(ResetPath, ClockwiseAndCollinearConvexGeometryHaveEquivalentRoutes) {
    auto obstacle = box(-0.5, 0.5, -0.5, 0.5);
    const auto baseline = planVisibilityPath({-2, 0}, {2, 0}, {obstacle}, Fence(), 0.3, 0.1);
    std::reverse(obstacle.vertices.begin(), obstacle.vertices.end());
    obstacle.vertices.insert(obstacle.vertices.begin() + 1, Eigen::Vector2d(0, 0.5));
    const auto alternative = planVisibilityPath({-2, 0}, {2, 0}, {obstacle}, Fence(), 0.3, 0.1);
    ASSERT_TRUE(baseline.valid);
    ASSERT_TRUE(alternative.valid) << alternative.message;
    auto length = [](const VisibilityPath& path) {
        double result = 0.0;
        for (std::size_t i = 1; i < path.points.size(); ++i) {
            result += (path.points[i] - path.points[i - 1]).norm();
        }
        return result;
    };
    EXPECT_NEAR(length(baseline), length(alternative), 1e-10);
}

TEST(ResetPath, SeededBoxPlacementsProduceGeometricallyClearRoutes) {
    std::mt19937 random(20260910);
    std::uniform_real_distribution<double> center(-0.8, 0.8);
    std::uniform_real_distribution<double> extent(0.1, 0.6);
    for (int trial = 0; trial < 100; ++trial) {
        const double x = center(random), y = center(random), hx = extent(random),
                     hy = extent(random);
        const auto path = planVisibilityPath(
            {-2.5, -0.2}, {2.5, 0.2}, {box(x - hx, x + hx, y - hy, y + hy)}, Fence(), 0.3, 0.1);
        ASSERT_TRUE(path.valid) << "trial=" << trial << " " << path.message;
        for (std::size_t i = 1; i < path.points.size(); ++i) {
            for (int sample = 0; sample <= 100; ++sample) {
                const double ratio = sample / 100.0;
                const Eigen::Vector2d p =
                    (1.0 - ratio) * path.points[i - 1] + ratio * path.points[i];
                ASSERT_GE(distanceToBox(p, x - hx, x + hx, y - hy, y + hy), 0.4 - 1e-8);
            }
        }
    }
}

TEST(ResetPath, NonFinitePoseInvalidatesAnActiveRoute) {
    Robot robot = makeRobot();
    ResetTarget target;
    target.position.x() = 1.0;
    ResetPath guidance;
    ASSERT_EQ(guidance.setGoal(robot, target, {}, Fence()).status, PathStatus::Moving);
    robot.yaw = std::numeric_limits<double>::infinity();
    const auto result = guidance.step(robot);
    EXPECT_EQ(result.status, PathStatus::InvalidInput);
}

TEST(ResetPath, PeerDetoursKeepTheOriginalGoalAndChooseOppositePassingSides) {
    std::vector<Robot> robots{makeRobot(), makeRobot()};
    robots[0].id = "left";
    robots[1].id = "right";
    robots[0].position = {-2.0, 0.0};
    robots[1].position = {2.0, 0.0};
    std::vector<ResetPath> paths(2);
    paths[0].setGoal(robots[0], {{4.0, 0.0}, 0.0}, {}, Fence());
    paths[1].setGoal(robots[1], {{-4.0, 0.0}, 0.0}, {}, Fence());
    const auto routes = planPeerPaths(robots, paths, {}, Fence());
    ASSERT_EQ(routes.size(), 2U);
    for (std::size_t i = 0; i < routes.size(); ++i) {
        ASSERT_GT(routes[i].path().size(), 2U);
        EXPECT_EQ(paths[i].path().size(), 2U);
        EXPECT_TRUE(routes[i].target().position.isApprox(paths[i].target().position));
        EXPECT_TRUE(routes[i].path().back().isApprox(paths[i].target().position));
    }
    EXPECT_LT(routes[0].path()[1].y(), 0.0);
    EXPECT_GT(routes[1].path()[1].y(), 0.0);
}

}  // namespace
}  // namespace ugv_reset_safety
