#include "ugv_reset_safety/safety_filter.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ugv_reset_safety {
namespace {

constexpr double kPi = 3.14159265358979323846;

Robot robot(const std::string& id = "scout") {
    Robot value;
    value.id = id;
    value.half_length = 0.25;
    value.half_width = 0.15;
    value.limits.max_vx = 0.6;
    value.limits.max_vy = 0.6;
    value.limits.max_omega = 1.0;
    value.limits.accel_vx = 100.0;
    value.limits.accel_vy = 100.0;
    value.limits.accel_omega = 100.0;
    return value;
}

FilterConfig config() {
    FilterConfig value;
    value.clearance = 0.05;
    value.barrier_gain = 0.5;
    value.smoothing_weight = 0.0;
    return value;
}

ConvexObstacle box(double xmin, double xmax, double ymin, double ymax) {
    return {"box", {{xmin, ymin}, {xmax, ymin}, {xmax, ymax}, {xmin, ymax}}};
}

Fence noFence() {
    Fence fence;
    fence.enabled = false;
    return fence;
}

TEST(SafetyFootprint, CoversRectangleIncludingCornersAfterPoseTransform) {
    Robot value = robot();
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

TEST(SafetyFootprint, VelocityMapMatchesIndependentPoseFiniteDifference) {
    for (RobotType type : {RobotType::Unicycle, RobotType::Mecanum}) {
        Robot value = robot();
        value.type = type;
        value.position = Eigen::Vector2d(-0.3, 0.8);
        value.yaw = 1.2;
        value.body_center_offset = Eigen::Vector2d(0.07, -0.04);
        const Eigen::Vector3d command(0.3, type == RobotType::Unicycle ? 0.0 : -0.15, -0.4);
        const auto original = coveringDisks(value, 2);
        const double dt = 1.0e-7;
        const double c = std::cos(value.yaw);
        const double s = std::sin(value.yaw);
        value.position += dt * Eigen::Vector2d(c * command.x() - s * command.y(),
                                               s * command.x() + c * command.y());
        value.yaw += dt * command.z();
        const auto moved = coveringDisks(value, 2);
        for (std::size_t i = 0; i < original.size(); ++i) {
            EXPECT_LT(
                ((moved[i].center - original[i].center) / dt - original[i].velocity_map * command)
                    .norm(),
                1.0e-7);
        }
    }
}

TEST(SafetyFilter, ObliqueObstacleCanChangeScoutSteering) {
    Robot value = robot();
    value.nominal.x() = 0.5;
    const auto result =
        solveSafetyFilter({value}, {box(0.55, 0.85, 0.12, 0.42)}, noFence(), config());
    ASSERT_TRUE(result.ok()) << result.detail;
    ASSERT_EQ(result.commands.size(), 1U);
    EXPECT_LT(result.commands[0].z(), -1.0e-4);
    EXPECT_LT(result.commands[0].x(), value.nominal.x());
    EXPECT_NEAR(result.commands[0].y(), 0.0, 1.0e-7);
    EXPECT_LE(result.max_constraint_violation, 1.0e-6);
}

TEST(SafetyFilter, HeadOnBarrierDoesNotInventAGlobalNavigationGuarantee) {
    Robot value = robot();
    value.nominal.x() = 0.5;
    const auto result = solveSafetyFilter({value}, {box(0.5, 0.8, -2.0, 2.0)}, noFence(), config());
    ASSERT_TRUE(result.ok()) << result.detail;
    EXPECT_LT(result.commands[0].x(), 0.1);
    EXPECT_NEAR(result.commands[0].z(), 0.0, 1.0e-7);
}

TEST(SafetyFilter, JointPairConstraintRestrictsBothApproachingRobots) {
    Robot first = robot("first");
    Robot second = robot("second");
    first.position.x() = -0.5;
    second.position.x() = 0.5;
    second.yaw = kPi;
    first.nominal.x() = second.nominal.x() = 0.5;
    const auto result = solveSafetyFilter({first, second}, {}, noFence(), config());
    ASSERT_TRUE(result.ok()) << result.detail;
    EXPECT_GT(result.commands[0].x(), 0.0);
    EXPECT_LT(result.commands[0].x(), 0.1);
    EXPECT_NEAR(result.commands[0].x(), result.commands[1].x(), 1.0e-6);
    const auto a = coveringDisks(first, 2);
    const auto b = coveringDisks(second, 2);
    for (const auto& da : a) {
        for (const auto& db : b) {
            const Eigen::Vector2d delta = da.center - db.center;
            const double radius = da.radius + db.radius + config().clearance;
            const double h = delta.squaredNorm() - radius * radius;
            const double derivative = 2.0 * delta.dot(da.velocity_map * result.commands[0] -
                                                      db.velocity_map * result.commands[1]);
            EXPECT_GE(derivative + config().barrier_gain * h, -1.0e-6);
        }
    }
}

TEST(SafetyFilter, VelocityAndSlewAreSolvedTogetherInBodyFrame) {
    Robot value = robot();
    value.type = RobotType::Mecanum;
    value.yaw = 1.7;
    value.previous = Eigen::Vector3d(0.1, 0.1, 0.1);
    value.nominal = Eigen::Vector3d(8.0, -8.0, 8.0);
    value.limits.accel_vx = 0.5;
    value.limits.accel_vy = 0.4;
    value.limits.accel_omega = 0.3;
    auto cfg = config();
    cfg.dt = 0.1;
    const auto result = solveSafetyFilter({value}, {}, noFence(), cfg);
    ASSERT_TRUE(result.ok()) << result.detail;
    EXPECT_NEAR(result.commands[0].x(), 0.15, 1.0e-6);
    EXPECT_NEAR(result.commands[0].y(), 0.06, 1.0e-6);
    EXPECT_NEAR(result.commands[0].z(), 0.13, 1.0e-6);
}

TEST(SafetyFilter, InsufficientBrakingIsExplicitlyInfeasible) {
    Robot value = robot();
    value.previous.x() = 0.4;
    value.nominal.x() = 0.4;
    value.limits.accel_vx = 0.1;
    const auto result = solveSafetyFilter({value}, {box(0.5, 0.8, -2.0, 2.0)}, noFence(), config());
    EXPECT_EQ(result.status, Status::Infeasible) << result.detail;
    EXPECT_TRUE(result.commands.empty());
    EXPECT_GT(result.min_clearance, 0.0);
}

TEST(SafetyFilter, RearFootprintCollisionIsRejected) {
    const auto result =
        solveSafetyFilter({robot()}, {box(-0.31, -0.28, -0.02, 0.02)}, noFence(), config());
    EXPECT_EQ(result.status, Status::UnsafeInitialState);
    EXPECT_TRUE(result.commands.empty());
}

TEST(SafetyFilter, FenceCoversTheBodyAndCanAffectAngularVelocity) {
    Robot value = robot();
    value.yaw = 0.6;
    value.nominal.z() = 0.8;
    Fence fence;
    fence.ymax = 0.38;
    const auto result = solveSafetyFilter({value}, {}, fence, config());
    ASSERT_TRUE(result.ok()) << result.detail;
    EXPECT_LT(result.commands[0].z(), value.nominal.z());
    value.position.y() = 0.2;
    EXPECT_EQ(solveSafetyFilter({value}, {}, fence, config()).status, Status::UnsafeInitialState);
}

TEST(SafetyFilter, AcceptsBothConvexPolygonWindings) {
    auto obstacle = box(0.6, 0.8, 0.1, 0.4);
    Robot value = robot();
    value.nominal.x() = 0.5;
    const auto first = solveSafetyFilter({value}, {obstacle}, noFence(), config());
    std::reverse(obstacle.vertices.begin(), obstacle.vertices.end());
    const auto second = solveSafetyFilter({value}, {obstacle}, noFence(), config());
    ASSERT_TRUE(first.ok()) << first.detail;
    ASSERT_TRUE(second.ok()) << second.detail;
    EXPECT_LT((first.commands[0] - second.commands[0]).norm(), 1.0e-8);
}

TEST(SafetyFilter, RejectsBadGeometryInputsAndUnobservedMotionAssumptions) {
    auto cfg = config();
    cfg.dt = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(solveSafetyFilter({robot()}, {}, noFence(), cfg).status, Status::InvalidInput);
    Robot value = robot();
    value.active = false;
    value.previous.x() = 0.1;
    EXPECT_EQ(solveSafetyFilter({value}, {}, noFence(), config()).status, Status::InvalidInput);
    value = robot();
    value.yaw = std::numeric_limits<double>::infinity();
    EXPECT_EQ(solveSafetyFilter({value}, {}, noFence(), config()).status, Status::InvalidInput);
    const ConvexObstacle concave{"concave",
                                 {{1.0, 0.0}, {2.0, 0.0}, {1.5, 0.2}, {2.0, 1.0}, {1.0, 1.0}}};
    EXPECT_EQ(solveSafetyFilter({robot()}, {concave}, noFence(), config()).status,
              Status::InvalidGeometry);
    const ConvexObstacle crossing{"crossing", {{1.0, 0.0}, {2.0, 1.0}, {2.0, 0.0}, {1.0, 1.0}}};
    EXPECT_EQ(solveSafetyFilter({robot()}, {crossing}, noFence(), config()).status,
              Status::InvalidGeometry);
    EXPECT_EQ(solveSafetyFilter({robot(), robot()}, {}, noFence(), config()).status,
              Status::InvalidInput);
}

TEST(SafetyFilter, KnownStationaryPeerRemainsAFullFootprintObstacle) {
    Robot first = robot("active");
    Robot second = robot("parked");
    first.position.x() = -0.5;
    second.position.x() = 0.5;
    first.nominal.x() = 0.5;
    second.active = false;
    const auto result = solveSafetyFilter({first, second}, {}, noFence(), config());
    ASSERT_TRUE(result.ok()) << result.detail;
    EXPECT_LT(result.commands[0].x(), first.nominal.x());
    EXPECT_EQ(result.commands[1], Eigen::Vector3d::Zero());
    EXPECT_DOUBLE_EQ(result.commands[0].y(), 0.0);
}

TEST(SafetyFilter, RequestedStopReturnsCertifiedExactZeroAndRetainsSlew) {
    auto value = robot();
    value.stop_requested = true;
    value.nominal = Eigen::Vector3d(0.4, 0.0, -0.3);
    const auto stopped = solveSafetyFilter({value}, {}, noFence(), config());
    ASSERT_TRUE(stopped.ok()) << stopped.detail;
    EXPECT_EQ(stopped.commands[0], Eigen::Vector3d::Zero());
    value.previous.x() = 0.2;
    value.limits.accel_vx = 0.1;
    const auto braking = solveSafetyFilter({value}, {}, noFence(), config());
    EXPECT_EQ(braking.status, Status::Infeasible);
    EXPECT_TRUE(braking.commands.empty());
}

TEST(SafetyFilter, RequestedStopCannotRemoveActiveVelocityUncertainty) {
    auto value = robot();
    auto cfg = config();
    cfg.velocity_uncertainty = 0.1;
    const auto near = box(0.40, 0.7, -0.5, 0.5);
    const auto moving = solveSafetyFilter({value}, {near}, noFence(), cfg);
    ASSERT_TRUE(moving.ok()) << moving.detail;
    EXPECT_LT(moving.commands[0].x(), 0.0);
    value.stop_requested = true;
    const auto stopped = solveSafetyFilter({value}, {near}, noFence(), cfg);
    EXPECT_EQ(stopped.status, Status::Infeasible) << stopped.detail;
    EXPECT_TRUE(stopped.commands.empty());
}

TEST(SafetyFilter, EveryLateralCouplingExtremeSatisfiesPairBarrier) {
    Robot first = robot("first");
    Robot second = robot("second");
    first.position = Eigen::Vector2d(-0.5, -0.15);
    second.position = Eigen::Vector2d(0.5, 0.15);
    first.yaw = 0.1;
    second.yaw = kPi + 0.1;
    first.nominal = Eigen::Vector3d(0.5, 0.0, 0.4);
    second.nominal = Eigen::Vector3d(0.5, 0.0, -0.4);
    first.lateral_velocity_per_yaw_bound = second.lateral_velocity_per_yaw_bound = 0.229;
    auto cfg = config();
    cfg.velocity_uncertainty = 0.02;
    const auto result = solveSafetyFilter({first, second}, {}, noFence(), cfg);
    ASSERT_TRUE(result.ok()) << result.detail;
    const Eigen::Vector2d left_a(-std::sin(first.yaw), std::cos(first.yaw));
    const Eigen::Vector2d left_b(-std::sin(second.yaw), std::cos(second.yaw));
    for (const auto& a : coveringDisks(first, 2)) {
        for (const auto& b : coveringDisks(second, 2)) {
            const Eigen::Vector2d d = a.center - b.center;
            const double radius = a.radius + b.radius + cfg.clearance;
            const double h = d.squaredNorm() - radius * radius;
            for (double sa : {-1.0, 1.0}) {
                for (double sb : {-1.0, 1.0}) {
                    const Eigen::Vector2d va =
                        a.velocity_map * result.commands[0] +
                        sa * first.lateral_velocity_per_yaw_bound * result.commands[0].z() * left_a;
                    const Eigen::Vector2d vb = b.velocity_map * result.commands[1] +
                                               sb * second.lateral_velocity_per_yaw_bound *
                                                   result.commands[1].z() * left_b;
                    const double residual = 2.0 * d.dot(va - vb) + cfg.barrier_gain * h -
                                            4.0 * d.norm() * cfg.velocity_uncertainty;
                    EXPECT_GE(residual, -1.0e-6);
                }
            }
        }
    }
}

TEST(SafetyFilter, EveryLateralCouplingExtremeSatisfiesObstacleAndFenceBarriers) {
    Robot value = robot();
    value.yaw = 0.3;
    value.lateral_velocity_per_yaw_bound = 0.229;
    value.nominal = Eigen::Vector3d(0.5, 0.0, 0.6);
    Fence fence;
    fence.ymax = 0.5;
    const auto obstacle = box(0.6, 1.0, 0.15, 0.45);
    const auto cfg = config();
    const auto result = solveSafetyFilter({value}, {obstacle}, fence, cfg);
    ASSERT_TRUE(result.ok()) << result.detail;
    const Eigen::Vector2d left(-std::sin(value.yaw), std::cos(value.yaw));
    for (const auto& disk : coveringDisks(value, 2)) {
        const Eigen::Vector2d nearest(std::clamp(disk.center.x(), 0.6, 1.0),
                                      std::clamp(disk.center.y(), 0.15, 0.45));
        const Eigen::Vector2d delta = disk.center - nearest;
        const double radius = disk.radius + cfg.clearance;
        const double h = delta.squaredNorm() - radius * radius;
        for (double sign : {-1.0, 1.0}) {
            const Eigen::Vector2d velocity =
                disk.velocity_map * result.commands[0] +
                sign * value.lateral_velocity_per_yaw_bound * result.commands[0].z() * left;
            EXPECT_GE(2.0 * delta.dot(velocity) + cfg.barrier_gain * h, -1.0e-6);
            EXPECT_GE(-velocity.y() + cfg.barrier_gain * (fence.ymax - disk.center.y() - radius),
                      -1.0e-6);
        }
    }
    value.lateral_velocity_per_yaw_bound = -0.1;
    EXPECT_EQ(solveSafetyFilter({value}, {}, fence, cfg).status, Status::InvalidInput);
}

}  // namespace
}  // namespace ugv_reset_safety
