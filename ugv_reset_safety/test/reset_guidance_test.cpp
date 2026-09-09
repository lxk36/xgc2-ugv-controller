#include <gtest/gtest.h>
#include <ugv_reset_safety/reset_guidance.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace ugv_reset_safety {
namespace {
constexpr double kPi = 3.14159265358979323846;
double angle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

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

// Independent exact zero-order-hold kinematics, not a copy of the tracker.
void integrate(Robot* robot, const Eigen::Vector3d& command, double dt) {
    const double delta = command.z() * dt;
    const double scale = std::abs(delta) < 1e-9 ? dt : 2.0 * std::sin(delta / 2.0) / command.z();
    const double yaw_midpoint = robot->yaw + delta / 2.0;
    robot->position +=
        scale * Eigen::Vector2d(
                    std::cos(yaw_midpoint) * command.x() - std::sin(yaw_midpoint) * command.y(),
                    std::sin(yaw_midpoint) * command.x() + std::cos(yaw_midpoint) * command.y());
    robot->yaw = angle(robot->yaw + delta);
}

void expectBounded(const Robot& robot, const Eigen::Vector3d& command) {
    ASSERT_TRUE(command.allFinite());
    EXPECT_LE(std::abs(command.x()), robot.limits.max_vx + 1e-12);
    EXPECT_LE(std::abs(command.y()), robot.limits.max_vy + 1e-12);
    EXPECT_LE(std::abs(command.z()), robot.limits.max_omega + 1e-12);
    if (robot.type == RobotType::Unicycle) {
        EXPECT_DOUBLE_EQ(command.y(), 0.0);
    }
}

double distanceToBox(const Eigen::Vector2d& point, double xmin, double xmax, double ymin,
                     double ymax) {
    const double dx = std::max({xmin - point.x(), 0.0, point.x() - xmax});
    const double dy = std::max({ymin - point.y(), 0.0, point.y() - ymax});
    return std::hypot(dx, dy);
}

TEST(ResetGuidance, BothModelsConvergeFromVariedPositionsAndHeadings) {
    // 288 deterministic initial/goal pose combinations, including side-on,
    // behind, short-distance, wrapped yaw, and independent terminal heading.
    for (RobotType type : {RobotType::Unicycle, RobotType::Mecanum}) {
        for (int heading_index = 0; heading_index < 12; ++heading_index) {
            for (int bearing_index = 0; bearing_index < 12; ++bearing_index) {
                SCOPED_TRACE(::testing::Message()
                             << "type=" << static_cast<int>(type) << " heading=" << heading_index
                             << " bearing=" << bearing_index);
                Robot robot = makeRobot(type);
                robot.position = Eigen::Vector2d(-0.4, 0.2);
                robot.yaw = -kPi + heading_index * kPi / 6.0;
                const double bearing = bearing_index * kPi / 6.0;
                const double distance = 0.08 + 0.21 * (bearing_index + 1);
                ResetTarget target;
                target.position = robot.position +
                                  distance * Eigen::Vector2d(std::cos(bearing), std::sin(bearing));
                target.yaw = angle(0.7 * heading_index - 0.4 * bearing_index);
                ResetGuidance guidance;
                ASSERT_EQ(guidance.setGoal(robot, target, {}, Fence()).status,
                          GuidanceStatus::Moving);
                bool reached = false;
                for (int step = 0; step < 5000; ++step) {
                    const auto result = guidance.step(robot);
                    expectBounded(robot, result.nominal);
                    if (result.status == GuidanceStatus::Reached) {
                        reached = true;
                        EXPECT_TRUE(result.nominal.isZero());
                        break;
                    }
                    integrate(&robot, result.nominal, 0.02);
                }
                ASSERT_TRUE(reached);
                EXPECT_LE((target.position - robot.position).norm(), 0.05);
            }
        }
    }
}

TEST(ResetGuidance, MecanumTransformsWorldVelocityUsingActualYaw) {
    Robot robot = makeRobot(RobotType::Mecanum);
    robot.yaw = kPi / 2.0;
    ResetTarget target;
    target.position = Eigen::Vector2d(1.0, 0.0);
    target.yaw = 0.0;
    ResetGuidance guidance;
    const auto command = guidance.setGoal(robot, target, {}, Fence()).nominal;
    EXPECT_NEAR(command.x(), 0.0, 1e-12);
    EXPECT_LT(command.y(), 0.0);
    EXPECT_LT(command.z(), 0.0);
    EXPECT_NEAR(command.y(), -robot.limits.max_vy, 1e-12);
}

TEST(ResetGuidance, ReverseChoiceIsFixedAcrossNinetyDegreeBoundary) {
    Robot robot = makeRobot();
    robot.yaw = kPi;
    ResetTarget target;
    target.position = Eigen::Vector2d(1.0, 0.0);
    ResetGuidance guidance;
    EXPECT_LT(guidance.setGoal(robot, target, {}, Fence()).nominal.x(), 0.0);
    ASSERT_EQ(guidance.travelDirection(), -1);
    robot.yaw = kPi / 2.0 - 1e-6;
    const auto left = guidance.step(robot);
    robot.yaw = kPi / 2.0 + 1e-6;
    const auto right = guidance.step(robot);
    EXPECT_EQ(guidance.travelDirection(), -1);
    EXPECT_GE(left.nominal.z() * right.nominal.z(), 0.0);
    EXPECT_NEAR(left.nominal.z(), right.nominal.z(), 1e-5);
    EXPECT_LE(left.nominal.x(), 0.0);
    EXPECT_LE(right.nominal.x(), 0.0);
}

TEST(ResetGuidance, LateralPositionErrorProducesCorrectiveYaw) {
    Robot robot = makeRobot();
    robot.position = Eigen::Vector2d(0.0, 0.1);
    robot.yaw = kPi;
    ResetTarget target;
    target.position = Eigen::Vector2d(1.0, 0.0);
    ResetGuidance guidance;
    const auto result = guidance.setGoal(robot, target, {}, Fence());
    EXPECT_LT(result.nominal.z(), 0.0);
    EXPECT_LT(result.nominal.x(), 0.0);
}

TEST(ResetGuidance, AntipodalTurnHasDeterministicSignForEquivalentYaws) {
    Robot robot = makeRobot();
    ResetTarget target;
    target.position.x() = 0.1;
    ResetGuidance guidance;
    guidance.setGoal(robot, target, {}, Fence());
    for (double yaw : {0.0, 2.0 * kPi, -2.0 * kPi}) {
        robot.yaw = yaw;
        for (int repetition = 0; repetition < 5; ++repetition) {
            const auto result = guidance.step(robot);
            EXPECT_GT(result.nominal.z(), 0.0);
            EXPECT_LT(result.nominal.x(), 0.0);
        }
    }
}

TEST(ResetGuidance, CompatibleScoutTargetYawBendsTheReferenceWithoutArrivalGate) {
    Robot robot = makeRobot();
    robot.position.x() = -2.0;
    robot.yaw = kPi;
    ResetTarget above;
    above.yaw = kPi - 0.5;
    ResetTarget below;
    below.yaw = kPi + 0.5;
    ResetGuidance first, second;
    const auto upper = first.setGoal(robot, above, {}, Fence());
    const auto lower = second.setGoal(robot, below, {}, Fence());
    ASSERT_EQ(first.path().size(), 3U);
    ASSERT_EQ(second.path().size(), 3U);
    EXPECT_GT(first.path()[1].y(), 0.0);
    EXPECT_LT(second.path()[1].y(), 0.0);
    EXPECT_GT(upper.nominal.z(), 0.0);
    EXPECT_LT(lower.nominal.z(), 0.0);
    robot.position.setZero();
    robot.yaw = 0.0;
    EXPECT_EQ(first.step(robot).status, GuidanceStatus::Reached);
    EXPECT_TRUE(first.step(robot).nominal.isZero());
}

TEST(ResetGuidance, IncompatibleHeadingDoesNotCreateATurnaroundLoop) {
    Robot robot = makeRobot();
    robot.position.x() = -2.0;
    ResetTarget target;
    target.yaw = 0.0;
    ResetGuidance guidance;
    const auto result = guidance.setGoal(robot, target, {}, Fence());
    EXPECT_EQ(result.status, GuidanceStatus::Moving);
    EXPECT_EQ(guidance.path().size(), 2U);
    EXPECT_NE(result.message.find("heading is unconstrained"), std::string::npos);
}

TEST(ResetGuidance, YawUsesShortestAngleAndXyArrivalDoesNotRequireFinalRotation) {
    Robot robot = makeRobot(RobotType::Mecanum);
    robot.yaw = kPi - 0.1;
    ResetTarget target;
    target.position = Eigen::Vector2d(1.0, 0.0);
    target.yaw = -kPi + 0.1;
    ResetGuidance guidance;
    const auto initial = guidance.setGoal(robot, target, {}, Fence());
    ASSERT_EQ(initial.status, GuidanceStatus::Moving);
    EXPECT_GT(initial.nominal.z(), 0.0);
    robot.position = target.position;
    robot.yaw = 0.0;
    const auto final = guidance.step(robot);
    EXPECT_EQ(final.status, GuidanceStatus::Reached);
    EXPECT_TRUE(final.nominal.isZero());
}

TEST(ResetGuidance, VisibilityGraphRoutesAroundBlockingBox) {
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

TEST(ResetGuidance, NominalClosedLoopNavigatesBoxWithoutCuttingBodyCorners) {
    // Tests nominal guidance only; the coupled QP has its own full-loop tests.
    for (RobotType type : {RobotType::Unicycle, RobotType::Mecanum}) {
        for (int yaw_index = 0; yaw_index < 8; ++yaw_index) {
            SCOPED_TRACE(::testing::Message()
                         << "type=" << static_cast<int>(type) << " yaw=" << yaw_index);
            Robot robot = makeRobot(type);
            robot.position = Eigen::Vector2d(-2.0, 0.0);
            robot.yaw = yaw_index * kPi / 4.0;
            ResetTarget target;
            target.position = Eigen::Vector2d(2.0, 0.0);
            target.yaw = -1.1;
            ResetGuidance guidance;
            ASSERT_EQ(guidance.setGoal(robot, target, {box(-0.4, 0.4, -0.7, 0.7)}, Fence()).status,
                      GuidanceStatus::Moving);
            bool reached = false;
            for (int step = 0; step < 7000; ++step) {
                const auto result = guidance.step(robot);
                expectBounded(robot, result.nominal);
                integrate(&robot, result.nominal, 0.02);
                ASSERT_GT(distanceToBox(robot.position, -0.4, 0.4, -0.7, 0.7), 0.3 + 0.08);
                if (result.status == GuidanceStatus::Reached) {
                    reached = true;
                    break;
                }
            }
            ASSERT_TRUE(reached);
        }
    }
}

TEST(ResetGuidance, ImpossibleRouteRejectsAndClearsPreviousRoute) {
    Robot robot = makeRobot();
    robot.position = Eigen::Vector2d(-2.0, 0.0);
    ResetTarget target;
    target.position = Eigen::Vector2d(2.0, 0.0);
    Fence fence;
    fence.xmin = -3.0;
    fence.xmax = 3.0;
    fence.ymin = -2.0;
    fence.ymax = 2.0;
    ResetGuidance guidance;
    ASSERT_EQ(guidance.setGoal(robot, target, {}, fence).status, GuidanceStatus::Moving);
    const auto rejected = guidance.setGoal(robot, target, {box(-0.2, 0.2, -3.0, 3.0)}, fence);
    EXPECT_EQ(rejected.status, GuidanceStatus::NoRoute);
    EXPECT_TRUE(guidance.path().empty());
    EXPECT_TRUE(guidance.step(robot).nominal.isZero());
}

TEST(ResetGuidance, RejectsInvalidGeometryAndOccupiedTarget) {
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

TEST(ResetGuidance, ClockwiseAndCollinearConvexGeometryHaveEquivalentRoutes) {
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

TEST(ResetGuidance, SeededBoxPlacementsProduceGeometricallyClearRoutes) {
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

TEST(ResetGuidance, NonFinitePoseInvalidatesAnActiveRoute) {
    Robot robot = makeRobot();
    ResetTarget target;
    target.position.x() = 1.0;
    ResetGuidance guidance;
    ASSERT_EQ(guidance.setGoal(robot, target, {}, Fence()).status, GuidanceStatus::Moving);
    robot.yaw = std::numeric_limits<double>::infinity();
    const auto result = guidance.step(robot);
    EXPECT_EQ(result.status, GuidanceStatus::InvalidInput);
    EXPECT_TRUE(result.nominal.isZero());
}

TEST(ResetGuidance, ScoutReachesWithBoundedYawCoupledLateralMotionAndLag) {
    // This is an explicit perturbation family, not a calibrated Scout tire
    // model or a proof for arbitrary friction/delay. The lateral term changes
    // sign with yaw rate and exercises the prior yaw-amplification concern.
    for (double lateral_coupling : {-0.25, -0.229, -0.08, 0.0}) {
        for (int heading = 0; heading < 8; ++heading) {
            Robot robot = makeRobot();
            robot.position = Eigen::Vector2d(-1.4, 0.4);
            robot.yaw = heading * kPi / 4.0;
            ResetTarget target;
            target.position = Eigen::Vector2d(1.0, -0.6);
            target.yaw = -1.2;
            ResetGuidance guidance;
            ASSERT_EQ(guidance.setGoal(robot, target, {}, Fence()).status, GuidanceStatus::Moving);
            Eigen::Vector3d executed = Eigen::Vector3d::Zero();
            Eigen::Vector3d actual = Eigen::Vector3d::Zero();
            bool settled = false;
            int reached_samples = 0;
            for (int step = 0; step < 10000; ++step) {
                const auto result = guidance.step(robot);
                // The production QP owns command slew; this independent actuator
                // surrogate applies conservative finite acceleration plus 150 ms lag.
                for (int axis : {0, 2}) {
                    const double maximum_change = (axis == 0 ? 0.4 : 0.8) * 0.02;
                    executed[axis] +=
                        std::max(-maximum_change,
                                 std::min(maximum_change, result.nominal[axis] - executed[axis]));
                    actual[axis] +=
                        (1.0 - std::exp(-0.02 / 0.15)) * (executed[axis] - actual[axis]);
                }
                actual.y() = lateral_coupling * actual.z();
                integrate(&robot, actual, 0.02);
                if (result.status == GuidanceStatus::Reached && actual.norm() < 0.01) {
                    ++reached_samples;
                } else {
                    reached_samples = 0;
                }
                if (reached_samples >= 25) {
                    settled = true;
                    break;
                }
            }
            ASSERT_TRUE(settled) << "lateral=" << lateral_coupling << " heading=" << heading
                                 << " position=" << robot.position.transpose()
                                 << " yaw=" << robot.yaw
                                 << " command=" << guidance.step(robot).nominal.transpose();
            EXPECT_LE((robot.position - target.position).norm(), 0.06);
        }
    }
}

TEST(ResetGuidance, ScoutSmallOffsetInitialPosesWithYawCoupling) {
    for (double range : {0.051, 0.10, 0.30}) {
        for (int heading = 0; heading < 8; ++heading) {
            Robot robot = makeRobot();
            robot.yaw = heading * kPi / 4.0;
            ResetTarget target;
            target.position.x() = range;
            ResetGuidance guidance;
            guidance.setGoal(robot, target, {}, Fence());
            bool reached = false;
            for (int step = 0; step < 10000; ++step) {
                const auto result = guidance.step(robot);
                Eigen::Vector3d actual = result.nominal;
                actual.y() = -0.229 * actual.z();
                integrate(&robot, actual, 0.02);
                if (result.status == GuidanceStatus::Reached) {
                    reached = true;
                    break;
                }
            }
            ASSERT_TRUE(reached) << "range=" << range << " heading=" << heading
                                 << " position=" << robot.position.transpose();
        }
    }
}

TEST(ResetGuidance, ScoutProductionLimitsWithCloseGoalsAndActuatorLag) {
    // Fixed production envelope for all 120 cases; the controller does not
    // receive the actual perturbation value or special-case a starting pose.
    for (double coupling : {0.0, -0.229, -0.25}) {
        for (double range : {0.051, 0.08, 0.12, 0.30, 2.4}) {
            for (double bearing : {0.0, kPi / 2.0}) {
                for (double yaw : {0.0, 0.9, kPi / 2.0, kPi}) {
                    Robot robot = makeRobot();
                    robot.yaw = yaw;
                    robot.limits.max_vx = 0.35;
                    robot.limits.max_omega = 0.5;
                    robot.limits.accel_vx = 0.35;
                    robot.limits.accel_omega = 0.6;
                    ResetTarget target;
                    target.position = range * Eigen::Vector2d(std::cos(bearing), std::sin(bearing));
                    target.yaw = -0.8;
                    ResetGuidance guidance;
                    guidance.setGoal(robot, target, {}, Fence());
                    Eigen::Vector3d command = Eigen::Vector3d::Zero();
                    Eigen::Vector3d actual = Eigen::Vector3d::Zero();
                    int settled_samples = 0;
                    for (int step = 0; step < 8000; ++step) {
                        const auto desired = guidance.step(robot);
                        expectBounded(robot, desired.nominal);
                        for (int axis : {0, 2}) {
                            const double slew = 0.02 * (axis == 0 ? 0.35 : 0.6);
                            command[axis] += std::max(
                                -slew, std::min(slew, desired.nominal[axis] - command[axis]));
                            const double lag = axis == 0 ? 0.12 : 0.16;
                            actual[axis] +=
                                (1.0 - std::exp(-0.02 / lag)) * (command[axis] - actual[axis]);
                        }
                        actual.y() = coupling * actual.z();
                        integrate(&robot, actual, 0.02);
                        if (desired.status == GuidanceStatus::Reached && actual.norm() < 0.01) {
                            ++settled_samples;
                        } else {
                            settled_samples = 0;
                        }
                        if (settled_samples >= 25) {
                            break;
                        }
                    }
                    ASSERT_GE(settled_samples, 25)
                        << "coupling=" << coupling << " range=" << range << " bearing=" << bearing
                        << " yaw=" << yaw
                        << " residual=" << (target.position - robot.position).norm();
                    EXPECT_LE((target.position - robot.position).norm(), 0.05);
                }
            }
        }
    }
}

}  // namespace
}  // namespace ugv_reset_safety
