#include <gtest/gtest.h>
#include <ugv_reset_safety/reset_dwa.h>

#include <cmath>
#include <limits>

using namespace ugv_reset_safety;
namespace {
Robot scout() {
    Robot r;
    r.id = "scout";
    r.yaw = 3.14159265358979323846;
    r.half_length = 0.31;
    r.half_width = 0.26;
    r.limits.max_vx = 0.35;
    r.limits.max_vy = 0.0;
    r.limits.max_omega = 0.5;
    r.limits.accel_vx = r.limits.accel_vy = 0.35;
    r.limits.accel_omega = 0.6;
    return r;
}

TEST(ResetDwa, FourMetresRestoresPositionAndHeadingWithinControllerTimeout) {
    std::vector<Robot> robots{scout()};
    ResetPath path;
    ASSERT_EQ(path.setGoal(robots[0], {{4.0, 0.0}, 0.0}, {}, Fence()).status, PathStatus::Moving);
    ResetDwa dwa;
    DwaConfig config;
    double peak_speed = 0.0;
    bool arrived = false;
    for (int step = 0; step < 2250; ++step) {
        auto& r = robots[0];
        dwa.apply(robots, {path}, {}, Fence(), config);
        ASSERT_TRUE(r.local_plan_feasible);
        EXPECT_LE(std::abs(r.command.x() - r.previous.x()), 0.35 * config.dt + 1e-9);
        EXPECT_LE(std::abs(r.command.z() - r.previous.z()), 0.6 * config.dt + 1e-9);
        EXPECT_LE(std::abs(r.command.x()), r.limits.max_vx);
        EXPECT_DOUBLE_EQ(r.command.y(), 0.0);
        peak_speed = std::max(peak_speed, std::abs(r.command.x()));
        const double mid_yaw = r.yaw + 0.5 * config.dt * r.command.z();
        r.position +=
            config.dt * r.command.x() * Eigen::Vector2d(std::cos(mid_yaw), std::sin(mid_yaw));
        r.yaw += config.dt * r.command.z();
        r.previous = r.command;
        if (withinTargetTolerance(r, path.target()) && r.command.norm() < 1e-6) {
            arrived = true;
            break;
        }
    }
    EXPECT_GT(peak_speed, 0.30);
    EXPECT_LE(std::abs(std::atan2(std::sin(robots[0].yaw), std::cos(robots[0].yaw))),
              10.0 * 3.14159265358979323846 / 180.0);
    EXPECT_TRUE(arrived) << robots[0].position.transpose() << " command "
                         << robots[0].command.transpose();
}

TEST(ResetDwa, AcceptedPoseBrakesResidualMotionBeforeParking) {
    std::vector<Robot> robots{scout()};
    auto& robot = robots[0];
    // Observed terminal pose from the native Scout transport regression.
    robot.position = {0.00508955, 0.04966349};
    robot.yaw = -0.13375718;
    robot.previous = {-0.02, 0.0, 0.01};
    robot.brake_requested = true;
    ResetPath path;
    ASSERT_EQ(path.setGoal(robot, {{0.0, 0.0}, 0.0}, {}, Fence()).status, PathStatus::Reached);
    ResetDwa dwa;
    DwaConfig config;
    for (int tick = 0; tick < 5; ++tick) {
        dwa.apply(robots, {path}, {}, Fence(), config);
        ASSERT_TRUE(robot.local_plan_feasible);
        EXPECT_LE(std::abs(robot.command.x() - robot.previous.x()),
                  robot.limits.accel_vx * config.dt + 1e-9);
        EXPECT_LE(std::abs(robot.command.z() - robot.previous.z()),
                  robot.limits.accel_omega * config.dt + 1e-9);
        robot.previous = robot.command;
    }
    EXPECT_TRUE(robot.command.isZero(0.0));
    EXPECT_FALSE(robot.stop_requested);

    // A braking request must not turn an unsafe footprint into a feasible stop.
    Fence blocked;
    blocked.xmax = robot.position.x();
    dwa.apply(robots, {path}, {}, blocked, config);
    EXPECT_FALSE(robot.local_plan_feasible);
    EXPECT_TRUE(robot.command.isZero(0.0));
}

TEST(ResetDwa, MissingOrInvalidPathCannotProduceMotion) {
    std::vector<Robot> robots{scout()};
    robots[0].command = {-0.2, 0, 0};
    ResetDwa dwa;
    dwa.apply(robots, {}, {}, Fence());
    EXPECT_FALSE(robots[0].local_plan_feasible);
    EXPECT_TRUE(robots[0].command.isZero());
    dwa.apply(robots, {ResetPath()}, {}, Fence());
    EXPECT_FALSE(robots[0].local_plan_feasible);
    EXPECT_TRUE(robots[0].command.isZero());
}

TEST(ResetDwa, ReversingTargetUsesTheNewPathWithoutOldPassingState) {
    ResetDwa dwa;
    std::vector<Robot> robots{scout()};
    ResetPath first, second;
    first.setGoal(robots[0], {{4.0, 0.0}, 0.0}, {}, Fence());
    second.setGoal(robots[0], {{-4.0, 0.0}, 0.0}, {}, Fence());
    dwa.apply(robots, {first}, {}, Fence());
    EXPECT_LT(robots[0].command.x(), 0.0);
    dwa.apply(robots, {second}, {}, Fence());
    EXPECT_GT(robots[0].command.x(), 0.0);
    EXPECT_NEAR(robots[0].command.z(), 0.0, 1e-9);
}
TEST(ResetDwa, InvalidPoseOrWindowFailsClosedAndParkedCommandsAreCleared) {
    const auto initial = scout();
    ResetPath path;
    ASSERT_EQ(path.setGoal(initial, {{4.0, 0.0}, 0.0}, {}, Fence()).status, PathStatus::Moving);
    for (int fault = 0; fault < 4; ++fault) {
        std::vector<Robot> robots{initial};
        robots[0].command = {-0.1, 0.0, 0.0};
        if (fault == 0) {
            robots[0].yaw = std::numeric_limits<double>::quiet_NaN();
        }
        if (fault == 1) {
            robots[0].previous.x() = -0.5;
        }
        if (fault == 2) {
            robots[0].previous.y() = 0.1;
        }
        if (fault == 3) {
            robots[0].limits.accel_omega = 0.0;
        }
        ResetDwa().apply(robots, {path}, {}, Fence());
        EXPECT_FALSE(robots[0].local_plan_feasible) << fault;
        EXPECT_TRUE(robots[0].command.isZero()) << fault;
    }
    std::vector<Robot> parked{initial};
    parked[0].active = false;
    parked[0].command = {-0.1, 0.0, 0.0};
    ResetDwa().apply(parked, {ResetPath()}, {}, Fence());
    EXPECT_TRUE(parked[0].command.isZero());
}

TEST(ResetDwa, MecanumHeadingOnlyResetCrossesAngleWrapAndStops) {
    constexpr double pi = 3.14159265358979323846;
    for (double start : {-pi, -pi + 0.02, 0.0, pi - 0.02, pi}) {
        for (double target : {-pi + 0.02, -0.4, 0.0, pi - 0.02}) {
            std::vector<Robot> robots(1);
            auto& robot = robots.front();
            robot.id = "mecanum";
            robot.type = RobotType::Mecanum;
            robot.yaw = start;
            ResetPath path;
            path.setGoal(robot, {{0.0, 0.0}, target}, {}, Fence());
            ResetDwa dwa;
            bool arrived = false;
            for (int step = 0; step < 2250; ++step) {
                dwa.apply(robots, {path}, {}, Fence());
                ASSERT_TRUE(robot.local_plan_feasible);
                EXPECT_TRUE(robot.command.head<2>().isZero());
                EXPECT_LE(std::abs(robot.command.z() - robot.previous.z()),
                          0.02 * robot.limits.accel_omega + 1.0e-9);
                robot.yaw += 0.02 * robot.command.z();
                robot.previous = robot.command;
                if (withinTargetTolerance(robot, path.target()) && robot.command.isZero(1.0e-6)) {
                    arrived = true;
                    break;
                }
            }
            EXPECT_TRUE(arrived) << "start=" << start << " target=" << target;
        }
    }
}

}  // namespace
