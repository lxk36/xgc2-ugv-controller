#include <gtest/gtest.h>
#include <ugv_reset_safety/reset_dwa.h>
#include <ugv_reset_safety/reset_path.h>

#include <cmath>
using namespace ugv_reset_safety;
TEST(ResetClosedLoop, SingleRobotWithObstacle) {
    for (auto type : {RobotType::Unicycle, RobotType::Mecanum}) {
        Robot robot;
        robot.id = "ugv1";
        robot.type = type;
        robot.position = {-2, 0};
        robot.half_length = .25;
        robot.half_width = .2;
        robot.limits.max_vx = .35;
        robot.limits.max_vy = .35;
        robot.limits.max_omega = .7;
        Fence fence;
        fence.xmin = -5;
        fence.xmax = 5;
        fence.ymin = -5;
        fence.ymax = 5;
        std::vector<ConvexObstacle> obstacles{
            {"box", {{-.3, -.35}, {.3, -.35}, {.3, .35}, {-.3, .35}}}};
        ResetPath path;
        ResetDwa dwa;
        ResetTarget target;
        target.position = {2, 0};
        target.yaw = 0;
        ASSERT_NE(path.setGoal(robot, target, obstacles, fence).status, PathStatus::NoRoute);
        DwaConfig cfg;
        cfg.dt = .02;
        cfg.clearance = .08;
        cfg.uncertainty_margin = .03;
        bool arrived = false;
        for (int i = 0; i < 6000; ++i) {
            auto g = path.step(robot);
            robot.active = true;
            std::vector<Robot> robots{robot};
            dwa.apply(robots, {path}, obstacles, fence, cfg);
            ASSERT_TRUE(robots[0].local_plan_feasible) << "step " << i;
            const auto u = robots[0].command;
            EXPECT_LE((u - robot.previous).cwiseAbs().maxCoeff(), .02 * .8 + 1e-5);
            robot = robots[0];
            robot.previous = u;
            const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
            robot.position += .02 * Eigen::Vector2d(c * u.x() - s * u.y(), s * u.x() + c * u.y());
            robot.yaw += .02 * u.z();
            if (g.status == PathStatus::Reached && u.norm() < .005) {
                arrived = true;
                break;
            }
        }
        EXPECT_TRUE(arrived) << "type " << static_cast<int>(type) << " residual "
                             << (robot.position - target.position).norm();
    }
}

TEST(ResetDwa, BlockedSampleIsFailClosedZero) {
    Robot robot;
    robot.id = "ugv1";
    robot.active = true;
    robot.position = {0.0, 0.0};
    robot.previous = {0.2, 0.0, 0.0};
    robot.half_length = 0.25;
    robot.half_width = 0.2;
    robot.limits.max_vx = 0.35;
    robot.limits.max_vy = 0.35;
    robot.limits.max_omega = 0.7;
    robot.command = {0.2, 0.0, 0.0};
    Fence fence;
    fence.xmin = -5;
    fence.xmax = 5;
    fence.ymin = -5;
    fence.ymax = 5;
    ConvexObstacle box{"box", {{-0.05, -0.05}, {0.05, -0.05}, {0.05, 0.05}, {-0.05, 0.05}}};
    DwaConfig cfg;
    cfg.dt = 0.02;
    ResetDwa dwa;
    std::vector<Robot> robots{robot};
    ResetPath path;
    path.setGoal(robot, {{2.0, 0.0}, 0.0}, {}, fence);
    dwa.apply(robots, {path}, {box}, fence, cfg);
    EXPECT_FALSE(robots[0].local_plan_feasible);
    EXPECT_TRUE(robots[0].command.isZero(1.0e-12));
}
