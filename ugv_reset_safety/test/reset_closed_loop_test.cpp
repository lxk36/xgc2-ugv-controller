#include <gtest/gtest.h>
#include <ugv_reset_safety/reset_guidance.h>

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
        ResetGuidance guidance;
        ResetTarget target;
        target.position = {2, 0};
        target.yaw = 0;
        ASSERT_NE(guidance.setGoal(robot, target, obstacles, fence).status,
                  GuidanceStatus::NoRoute);
        FilterConfig cfg;
        cfg.dt = .02;
        cfg.clearance = .08;
        cfg.uncertainty_margin = .03;
        bool arrived = false;
        for (int i = 0; i < 6000; ++i) {
            auto g = guidance.step(robot);
            robot.nominal = g.nominal;
            auto result = solveSafetyFilter({robot}, obstacles, fence, cfg);
            ASSERT_TRUE(result.ok()) << "step " << i << ": " << result.detail;
            ASSERT_GE(result.min_clearance, -1e-6);
            const auto u = result.commands[0];
            EXPECT_LE((u - robot.previous).cwiseAbs().maxCoeff(), .02 * .8 + 1e-5);
            robot.previous = u;
            const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
            robot.position += .02 * Eigen::Vector2d(c * u.x() - s * u.y(), s * u.x() + c * u.y());
            robot.yaw += .02 * u.z();
            if (g.status == GuidanceStatus::Reached && u.norm() < .005) {
                arrived = true;
                break;
            }
        }
        EXPECT_TRUE(arrived) << "type " << static_cast<int>(type) << " residual "
                             << (robot.position - target.position).norm();
    }
}
