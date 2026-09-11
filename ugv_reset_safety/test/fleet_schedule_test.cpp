#include <gtest/gtest.h>
#include <ugv_reset_safety/fleet_schedule.h>

#include <cmath>
#include <limits>

namespace ugv_reset_safety {
namespace {

Robot robot(const std::string& id, double x, double y = 0.0) {
    Robot result;
    result.id = id;
    result.position = Eigen::Vector2d(x, y);
    result.half_length = 0.25;
    result.half_width = 0.20;
    return result;
}

ResetTarget target(double x, double y = 0.0) {
    ResetTarget result;
    result.position = Eigen::Vector2d(x, y);
    return result;
}

TEST(FleetSchedule, StartsEveryRequesterTogether) {
    const std::vector<Robot> robots{robot("a", 0), robot("b", 2), robot("c", 4)};
    const std::vector<ResetTarget> targets{target(2), target(4), target(6)};
    FleetSchedule schedule;
    auto result = schedule.initialize(robots, targets);
    ASSERT_TRUE(result.ok()) << result.detail;
    EXPECT_EQ(result.selected, (std::vector<std::size_t>{0, 1, 2}));
    EXPECT_EQ(schedule.select({false, false, false}).selected, result.selected);
    EXPECT_EQ(schedule.select({false, false, true}).selected, (std::vector<std::size_t>{0, 1}));
    EXPECT_EQ(schedule.select({false, true, true}).selected, (std::vector<std::size_t>{0}));
    EXPECT_EQ(schedule.select({true, true, true}).status, ScheduleStatus::Complete);
    EXPECT_TRUE(schedule.select({true, true, true}).selected.empty());
}

TEST(FleetSchedule, OppositeCornerSwapIsOneCohort) {
    const std::vector<Robot> robots{robot("alpha", -2.5, -2), robot("bravo", 2.5, -2),
                                    robot("charlie", 2.5, 2), robot("delta", -2.5, 2)};
    const std::vector<ResetTarget> targets{target(2.5, 2), target(-2.5, 2), target(-2.5, -2),
                                           target(2.5, -2)};
    FleetSchedule schedule;
    auto result = schedule.initialize(robots, targets);
    ASSERT_TRUE(result.ok()) << result.detail;
    EXPECT_EQ(result.selected, (std::vector<std::size_t>{0, 1, 2, 3}));
    EXPECT_EQ(schedule.select({true, false, false, false}).selected,
              (std::vector<std::size_t>{1, 2, 3}));
    EXPECT_EQ(schedule.select({true, true, true, true}).status, ScheduleStatus::Complete);
}

TEST(FleetSchedule, TieOrderDependsOnIdsRatherThanRosterOrder) {
    const std::vector<Robot> robots{robot("z", 0), robot("a", 2), robot("m", 4)};
    const std::vector<ResetTarget> targets{target(0, 3), target(2, 3), target(4, 3)};
    FleetSchedule schedule;
    EXPECT_EQ(schedule.initialize(robots, targets).selected, (std::vector<std::size_t>{1, 2, 0}));
    EXPECT_EQ(schedule.select({false, true, false}).selected, (std::vector<std::size_t>{2, 0}));
}

TEST(FleetSchedule, AdmitsThreeRobotGoalCycleAsOneCohort) {
    const std::vector<Robot> robots{robot("first_free", -4), robot("x", 0), robot("y", 2),
                                    robot("z", 4)};
    const std::vector<ResetTarget> targets{target(-4, 3), target(2), target(4), target(0)};
    FleetSchedule schedule;
    const auto result = schedule.initialize(robots, targets);
    EXPECT_EQ(result.status, ScheduleStatus::Ready);
    EXPECT_EQ(result.selected, (std::vector<std::size_t>{0, 1, 2, 3}));
}

TEST(FleetSchedule, RejectsTargetsThatOverlapOrAreOccupiedByNonparticipants) {
    std::vector<Robot> robots{robot("a", -2), robot("b", 2)};
    FleetSchedule schedule;
    EXPECT_EQ(schedule.initialize(robots, {target(0), target(0.1)}).status,
              ScheduleStatus::ConflictingTargets);
    robots[1].active = false;
    EXPECT_EQ(schedule.initialize(robots, {target(2), target(99)}).status,
              ScheduleStatus::OccupiedTarget);
    const auto valid = schedule.initialize(robots, {target(0), target(99)});
    EXPECT_EQ(valid.status, ScheduleStatus::Ready);
    EXPECT_EQ(valid.selected, std::vector<std::size_t>({0}));
}

TEST(FleetSchedule, RejectsInvalidRosterAndRegressedMeasuredCompletion) {
    FleetSchedule schedule;
    EXPECT_EQ(schedule.select({}).status, ScheduleStatus::Uninitialized);
    const std::vector<Robot> robots{robot("a", 0), robot("b", 2)};
    EXPECT_EQ(schedule.initialize(robots, {target(4)}).status, ScheduleStatus::InvalidInput);
    EXPECT_EQ(schedule.initialize({robot("a", 0), robot("a", 2)}, {target(4), target(6)}).status,
              ScheduleStatus::InvalidInput);
    EXPECT_EQ(schedule.initialize(robots, {target(2), target(4)}).selected,
              (std::vector<std::size_t>{0, 1}));
    EXPECT_EQ(schedule.select({false, true}).selected, (std::vector<std::size_t>{0}));
    EXPECT_EQ(schedule.select({true, true}).status, ScheduleStatus::Complete);
    const auto regressed = schedule.select({false, false});
    EXPECT_EQ(regressed.status, ScheduleStatus::InvalidInput);
    EXPECT_TRUE(regressed.selected.empty());
    auto invalid = robots;
    invalid[0].yaw = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(schedule.initialize(invalid, {target(2), target(4)}).status,
              ScheduleStatus::InvalidInput);
}

TEST(FleetSchedule, ParkedPeerGeometryPreservesRotationAndPoseOffset) {
    std::vector<Robot> robots{robot("moving", -2), robot("parked", 1, 2)};
    robots[1].yaw = 3.14159265358979323846 / 2.0;
    robots[1].half_length = 0.5;
    robots[1].half_width = 0.25;
    robots[1].body_center_offset = Eigen::Vector2d(0.1, -0.2);
    const auto obstacles = parkedPeerObstacles(robots, {true, false});
    ASSERT_EQ(obstacles.size(), 1U);
    EXPECT_EQ(obstacles[0].id, "held_robot/parked");
    ASSERT_EQ(obstacles[0].vertices.size(), 4U);
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto& vertex : obstacles[0].vertices) {
        centroid += vertex / 4.0;
        EXPECT_GE(vertex.x(), 0.95 - 1e-12);
        EXPECT_LE(vertex.x(), 1.45 + 1e-12);
        EXPECT_GE(vertex.y(), 1.6 - 1e-12);
        EXPECT_LE(vertex.y(), 2.6 + 1e-12);
    }
    EXPECT_NEAR(centroid.x(), 1.2, 1e-12);
    EXPECT_NEAR(centroid.y(), 2.1, 1e-12);
    EXPECT_THROW(parkedPeerObstacles(robots, {true}), std::invalid_argument);
}

TEST(FleetSchedule, SelectedRobotRoutesAroundParkedPeerRatherThanThroughIt) {
    std::vector<Robot> robots{robot("moving", -2), robot("parked", 0)};
    robots[1].active = false;
    const std::vector<ResetTarget> targets{target(2), target(0)};
    FleetSchedule schedule;
    const auto selected = schedule.initialize(robots, targets);
    ASSERT_EQ(selected.status, ScheduleStatus::Ready);
    ASSERT_EQ(selected.selected, std::vector<std::size_t>({0}));
    ResetGuidance guidance;
    const auto planned = guidance.setGoal(robots[0], targets[0],
                                          parkedPeerObstacles(robots, {true, false}), Fence());
    ASSERT_EQ(planned.status, GuidanceStatus::Moving) << planned.message;
    EXPECT_GE(guidance.path().size(), 4U);
}

TEST(FleetSchedule, ClearDiscardsTheFrozenDependencyBatch) {
    FleetSchedule schedule;
    EXPECT_EQ(schedule.initialize({robot("a", 0)}, {target(2)}).status, ScheduleStatus::Ready);
    schedule.clear();
    EXPECT_EQ(schedule.select({false}).status, ScheduleStatus::Uninitialized);
    EXPECT_EQ(schedule.initialize({}, {}).status, ScheduleStatus::Complete);
}

}  // namespace
}  // namespace ugv_reset_safety
