#include <gtest/gtest.h>
#include <ugv_reset_safety/fleet_schedule.h>
#include <ugv_reset_safety/reset_dwa.h>
#include <ugv_reset_safety/reset_path.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace ugv_reset_safety {
namespace {

constexpr double kPi = 3.14159265358979323846;
using Polygon = std::vector<Eigen::Vector2d>;

double wrap(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

Polygon bodyPolygon(const Robot& robot) {
    Polygon result;
    Eigen::Matrix2d rotation;
    rotation << std::cos(robot.yaw), -std::sin(robot.yaw), std::sin(robot.yaw), std::cos(robot.yaw);
    for (const auto& corner : Polygon{{-robot.half_length, -robot.half_width},
                                      {robot.half_length, -robot.half_width},
                                      {robot.half_length, robot.half_width},
                                      {-robot.half_length, robot.half_width}}) {
        result.push_back(robot.position + rotation * (corner + robot.body_center_offset));
    }
    return result;
}

double pointSegmentDistance(const Eigen::Vector2d& point, const Eigen::Vector2d& a,
                            const Eigen::Vector2d& b) {
    const Eigen::Vector2d edge = b - a;
    const double fraction = std::clamp((point - a).dot(edge) / edge.squaredNorm(), 0.0, 1.0);
    return (point - a - fraction * edge).norm();
}

// Independent full-rectangle collision audit: SAT plus segment distances,
// rather than reusing the DWA's footprint check.
double polygonClearance(const Polygon& first, const Polygon& second) {
    bool separated = false;
    double penetration = std::numeric_limits<double>::infinity();
    for (const auto* polygon : {&first, &second}) {
        for (std::size_t edge = 0; edge < polygon->size(); ++edge) {
            const Eigen::Vector2d tangent =
                (*polygon)[(edge + 1) % polygon->size()] - (*polygon)[edge];
            const Eigen::Vector2d normal = Eigen::Vector2d(-tangent.y(), tangent.x()).normalized();
            double amin = std::numeric_limits<double>::infinity(), amax = -amin;
            double bmin = amin, bmax = -amin;
            for (const auto& point : first) {
                amin = std::min(amin, normal.dot(point));
                amax = std::max(amax, normal.dot(point));
            }
            for (const auto& point : second) {
                bmin = std::min(bmin, normal.dot(point));
                bmax = std::max(bmax, normal.dot(point));
            }
            const double overlap = std::min(amax, bmax) - std::max(amin, bmin);
            if (overlap < 0.0) {
                separated = true;
            }
            penetration = std::min(penetration, overlap);
        }
    }
    if (!separated) {
        return -penetration;
    }
    double gap = std::numeric_limits<double>::infinity();
    for (const auto& point : first) {
        for (std::size_t edge = 0; edge < second.size(); ++edge) {
            gap = std::min(
                gap, pointSegmentDistance(point, second[edge], second[(edge + 1) % second.size()]));
        }
    }
    for (const auto& point : second) {
        for (std::size_t edge = 0; edge < first.size(); ++edge) {
            gap = std::min(
                gap, pointSegmentDistance(point, first[edge], first[(edge + 1) % first.size()]));
        }
    }
    return gap;
}

struct Plant {
    double lateral_offset{0.0};
    double linear_lag{0.0};
    double angular_lag{0.0};
};

struct ScenarioResult {
    bool completed{false};
    bool collision{false};
    bool initial_route_rejected{false};
    std::string failure;
    double elapsed{0.0};
    double max_position_error{0.0};
    double max_yaw_error{0.0};
    double min_clearance{std::numeric_limits<double>::infinity()};
    int infeasible_steps{0};
    std::string describe() const {
        std::ostringstream out;
        out << "completed=" << completed << "; " << failure << "; elapsed=" << elapsed
            << "; position=" << max_position_error << "; yaw=" << max_yaw_error
            << "; physical clearance=" << min_clearance
            << "; infeasible steps=" << infeasible_steps;
        return out.str();
    }
};

Robot makeRobot(const std::string& id, RobotType type, double x, double y, double yaw) {
    Robot robot;
    robot.id = id;
    robot.type = type;
    robot.position = Eigen::Vector2d(x, y);
    robot.yaw = yaw;
    robot.half_length = 0.25;
    robot.half_width = 0.2;
    robot.limits.max_vx = robot.limits.max_vy = 0.35;
    robot.limits.max_omega = 0.7;
    robot.limits.accel_vx = robot.limits.accel_vy = 0.5;
    robot.limits.accel_omega = 0.8;
    return robot;
}

ConvexObstacle obstacle(const std::string& id, double x, double y, double width, double height) {
    return {id,
            {{x - width / 2.0, y - height / 2.0},
             {x + width / 2.0, y - height / 2.0},
             {x + width / 2.0, y + height / 2.0},
             {x - width / 2.0, y + height / 2.0}}};
}

ScenarioResult runScenario(std::vector<Robot> robots, const std::vector<ResetTarget>& goals,
                           const std::vector<ConvexObstacle>& obstacles, const Plant& plant,
                           double duration = 100.0, bool production_profile = false,
                           const std::string& trace_file = {}) {
    ScenarioResult result;
    Fence fence;
    fence.xmin = fence.ymin = -5.0;
    fence.xmax = fence.ymax = 5.0;
    DwaConfig config;
    config.dt = 0.02;
    config.clearance = 0.08;
    config.uncertainty_margin = 0.03;
    for (auto& robot : robots) {
        // Keep one configured bound across trials; do not give the controller
        // the exact synthetic plant coefficient in each scenario.
        if (robot.type == RobotType::Unicycle) {
            robot.lateral_velocity_per_yaw_bound = 0.25;
        }
    }
    if (production_profile) {
        config.uncertainty_margin = 0.05;
    }
    std::vector<ResetPath> paths(robots.size());
    ResetDwa dwa;
    FleetSchedule schedule(config.clearance + config.uncertainty_margin);
    std::vector<bool> completed(robots.size(), false), selected(robots.size(), false);
    std::vector<ConvexObstacle> path_obstacles = obstacles;
    std::ofstream trace;
    if (!trace_file.empty()) {
        trace.open(trace_file);
        trace << std::setprecision(12)
              << "time,robot,x,y,yaw,actual_vx,actual_vy,actual_omega,"
                 "command_vx,command_vy,command_omega,path_remaining\n";
    }
    std::vector<Eigen::Vector3d> actual(robots.size(), Eigen::Vector3d::Zero());
    double stationary_time = 0.0;
    auto finish = [&]() {
        for (std::size_t i = 0; i < robots.size(); ++i) {
            result.max_position_error = std::max(result.max_position_error,
                                                 (robots[i].position - goals[i].position).norm());
            result.max_yaw_error =
                std::max(result.max_yaw_error, std::abs(wrap(robots[i].yaw - goals[i].yaw)));
            if (!result.completed && !result.initial_route_rejected) {
                std::ostringstream detail;
                detail << "; robot " << i << " pose " << robots[i].position.transpose() << " yaw "
                       << robots[i].yaw << " command " << robots[i].previous.transpose()
                       << " path size " << paths[i].path().size();
                result.failure += detail.str();
            }
        }
        static int recorded_scenarios = 0;
        ::testing::Test::RecordProperty("scenario_" + std::to_string(++recorded_scenarios),
                                        result.describe());
        return result;
    };
    const auto admission = schedule.initialize(robots, goals);
    if (!admission.ok()) {
        result.failure = "schedule admission rejected: " + admission.detail;
        return finish();
    }
    for (int step = 0; step < static_cast<int>(duration / config.dt); ++step) {
        result.elapsed = step * config.dt;
        for (std::size_t i = 0; i < robots.size(); ++i) {
            const double vy = robots[i].type == RobotType::Unicycle
                                  ? -plant.lateral_offset * actual[i].z()
                                  : actual[i].y();
            if (completed[i] &&
                (!withinTargetTolerance(robots[i], goals[i]) ||
                 std::hypot(actual[i].x(), vy) > 0.03 || std::abs(actual[i].z()) > 0.05)) {
                result.failure = "completed robot moved after certified stop";
                return finish();
            }
        }
        const auto next = schedule.select(completed);
        if (!next.ok()) {
            result.failure = "schedule failed: " + next.detail;
            return finish();
        }
        if (next.status == ScheduleStatus::Complete) {
            result.completed = true;
            return finish();
        }
        std::vector<bool> next_selected(robots.size(), false);
        for (const auto index : next.selected) {
            next_selected[index] = true;
        }
        if (next_selected != selected) {
            selected = next_selected;
            dwa.clear();
            path_obstacles = obstacles;
            const auto parked = parkedPeerObstacles(robots, selected);
            path_obstacles.insert(path_obstacles.end(), parked.begin(), parked.end());
            for (std::size_t i = 0; i < robots.size(); ++i) {
                if (!selected[i]) {
                    continue;
                }
                const auto status =
                    paths[i].setGoal(robots[i], goals[i], path_obstacles, fence).status;
                if (status == PathStatus::InvalidInput || status == PathStatus::NoRoute) {
                    result.initial_route_rejected = step == 0 && status == PathStatus::NoRoute;
                    result.failure = "route rejected robot " + std::to_string(i);
                    return finish();
                }
            }
        }
        for (std::size_t i = 0; i < robots.size(); ++i) {
            robots[i].active = selected[i];
            robots[i].stop_requested = false;
            robots[i].command.setZero();
            if (selected[i]) {
                const auto path_result = paths[i].step(robots[i]);
                const double vy = robots[i].type == RobotType::Unicycle
                                      ? -plant.lateral_offset * actual[i].z()
                                      : actual[i].y();
                const bool at_xy = withinTargetTolerance(robots[i], goals[i]);
                robots[i].stop_requested =
                    (path_result.status == PathStatus::Reached || at_xy) &&
                    robots[i].previous.cwiseAbs().maxCoeff() <= config.feasibility_tolerance &&
                    std::hypot(actual[i].x(), vy) <= 0.03 && std::abs(actual[i].z()) <= 0.05;
            } else {
                // A group switch only parks a robot after an applied zero
                // command and measured stop; residual plant lag still evolves.
                robots[i].previous.setZero();
            }
        }
        dwa.apply(robots, paths, path_obstacles, fence, config);
        std::vector<Eigen::Vector3d> commands(robots.size(), Eigen::Vector3d::Zero());
        for (std::size_t i = 0; i < robots.size(); ++i) {
            commands[i] = robots[i].active ? robots[i].command : Eigen::Vector3d::Zero();
            if (robots[i].active && !robots[i].local_plan_feasible) {
                ++result.infeasible_steps;
            }
        }
        bool stationary = true;
        for (std::size_t i = 0; i < robots.size(); ++i) {
            stationary = stationary && commands[i].norm() < 1.0e-6 && actual[i].norm() < 1.0e-6;
        }
        stationary_time = stationary ? stationary_time + config.dt : 0.0;
        if (stationary_time > 10.0) {
            result.failure = "fleet stationary without arrival for 10 s";
            return finish();
        }
        if (trace.is_open()) {
            for (std::size_t i = 0; i < robots.size(); ++i) {
                trace << result.elapsed << ',' << robots[i].id << ',' << robots[i].position.x()
                      << ',' << robots[i].position.y() << ',' << robots[i].yaw;
                for (int axis = 0; axis < 3; ++axis) {
                    trace << ',' << actual[i][axis];
                }
                for (int axis = 0; axis < 3; ++axis) {
                    trace << ',' << commands[i][axis];
                }
                trace << ',' << paths[i].project(robots[i].position).remaining << ',' << 0.0
                      << '\n';
            }
        }
        for (std::size_t i = 0; i < robots.size(); ++i) {
            const Eigen::Vector3d accelerations(
                robots[i].limits.accel_vx, robots[i].limits.accel_vy, robots[i].limits.accel_omega);
            // Fail-closed DWA may command exact zero outside the dynamic
            // window. Certified samples must still respect acceleration.
            if (robots[i].active && robots[i].local_plan_feasible &&
                ((commands[i] - robots[i].previous).cwiseAbs() - config.dt * accelerations)
                        .maxCoeff() > 2.0e-6) {
                result.failure = "command slew exceeded";
                return finish();
            }
            robots[i].previous = commands[i];
            if (robots[i].stop_requested && commands[i] == Eigen::Vector3d::Zero()) {
                completed[i] = true;
            }
        }
        // Ten plant steps per command audit intersample footprint clearance.
        const double dt = config.dt / 10.0;
        for (int substep = 0; substep < 10; ++substep) {
            for (std::size_t i = 0; i < robots.size(); ++i) {
                for (int axis = 0; axis < 3; ++axis) {
                    const double lag = axis == 2 ? plant.angular_lag : plant.linear_lag;
                    const double fraction = lag > 0.0 ? 1.0 - std::exp(-dt / lag) : 1.0;
                    actual[i][axis] += fraction * (commands[i][axis] - actual[i][axis]);
                }
                double vy = actual[i].y();
                if (robots[i].type == RobotType::Unicycle) {
                    vy = -plant.lateral_offset * actual[i].z();
                }
                const double c = std::cos(robots[i].yaw), s = std::sin(robots[i].yaw);
                robots[i].position +=
                    dt * Eigen::Vector2d(c * actual[i].x() - s * vy, s * actual[i].x() + c * vy);
                robots[i].yaw = wrap(robots[i].yaw + dt * actual[i].z());
            }
            std::vector<Polygon> footprints;
            footprints.reserve(robots.size());
            for (const auto& robot : robots) {
                footprints.push_back(bodyPolygon(robot));
            }
            for (std::size_t i = 0; i < robots.size(); ++i) {
                for (const auto& body : obstacles) {
                    result.min_clearance = std::min(result.min_clearance,
                                                    polygonClearance(footprints[i], body.vertices));
                }
                for (std::size_t j = i + 1; j < robots.size(); ++j) {
                    result.min_clearance = std::min(result.min_clearance,
                                                    polygonClearance(footprints[i], footprints[j]));
                }
                for (const auto& corner : footprints[i]) {
                    result.min_clearance = std::min(
                        {result.min_clearance, corner.x() - fence.xmin, fence.xmax - corner.x(),
                         corner.y() - fence.ymin, fence.ymax - corner.y()});
                }
            }
            if (result.min_clearance < -1.0e-5) {
                result.collision = true;
                result.failure = "physical footprint collision between control updates";
                return finish();
            }
        }
    }
    result.failure = "arrival timeout";
    return finish();
}

TEST(ResetScenarios, DiverseInitialPosesOnIdealPlants) {
    std::mt19937 rng(9102026);
    std::uniform_real_distribution<double> heading(-kPi, kPi);
    for (RobotType type : {RobotType::Unicycle, RobotType::Mecanum}) {
        for (int sample = 0; sample < 10; ++sample) {
            const double angle = heading(rng);
            auto robot = makeRobot("robot", type, 2.4 * std::cos(angle), 2.4 * std::sin(angle),
                                   heading(rng));
            const ResetTarget target{Eigen::Vector2d::Zero(), heading(rng)};
            const auto result = runScenario({robot}, {target}, {}, {});
            EXPECT_TRUE(result.completed) << "type=" << static_cast<int>(type)
                                          << " sample=" << sample << ": " << result.describe();
            EXPECT_FALSE(result.collision);
            // The current admission/completion contract is XY <= 5 cm. Yaw
            // is reported, but is not silently promoted to an arrival gate.
            if (result.completed) {
                EXPECT_LE(result.max_position_error, 0.05);
            }
        }
    }
}

TEST(ResetScenarios, ObstacleFieldWithVariedInitialAndTargetHeadings) {
    const std::vector<ConvexObstacle> field{obstacle("left", -0.65, 0.5, 0.4, 0.7),
                                            obstacle("right", 0.6, -0.6, 0.4, 0.7),
                                            obstacle("top", 0.1, 1.5, 0.6, 0.3)};
    for (RobotType type : {RobotType::Unicycle, RobotType::Mecanum}) {
        for (int sample = 0; sample < 4; ++sample) {
            auto robot = makeRobot("robot", type, -2.6, -1.2 + 0.8 * sample, -2.8 + 1.7 * sample);
            const ResetTarget target{Eigen::Vector2d(2.6, 1.2 - 0.8 * sample), 2.5 - 1.3 * sample};
            const auto result = runScenario({robot}, {target}, field, {});
            EXPECT_TRUE(result.completed) << "type=" << static_cast<int>(type)
                                          << " sample=" << sample << ": " << result.describe();
            EXPECT_FALSE(result.collision);
        }
    }
}

TEST(ResetScenarios, ScoutLateralOffsetAndActuatorLag) {
    const Plant perturbed{0.229, 0.12, 0.16};
    for (int sample = 0; sample < 8; ++sample) {
        const double start_angle = -kPi + sample * kPi / 4.0;
        auto robot = makeRobot("scout", RobotType::Unicycle, 2.2 * std::cos(start_angle),
                               2.2 * std::sin(start_angle), 0.45 + sample * 0.7);
        const ResetTarget target{Eigen::Vector2d::Zero(), -2.7 + sample * 0.75};
        const auto result = runScenario({robot}, {target}, {}, perturbed);
        EXPECT_TRUE(result.completed) << "sample=" << sample << ": " << result.describe();
        EXPECT_FALSE(result.collision);
    }
}

TEST(ResetScenarios, NearGoalLateralOffsetsWithZeroAndNegativeCoupling) {
    const std::vector<Eigen::Vector2d> starts{
        {0.0, 0.08}, {0.0, -0.12}, {0.10, 0.09}, {-0.08, -0.06}};
    for (double offset : {0.0, 0.1, 0.229, 0.25}) {
        for (std::size_t sample = 0; sample < starts.size(); ++sample) {
            auto robot = makeRobot("scout", RobotType::Unicycle, starts[sample].x(),
                                   starts[sample].y(), static_cast<double>(sample) * 0.9);
            const auto result =
                runScenario({robot}, {{Eigen::Vector2d::Zero(), 0.0}}, {}, {offset, 0.12, 0.16});
            EXPECT_TRUE(result.completed)
                << "offset=" << offset << " sample=" << sample << ": " << result.describe();
            EXPECT_FALSE(result.collision);
            if (result.completed) {
                EXPECT_LE(result.max_position_error, 0.05);
            }
        }
    }
}

TEST(ResetScenarios, ObstacleTrackingWithScoutLateralOffsetAndLag) {
    const auto robot = makeRobot("scout", RobotType::Unicycle, -2.4, -0.6, 0.5);
    const ResetTarget target{Eigen::Vector2d(2.4, 0.6), -0.8};
    const std::vector<ConvexObstacle> field{obstacle("middle", 0.0, 0.0, 0.6, 0.8)};
    const auto result = runScenario({robot}, {target}, field, {0.229, 0.12, 0.16});
    EXPECT_TRUE(result.completed) << result.describe();
    EXPECT_FALSE(result.collision);
}

TEST(ResetScenarios, TwoRobotsCrossingHeadOnAndPositionSwap) {
    for (RobotType second_type : {RobotType::Unicycle, RobotType::Mecanum}) {
        for (int encounter = 0; encounter < 3; ++encounter) {
            auto first = makeRobot("first", RobotType::Unicycle, -2.0, 0.0, 0.0);
            auto second = makeRobot("second", second_type, encounter == 0 ? 0.0 : 2.0,
                                    encounter == 0 ? -2.0 : 0.0, encounter == 0 ? kPi / 2.0 : kPi);
            const ResetTarget first_goal{Eigen::Vector2d(2.0, 0.0), 0.0};
            const ResetTarget second_goal{
                encounter == 0 ? Eigen::Vector2d(0.0, 2.0) : Eigen::Vector2d(-2.0, 0.0),
                encounter == 2 ? 0.0 : second.yaw};
            // Both robots start together; the four-Scout test below enforces the 45 s owner
            // timeout.
            const auto result =
                runScenario({first, second}, {first_goal, second_goal}, {}, {}, 600.0);
            EXPECT_TRUE(result.completed)
                << "second_type=" << static_cast<int>(second_type) << " encounter=" << encounter
                << ": " << result.describe();
            EXPECT_FALSE(result.collision);
        }
    }
}

TEST(ResetScenarios, TwoScoutsCrossingWithLateralOffsetAndLag) {
    const auto first = makeRobot("first", RobotType::Unicycle, -2.0, 0.0, 0.0);
    const auto second = makeRobot("second", RobotType::Unicycle, 0.0, -2.0, kPi / 2.0);
    const auto result = runScenario(
        {first, second}, {{Eigen::Vector2d(2.0, 0.0), 0.0}, {Eigen::Vector2d(0.0, 2.0), kPi / 2.0}},
        {}, {0.229, 0.12, 0.16}, 600.0);
    EXPECT_TRUE(result.completed) << result.describe();
    EXPECT_FALSE(result.collision);
}

TEST(ResetScenarios, SeededSparseLayoutsWithProductionScoutProfile) {
    std::mt19937 rng(20260910);
    std::uniform_real_distribution<double> coordinate(-1.8, 1.8);
    std::uniform_real_distribution<double> endpoint(-2.0, 2.0);
    std::uniform_real_distribution<double> angle(-kPi, kPi);
    std::uniform_real_distribution<double> size(0.16, 0.34);
    int completed = 0;
    int rejected_before_motion = 0;
    int eligible_failures = 0;
    for (int sample = 0; sample < 100; ++sample) {
        std::vector<ConvexObstacle> field;
        std::vector<Eigen::Vector2d> centers;
        const int count = 2 + sample % 3;
        for (int index = 0; index < count; ++index) {
            Eigen::Vector2d center;
            bool separated = false;
            for (int attempt = 0; attempt < 100; ++attempt) {
                center = Eigen::Vector2d(coordinate(rng), coordinate(rng));
                separated = true;
                for (const auto& other : centers) {
                    separated = separated && (center - other).norm() > 0.95;
                }
                if (separated) {
                    break;
                }
            }
            ASSERT_TRUE(separated) << "deterministic sparse generator capacity";
            centers.push_back(center);
            const double rotation = angle(rng);
            const double a = size(rng), b = size(rng);
            const int vertices = 3 + (sample + index) % 4;
            ConvexObstacle shape;
            shape.id = "shape_" + std::to_string(index);
            for (int vertex = 0; vertex < vertices; ++vertex) {
                const double parameter = 2.0 * kPi * vertex / vertices;
                const double x = a * std::cos(parameter), y = b * std::sin(parameter);
                shape.vertices.emplace_back(
                    center.x() + std::cos(rotation) * x - std::sin(rotation) * y,
                    center.y() + std::sin(rotation) * x + std::cos(rotation) * y);
            }
            field.push_back(std::move(shape));
        }
        auto robot = makeRobot("scout", RobotType::Unicycle, -3.0, endpoint(rng), angle(rng));
        robot.half_length = 0.31;
        robot.half_width = 0.26;
        robot.limits.max_vx = 0.35;
        robot.limits.max_vy = 0.0;
        robot.limits.max_omega = 0.5;
        robot.limits.accel_vx = robot.limits.accel_vy = 0.35;
        robot.limits.accel_omega = 0.6;
        const ResetTarget target{Eigen::Vector2d(3.0, endpoint(rng)), angle(rng)};
        const auto result = runScenario({robot}, {target}, field, {0.229, 0.12, 0.16}, 180.0, true);
        EXPECT_FALSE(result.collision)
            << "seed=20260910 sample=" << sample << ": " << result.describe();
        if (result.completed) {
            ++completed;
            EXPECT_LE(result.max_position_error, 0.05);
        } else if (result.initial_route_rejected) {
            ++rejected_before_motion;
            EXPECT_DOUBLE_EQ(result.elapsed, 0.0);
        } else {
            ++eligible_failures;
            std::ostringstream fixture;
            fixture << " start " << robot.position.transpose() << " yaw " << robot.yaw << " target "
                    << target.position.transpose() << " yaw " << target.yaw;
            for (const auto& shape : field) {
                fixture << "; " << shape.id;
                for (const auto& point : shape.vertices) {
                    fixture << " [" << point.transpose() << "]";
                }
            }
            ADD_FAILURE() << "seed=20260910 sample=" << sample << ": " << result.describe()
                          << fixture.str();
        }
    }
    RecordProperty("completed", completed);
    RecordProperty("rejected_before_motion", rejected_before_motion);
    RecordProperty("eligible_failures", eligible_failures);
    // Ensure the generator exercises navigation rather than dwa because
    // every start or goal is rejected by conservative footprint admission.
    EXPECT_GE(completed + eligible_failures, 90);
}

TEST(ResetScenarios, ScoutProductionCloseGoalsAndTransportPose) {
    auto production = [](double yaw) {
        auto robot = makeRobot("scout", RobotType::Unicycle, 0.0, 0.0, yaw);
        robot.limits.max_vx = 0.35;
        robot.limits.max_vy = 0.0;
        robot.limits.max_omega = 0.5;
        robot.limits.accel_vx = robot.limits.accel_vy = 0.35;
        robot.limits.accel_omega = 0.6;
        return robot;
    };
    for (double offset : {-0.25, -0.229, 0.0, 0.229, 0.25}) {
        for (double range : {0.051, 0.08, 0.12, 0.30, 2.4}) {
            for (double bearing : {0.0, kPi / 2.0}) {
                for (double yaw : {0.0, 0.9, kPi / 2.0, kPi}) {
                    const ResetTarget target{
                        range * Eigen::Vector2d(std::cos(bearing), std::sin(bearing)), -0.8};
                    const auto result = runScenario({production(yaw)}, {target}, {},
                                                    {offset, 0.12, 0.16}, 45.0, true);
                    EXPECT_TRUE(result.completed)
                        << "offset=" << offset << " range=" << range << " bearing=" << bearing
                        << " yaw=" << yaw << ": " << result.describe();
                    EXPECT_FALSE(result.collision);
                }
            }
        }
    }
    auto transport = production(0.35);
    transport.position = {-0.6, 0.25};
    const auto result =
        runScenario({transport}, {{{0.0, 0.0}, 0.0}}, {}, {0.229, 0.12, 0.16}, 45.0, true);
    EXPECT_TRUE(result.completed) << result.describe();
}

TEST(ResetScenarios, GazeboPositiveLateralMotionDoesNotOrbitTheResetTarget) {
    auto robot = makeRobot("ugv1", RobotType::Unicycle, 0.058, -2.535, 0.761);
    robot.half_length = 0.31;
    robot.half_width = 0.26;
    robot.limits = {0.35, 0.0, 0.5, 0.35, 0.35, 0.6};
    const ResetTarget target{{0.08037089645766905, -2.8945167390828646}, 0.30643537735432513};
    // Live pose differences: vy/omega = +0.22 m. The old nonnegative
    // offset estimator discarded this sign and held a ~0.35 m orbit.
    const auto result = runScenario({robot}, {target}, {}, {-0.22, 0.12, 0.16}, 45.0, true);
    EXPECT_TRUE(result.completed) << result.describe();
    EXPECT_FALSE(result.collision);
    EXPECT_LE(result.max_position_error, 0.05);
}

TEST(ResetScenarios, FourScoutsCrossAndReturnWithProductionProfile) {
    const std::vector<Eigen::Vector2d> corners{{-2.5, -2.0}, {2.5, -2.0}, {2.5, 2.0}, {-2.5, 2.0}};
    for (const Plant plant : {Plant{}, Plant{-0.229, 0.12, 0.16}, Plant{0.229, 0.12, 0.16}}) {
        // Opposite-corner exchange; all four start together under the DWA.
        for (int leg = 0; leg < 2; ++leg) {
            std::vector<Robot> robots;
            std::vector<ResetTarget> goals;
            for (std::size_t i = 0; i < corners.size(); ++i) {
                const auto start = corners[(i + 2 * leg) % corners.size()];
                const auto target = corners[(i + 2 * (1 - leg)) % corners.size()];
                auto robot = makeRobot("scout_" + std::to_string(i), RobotType::Unicycle, start.x(),
                                       start.y(), -1.7 + 1.1 * i);
                robot.half_length = 0.31;
                robot.half_width = 0.26;
                robot.limits.max_vx = 0.35;
                robot.limits.max_vy = 0.0;
                robot.limits.max_omega = 0.5;
                robot.limits.accel_vx = robot.limits.accel_vy = 0.35;
                robot.limits.accel_omega = 0.6;
                robots.push_back(robot);
                goals.push_back({target, 0.4 + 0.7 * i});
            }
            const auto result = runScenario(robots, goals, {}, plant, 45.0, true,
                                            plant.lateral_offset == 0.0 && leg == 0
                                                ? "/tmp/ugv_reset_four_scout_trace.csv"
                                                : "");
            EXPECT_TRUE(result.completed)
                << "ell=" << plant.lateral_offset << " leg=" << leg << ": " << result.describe();
            EXPECT_FALSE(result.collision);
            if (result.completed) {
                EXPECT_LE(result.max_position_error, 0.05);
            }
        }
    }
}

}  // namespace
}  // namespace ugv_reset_safety
