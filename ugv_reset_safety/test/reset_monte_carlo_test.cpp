// Reuse the deterministic closed-loop plant and independent rectangle audit from
// reset_scenarios_test.cpp. This executable is intentionally not registered as
// a normal catkin gtest target: the campaign size is selected by environment and
// may be sharded into thousands of cases by tools/run_reset_monte_carlo.py.
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include "reset_scenarios_test.cpp"

namespace ugv_reset_safety {
namespace {

enum class MonteCarloMode { Scout, Mecanum, Mixed };

int envInt(const char* name, int fallback, int minimum, int maximum) {
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') {
        return fallback;
    }
    const long parsed = std::stol(value);
    if (parsed < minimum || parsed > maximum) {
        throw std::runtime_error(std::string(name) + " outside supported range");
    }
    return static_cast<int>(parsed);
}

double envDouble(const char* name, double fallback, double minimum, double maximum) {
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') {
        return fallback;
    }
    const double parsed = std::stod(value);
    if (!std::isfinite(parsed) || parsed < minimum || parsed > maximum) {
        throw std::runtime_error(std::string(name) + " outside supported range");
    }
    return parsed;
}

MonteCarloMode envMode() {
    const char* value = std::getenv("XGC_RESET_MC_MODE");
    const std::string mode = value == nullptr ? "mixed" : value;
    if (mode == "scout") {
        return MonteCarloMode::Scout;
    }
    if (mode == "mecanum") {
        return MonteCarloMode::Mecanum;
    }
    if (mode == "mixed") {
        return MonteCarloMode::Mixed;
    }
    throw std::runtime_error("XGC_RESET_MC_MODE must be scout, mecanum, or mixed");
}

const char* modeName(MonteCarloMode mode) {
    switch (mode) {
        case MonteCarloMode::Scout:
            return "scout";
        case MonteCarloMode::Mecanum:
            return "mecanum";
        case MonteCarloMode::Mixed:
            return "mixed";
    }
    return "unknown";
}

RobotType typeFor(MonteCarloMode mode, int index, std::uint32_t seed) {
    if (mode == MonteCarloMode::Scout) {
        return RobotType::Unicycle;
    }
    if (mode == MonteCarloMode::Mecanum) {
        return RobotType::Mecanum;
    }
    // Alternate in mixed mode so every multi-robot case actually exercises both
    // kinematic families instead of depending on a lucky Bernoulli draw.
    return ((index + static_cast<int>(seed & 1U)) % 2) == 0 ? RobotType::Unicycle
                                                            : RobotType::Mecanum;
}

ConvexObstacle randomObstacle(std::mt19937& rng, int index,
                              const std::vector<Eigen::Vector2d>& existing_centers) {
    std::uniform_real_distribution<double> coordinate(-1.65, 1.65);
    std::uniform_real_distribution<double> angle(-kPi, kPi);
    std::uniform_real_distribution<double> radius(0.16, 0.36);
    std::uniform_int_distribution<int> vertex_count(3, 6);

    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    bool separated = false;
    for (int attempt = 0; attempt < 200 && !separated; ++attempt) {
        center = {coordinate(rng), coordinate(rng)};
        separated = true;
        for (const auto& other : existing_centers) {
            if ((center - other).norm() < 0.82) {
                separated = false;
                break;
            }
        }
    }
    if (!separated) {
        center = {coordinate(rng), coordinate(rng)};
    }

    const double rotation = angle(rng);
    const double rx = radius(rng);
    const double ry = radius(rng);
    const int vertices = vertex_count(rng);
    ConvexObstacle result;
    result.id = "mc_obstacle_" + std::to_string(index);
    result.origin = center;
    for (int vertex = 0; vertex < vertices; ++vertex) {
        const double parameter =
            2.0 * kPi * static_cast<double>(vertex) / static_cast<double>(vertices);
        const double x = rx * std::cos(parameter);
        const double y = ry * std::sin(parameter);
        result.vertices.emplace_back(center.x() + std::cos(rotation) * x - std::sin(rotation) * y,
                                     center.y() + std::sin(rotation) * x + std::cos(rotation) * y);
    }
    return result;
}

struct GeneratedCase {
    std::vector<Robot> robots;
    std::vector<ResetTarget> goals;
    std::vector<ConvexObstacle> obstacles;
    Plant plant;
};

GeneratedCase generateCase(std::uint32_t seed, MonteCarloMode mode, int robot_min, int robot_max,
                           int obstacle_min, int obstacle_max) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> robot_count_dist(robot_min, robot_max);
    std::uniform_int_distribution<int> obstacle_count_dist(obstacle_min, obstacle_max);
    std::uniform_real_distribution<double> heading(-kPi, kPi);
    std::uniform_real_distribution<double> phase(-kPi, kPi);
    std::uniform_real_distribution<double> radial_jitter(-0.12, 0.12);
    std::uniform_real_distribution<double> lateral_offset(0.0, 0.25);
    std::uniform_real_distribution<double> linear_lag(0.0, 0.18);
    std::uniform_real_distribution<double> angular_lag(0.0, 0.22);

    GeneratedCase generated;
    const int robot_count = robot_count_dist(rng);
    const double base_phase = phase(rng);
    std::vector<Eigen::Vector2d> slots;
    slots.reserve(robot_count);
    for (int i = 0; i < robot_count; ++i) {
        const double theta =
            base_phase + 2.0 * kPi * static_cast<double>(i) / static_cast<double>(robot_count);
        const double radius = 3.25 + radial_jitter(rng);
        slots.emplace_back(radius * std::cos(theta), radius * std::sin(theta));
    }

    const int shift = robot_count == 1 ? 0 : 1 + static_cast<int>(rng() % (robot_count - 1));
    for (int i = 0; i < robot_count; ++i) {
        const RobotType type = typeFor(mode, i, seed);
        auto robot = makeRobot("mc_robot_" + std::to_string(i), type, slots[i].x(), slots[i].y(),
                               heading(rng));
        if (type == RobotType::Unicycle) {
            robot.half_length = 0.31;
            robot.half_width = 0.26;
            robot.limits.max_vy = 0.0;
        } else {
            robot.half_length = 0.25;
            robot.half_width = 0.20;
            robot.limits.max_vy = 0.35;
        }
        robot.limits.max_vx = 0.35;
        robot.limits.max_omega = 0.5;
        robot.limits.accel_vx = robot.limits.accel_vy = 0.35;
        robot.limits.accel_omega = 0.6;
        generated.robots.push_back(robot);

        const Eigen::Vector2d goal_position =
            robot_count == 1 ? -slots[i] : slots[(i + shift) % robot_count];
        generated.goals.push_back({goal_position, heading(rng)});
    }

    const int obstacle_count = obstacle_count_dist(rng);
    std::vector<Eigen::Vector2d> centers;
    centers.reserve(obstacle_count);
    for (int i = 0; i < obstacle_count; ++i) {
        auto body = randomObstacle(rng, i, centers);
        centers.push_back(body.origin);
        generated.obstacles.push_back(std::move(body));
    }
    generated.plant = {lateral_offset(rng), linear_lag(rng), angular_lag(rng)};
    return generated;
}

const char* outcomeName(const ScenarioResult& result) {
    if (result.completed) {
        return "completed";
    }
    if (result.collision) {
        return "collision";
    }
    if (result.initial_route_rejected ||
        result.failure.find("guidance rejected") != std::string::npos ||
        result.failure.find("schedule admission rejected") != std::string::npos) {
        return "no_route";
    }
    if (result.failure.find("filter status") != std::string::npos) {
        return "filter_failure";
    }
    if (result.failure.find("arrival timeout") != std::string::npos) {
        return "timeout";
    }
    return "contract_failure";
}

TEST(ResetMonteCarlo, Campaign) {
    const int cases = envInt("XGC_RESET_MC_CASES", 8, 1, 1000000);
    const int robot_min = envInt("XGC_RESET_MC_ROBOTS_MIN", 1, 1, 8);
    const int robot_max = envInt("XGC_RESET_MC_ROBOTS_MAX", 4, robot_min, 8);
    const int obstacle_min = envInt("XGC_RESET_MC_OBSTACLES_MIN", 0, 0, 8);
    const int obstacle_max = envInt("XGC_RESET_MC_OBSTACLES_MAX", 4, obstacle_min, 8);
    const int base_seed = envInt("XGC_RESET_MC_SEED", 20260912, 0, 2147483647);
    const double max_time = envDouble("XGC_RESET_MC_MAX_TIME", 240.0, 1.0, 1200.0);
    const MonteCarloMode mode = envMode();

    int completed = 0, no_route = 0, timeout = 0, filter_failure = 0, collision = 0,
        contract_failure = 0;
    double minimum_clearance = std::numeric_limits<double>::infinity();
    double maximum_elapsed = 0.0;

    for (int case_index = 0; case_index < cases; ++case_index) {
        const std::uint32_t seed =
            static_cast<std::uint32_t>(base_seed) + static_cast<std::uint32_t>(case_index);
        auto generated = generateCase(seed, mode, robot_min, robot_max, obstacle_min, obstacle_max);
        const int robots = static_cast<int>(generated.robots.size());
        const int obstacles = static_cast<int>(generated.obstacles.size());
        const auto result = runScenario(generated.robots, generated.goals, generated.obstacles,
                                        generated.plant, max_time, true);
        const std::string outcome = outcomeName(result);
        minimum_clearance = std::min(minimum_clearance, result.min_clearance);
        maximum_elapsed = std::max(maximum_elapsed, result.elapsed);

        if (outcome == "completed") {
            ++completed;
        } else if (outcome == "no_route") {
            ++no_route;
        } else if (outcome == "timeout") {
            ++timeout;
        } else if (outcome == "filter_failure") {
            ++filter_failure;
        } else if (outcome == "collision") {
            ++collision;
        } else {
            ++contract_failure;
        }

        if (outcome != "completed") {
            std::cout << "XGC_RESET_MC_CASE {\"seed\":" << seed << ",\"mode\":\"" << modeName(mode)
                      << "\",\"robots\":" << robots << ",\"obstacles\":" << obstacles
                      << ",\"outcome\":\"" << outcome << "\",\"elapsed\":" << result.elapsed
                      << ",\"min_clearance\":" << result.min_clearance
                      << ",\"plant_lateral_offset\":" << generated.plant.lateral_offset
                      << ",\"plant_linear_lag\":" << generated.plant.linear_lag
                      << ",\"plant_angular_lag\":" << generated.plant.angular_lag << "}"
                      << std::endl;
        }
    }

    const double completion_rate = static_cast<double>(completed) / static_cast<double>(cases);
    std::cout << "XGC_RESET_MC_SUMMARY {\"seed\":" << base_seed << ",\"mode\":\"" << modeName(mode)
              << "\",\"cases\":" << cases << ",\"completed\":" << completed
              << ",\"no_route\":" << no_route << ",\"timeout\":" << timeout
              << ",\"filter_failure\":" << filter_failure << ",\"collision\":" << collision
              << ",\"contract_failure\":" << contract_failure
              << ",\"completion_rate\":" << completion_rate
              << ",\"min_clearance\":" << minimum_clearance
              << ",\"max_elapsed\":" << maximum_elapsed << "}" << std::endl;

    RecordProperty("mc_cases", cases);
    RecordProperty("mc_completed", completed);
    RecordProperty("mc_no_route", no_route);
    RecordProperty("mc_timeout", timeout);
    RecordProperty("mc_filter_failure", filter_failure);
    RecordProperty("mc_collision", collision);
    RecordProperty("mc_contract_failure", contract_failure);
    EXPECT_EQ(collision, 0) << "Monte Carlo found a physical rectangle collision";
    EXPECT_EQ(contract_failure, 0) << "Monte Carlo found a controller/plant contract violation";
}

}  // namespace
}  // namespace ugv_reset_safety
