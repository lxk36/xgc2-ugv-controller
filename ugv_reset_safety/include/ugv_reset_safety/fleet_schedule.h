#pragma once

#include <ugv_reset_safety/reset_guidance.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ugv_reset_safety {

enum class ScheduleStatus {
    Uninitialized,
    Ready,
    Complete,
    InvalidInput,
    ConflictingTargets,
    OccupiedTarget,
    UnsupportedCoordination
};

struct ScheduleResult {
    ScheduleStatus status{ScheduleStatus::Uninitialized};
    std::vector<std::size_t> selected;
    std::string detail;
    bool ok() const {
        return status == ScheduleStatus::Ready || status == ScheduleStatus::Complete;
    }
};

// Batch scheduler for goal-occupancy dependencies, not a complete multi-agent
// planner. i -> j means i's goal is occupied by j's initial/current footprint.
// Sink strongly connected components run first; only singleton and two-robot
// components are supported by the tested nominal/pair-passing coordinator.
// Larger cycles and incompatible targets are rejected before any group starts.
//
// The caller must supply a fresh, stopped fleet on initialize(), freeze goals
// for the batch, plan around parkedPeerObstacles(), and hard-hold every
// nonselected robot in the joint safety solve. A completion bit means BOTH
// original target arrival AND measured stop, not merely a zero desired input.
// This graph policy does not prove geometric route existence, safe continuous
// execution, deadlock freedom of arbitrary scenes, or arbitrary fleet counts.
class FleetSchedule {
   public:
    explicit FleetSchedule(double clearance = 0.1) : clearance_(clearance) {}

    void clear() {
        result_ = ScheduleResult();
        groups_.clear();
        completed_.clear();
        roster_size_ = 0;
        group_ = 0;
    }

    ScheduleResult initialize(const std::vector<Robot>& robots,
                              const std::vector<ResetTarget>& targets) {
        clear();
        roster_size_ = robots.size();
        if (robots.size() != targets.size() || !std::isfinite(clearance_) || clearance_ < 0.0) {
            return reject(ScheduleStatus::InvalidInput, "Invalid scheduler roster or clearance");
        }
        std::set<std::string> identifiers;
        std::vector<std::size_t> requesting;
        std::vector<double> radius(robots.size(), 0.0);
        for (std::size_t i = 0; i < robots.size(); ++i) {
            const auto& robot = robots[i];
            if (robot.id.empty() || !identifiers.insert(robot.id).second ||
                !robot.position.allFinite() || !std::isfinite(robot.yaw) ||
                !robot.body_center_offset.allFinite() || !std::isfinite(robot.half_length) ||
                !std::isfinite(robot.half_width) || robot.half_length <= 0.0 ||
                robot.half_width <= 0.0) {
                return reject(ScheduleStatus::InvalidInput,
                              "Invalid scheduler robot geometry or ID");
            }
            radius[i] =
                std::hypot(robot.half_length, robot.half_width) + robot.body_center_offset.norm();
            if (robot.active) {
                if (!targets[i].position.allFinite() || !std::isfinite(targets[i].yaw)) {
                    return reject(ScheduleStatus::InvalidInput, "Invalid requested reset target");
                }
                requesting.push_back(i);
            }
        }
        auto lexical = [&](std::size_t left, std::size_t right) {
            return robots[left].id < robots[right].id;
        };
        std::sort(requesting.begin(), requesting.end(), lexical);
        std::vector<std::vector<std::size_t>> edges(robots.size());
        for (std::size_t position = 0; position < requesting.size(); ++position) {
            const auto i = requesting[position];
            for (std::size_t other = position + 1; other < requesting.size(); ++other) {
                const auto j = requesting[other];
                if ((targets[i].position - targets[j].position).norm() <=
                    radius[i] + radius[j] + clearance_) {
                    return reject(ScheduleStatus::ConflictingTargets,
                                  "Requested target footprint clearances overlap: " + robots[i].id +
                                      ", " + robots[j].id);
                }
            }
            for (std::size_t j = 0; j < robots.size(); ++j) {
                if (i == j || (targets[i].position - robots[j].position).norm() >
                                  radius[i] + radius[j] + clearance_) {
                    continue;
                }
                if (!robots[j].active) {
                    return reject(
                        ScheduleStatus::OccupiedTarget,
                        "Requested target occupied by stationary nonparticipant: " + robots[j].id);
                }
                edges[i].push_back(j);
            }
            std::sort(edges[i].begin(), edges[i].end(), lexical);
        }

        // Tarjan SCC, linear in the goal dependency graph size.
        const std::size_t absent = robots.size();
        std::vector<std::size_t> discovery(robots.size(), absent), low(robots.size(), absent);
        std::vector<std::size_t> component(robots.size(), absent), stack;
        std::vector<bool> on_stack(robots.size(), false);
        std::vector<std::vector<std::size_t>> components;
        std::size_t next_index = 0;
        std::function<void(std::size_t)> visit = [&](std::size_t node) {
            discovery[node] = low[node] = next_index++;
            stack.push_back(node);
            on_stack[node] = true;
            for (const auto adjacent : edges[node]) {
                if (discovery[adjacent] == absent) {
                    visit(adjacent);
                    low[node] = std::min(low[node], low[adjacent]);
                } else if (on_stack[adjacent]) {
                    low[node] = std::min(low[node], discovery[adjacent]);
                }
            }
            if (low[node] != discovery[node]) {
                return;
            }
            std::vector<std::size_t> members;
            while (!stack.empty()) {
                const auto member = stack.back();
                stack.pop_back();
                on_stack[member] = false;
                component[member] = components.size();
                members.push_back(member);
                if (member == node) {
                    break;
                }
            }
            std::sort(members.begin(), members.end(), lexical);
            components.push_back(std::move(members));
        };
        for (const auto robot : requesting) {
            if (discovery[robot] == absent) {
                visit(robot);
            }
        }
        for (const auto& members : components) {
            if (members.size() > 2) {
                return reject(ScheduleStatus::UnsupportedCoordination,
                              "Goal occupancy cycle exceeds the supported two-robot group");
            }
        }

        // Deterministic reverse topological order: an occupant evacuates before
        // the component whose goal it blocks. Ties use the smallest robot ID.
        std::vector<bool> scheduled(components.size(), false);
        while (groups_.size() < components.size()) {
            std::size_t selected = components.size();
            for (std::size_t candidate = 0; candidate < components.size(); ++candidate) {
                if (scheduled[candidate]) {
                    continue;
                }
                bool sink = true;
                for (const auto robot : components[candidate]) {
                    for (const auto dependency : edges[robot]) {
                        if (component[dependency] != candidate &&
                            !scheduled[component[dependency]]) {
                            sink = false;
                        }
                    }
                }
                if (sink &&
                    (selected == components.size() ||
                     lexical(components[candidate].front(), components[selected].front()))) {
                    selected = candidate;
                }
            }
            if (selected == components.size()) {
                return reject(ScheduleStatus::InvalidInput, "Invalid condensed dependency graph");
            }
            scheduled[selected] = true;
            groups_.push_back(components[selected]);
        }
        completed_.assign(robots.size(), false);
        result_.status = groups_.empty() ? ScheduleStatus::Complete : ScheduleStatus::Ready;
        return select(std::vector<bool>(robots.size(), false));
    }

    ScheduleResult select(const std::vector<bool>& arrived_and_stopped) {
        if (!result_.ok()) {
            return result_;
        }
        if (arrived_and_stopped.size() != roster_size_) {
            return reject(ScheduleStatus::InvalidInput, "Completion vector does not match roster");
        }
        for (std::size_t i = 0; i < completed_.size(); ++i) {
            if (completed_[i] && !arrived_and_stopped[i]) {
                return reject(ScheduleStatus::InvalidInput,
                              "Previously completed robot lost its arrived-and-stopped state");
            }
        }
        while (group_ < groups_.size()) {
            bool finished = true;
            for (const auto robot : groups_[group_]) {
                finished = finished && arrived_and_stopped[robot];
            }
            if (!finished) {
                break;
            }
            for (const auto robot : groups_[group_]) {
                completed_[robot] = true;
            }
            ++group_;
        }
        result_.selected.clear();
        result_.status =
            group_ == groups_.size() ? ScheduleStatus::Complete : ScheduleStatus::Ready;
        if (group_ < groups_.size()) {
            for (const auto robot : groups_[group_]) {
                if (!arrived_and_stopped[robot]) {
                    result_.selected.push_back(robot);
                }
            }
        }
        return result_;
    }

   private:
    ScheduleResult reject(ScheduleStatus status, std::string detail) {
        result_.status = status;
        result_.selected.clear();
        result_.detail = std::move(detail);
        return result_;
    }
    double clearance_;
    ScheduleResult result_;
    std::vector<std::vector<std::size_t>> groups_;
    std::vector<bool> completed_;
    std::size_t roster_size_{0};
    std::size_t group_{0};
};

// Exact rectangular footprints of every nonselected peer, frozen only for
// the group's geometric route initialization. The safety QP must still include
// these peers at their fresh actual pose as hard stationary constraints.
inline std::vector<ConvexObstacle> parkedPeerObstacles(const std::vector<Robot>& robots,
                                                       const std::vector<bool>& selected_mask) {
    if (selected_mask.size() != robots.size()) {
        throw std::invalid_argument("Selected mask does not match the scheduler roster");
    }
    std::vector<ConvexObstacle> obstacles;
    for (std::size_t index = 0; index < robots.size(); ++index) {
        if (selected_mask[index]) {
            continue;
        }
        const auto& robot = robots[index];
        const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
        ConvexObstacle obstacle;
        obstacle.id = "held_robot/" + robot.id;
        for (const auto& corner :
             std::vector<Eigen::Vector2d>{{-robot.half_length, -robot.half_width},
                                          {robot.half_length, -robot.half_width},
                                          {robot.half_length, robot.half_width},
                                          {-robot.half_length, robot.half_width}}) {
            const Eigen::Vector2d local = corner + robot.body_center_offset;
            obstacle.vertices.emplace_back(robot.position.x() + c * local.x() - s * local.y(),
                                           robot.position.y() + s * local.x() + c * local.y());
        }
        obstacles.push_back(std::move(obstacle));
    }
    return obstacles;
}

}  // namespace ugv_reset_safety
