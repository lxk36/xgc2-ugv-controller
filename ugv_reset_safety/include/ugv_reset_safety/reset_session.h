#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <random>

namespace ugv_reset_safety {

inline double monotonicSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// No ROS callbacks or alternate control law: this is the request/response lease
// used by the existing native command owner. Clocks are explicit for regression.
class ResetSession {
   public:
    struct Pose {
        double x{0}, y{0}, yaw{0};
    };
    struct Command {
        double x{0}, y{0}, yaw{0};
    };
    enum Status : uint8_t { RUNNING = 0, ARRIVED = 1, REJECTED = 2 };
    struct Feedback {
        bool valid{false};
        Status status{RUNNING};
        Command command;
    };
    struct Request {
        bool valid{false};
        uint32_t generation{0};
        uint64_t stamp{0};
        Pose pose, target;
        Command applied_command;
        uint64_t applied_stamp{0};
    };

    void begin(Pose target) {
        cancel();
        // Refuse wraparound, which could admit a message from an older session.
        if (generation_ == std::numeric_limits<uint32_t>::max() || !finite(target)) {
            return;
        }
        ++generation_;
        target_ = target;
        active_ = true;
    }
    void cancel() {
        active_ = false;
        feedback_ = {};
        issued_.clear();
        last_issued_ = 0;
        last_accepted_ = 0;
    }
    bool active() const {
        return active_;
    }
    uint32_t generation() const {
        return generation_;
    }
    Pose target() const {
        return target_;
    }

    // Called by the sole native publisher, after publish(), including zero.
    // Receiving a safe proposal must never advance the executed slew state.
    void noteApplied(Command command, uint64_t stamp) {
        if (!finite(command) || stamp == 0) {
            return;
        }
        applied_command_ = command;
        applied_stamp_ = stamp;
    }

    Request issue(Pose pose, uint64_t stamp, double wall) {
        if (!active_ || !finite(pose) || stamp == 0 || !std::isfinite(wall)) {
            return {};
        }
        // A rewound or paused ROS clock must not renew a moving command lease.
        if (stamp <= last_issued_) {
            return {};
        }
        if (!issued_.empty() && wall - issued_.back().wall < 0.02) {
            return {};
        }
        while (!issued_.empty() && wall - issued_.front().wall > kLeaseSeconds) {
            issued_.pop_front();
        }
        last_issued_ = stamp;
        issued_.push_back({stamp, wall});
        return {true, generation_, stamp, pose, target_, applied_command_, applied_stamp_};
    }

    bool accept(uint32_t generation, uint64_t stamp, uint8_t status, Command command, uint64_t now,
                double wall) {
        if (!active_ || generation != generation_ || stamp <= last_accepted_ || status > REJECTED ||
            !finite(command) || !freshRos(stamp, now)) {
            return false;
        }
        for (const auto& issued : issued_) {
            if (issued.stamp != stamp) {
                continue;
            }
            if (!freshWall(issued.wall, wall)) {
                return false;
            }
            if (status != RUNNING && (command.x != 0 || command.y != 0 || command.yaw != 0)) {
                return false;
            }
            last_accepted_ = stamp;
            response_wall_ = issued.wall;  // issue time, never delayed receive time
            feedback_ = {true, static_cast<Status>(status), command};
            return true;
        }
        return false;
    }

    Feedback feedback(uint64_t now, double wall) const {
        if (!active_ || !feedback_.valid || !freshRos(last_accepted_, now) ||
            !freshWall(response_wall_, wall)) {
            return {};
        }
        return feedback_;
    }

   private:
    struct Issued {
        uint64_t stamp;
        double wall;
    };
    static constexpr double kLeaseSeconds = 0.15;
    static bool finite(Pose value) {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.yaw);
    }
    static bool finite(Command value) {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.yaw);
    }
    static bool freshRos(uint64_t stamp, uint64_t now) {
        return stamp > 0 && now >= stamp && now - stamp <= 150000000ULL;
    }
    static bool freshWall(double sent, double now) {
        return std::isfinite(now) && now >= sent && now - sent <= kLeaseSeconds;
    }
    bool active_{false};
    // Do not reuse deterministic session 1 after a native owner restart while
    // /clock is paused. The request timestamp ledger supplies the second key.
    uint32_t generation_{std::random_device{}() & 0x7fffffffU};
    uint64_t last_issued_{0}, last_accepted_{0};
    double response_wall_{0};
    Pose target_;
    Command applied_command_;
    uint64_t applied_stamp_{0};
    Feedback feedback_;
    std::deque<Issued> issued_;
};

}  // namespace ugv_reset_safety
