#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

#include <ros/console.h>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "unicycle_ugv_controller/common/rigid_to_unicycle.h"
#include "unicycle_ugv_controller/state_machine/custom1_state.h"
#include "unicycle_ugv_controller/state_machine/health_monitor_state.h"
#include "unicycle_ugv_controller/state_machine/ready_state.h"
#include "unicycle_ugv_controller/state_machine/reset_state.h"
#include "unicycle_ugv_controller/state_machine/self_check_state.h"

namespace unicycle_ugv_controller {
namespace {

namespace sm = ::state_machine;

void requireOk(const sm::Status& status, const char* operation) {
    if (!status.ok()) {
        throw std::runtime_error(std::string(operation) + ": " + status.message);
    }
}

}  // namespace

UnicycleUgvController::UnicycleUgvController(const UgvState& state) : state_(state) {
    setupMachine();
}

void UnicycleUgvController::update(double now_sec) {
    current_time_sec_ = now_sec;
    maybeUpdatePoseVelocity();
    maybeAutoStartCustom1();
    if (machine_) {
        (void)machine_->update();
        noteResetAdmissionAfterUpdate();
    }
}

::state_machine::Status UnicycleUgvController::postEvent(::state_machine::Event event) {
    const auto id = event.id;
    const std::string source = event.source;
    if (id == event_type::RESET_REQUESTED) {
        pending_reset_requested_ = true;
        pending_reset_source_ = source.empty() ? "command" : source;
        last_reset_admission_miss_.clear();
    }
    auto status = machine_->postEvent(std::move(event));
    if (id == event_type::RESET_REQUESTED && !status.ok()) {
        pending_reset_requested_ = false;
        last_reset_admission_miss_ = std::string("Failed to post command event: ") +
                                     status.message + " source=" + pending_reset_source_ +
                                     " CONTROL=" + machine_->currentStateName(region_type::CONTROL);
        ROS_ERROR("[UnicycleUgvController] %s", last_reset_admission_miss_.c_str());
    }
    return status;
}

void UnicycleUgvController::setResetHoldReason(std::string reason) {
    last_reset_hold_reason_ = std::move(reason);
}

void UnicycleUgvController::noteResetAdmissionAfterUpdate() {
    if (!pending_reset_requested_) {
        return;
    }
    pending_reset_requested_ = false;
    if (machine_->currentState(region_type::CONTROL) == state_type::Reset) {
        last_reset_admission_miss_.clear();
        return;
    }
    last_reset_admission_miss_ = describeResetAdmissionMiss(pending_reset_source_);
    ROS_ERROR("[UnicycleUgvController] %s", last_reset_admission_miss_.c_str());
}

std::string UnicycleUgvController::describeResetAdmissionMiss(const std::string& source) const {
    const std::string control = machine_->currentStateName(region_type::CONTROL);
    const bool health = healthReady();
    const bool target = resetTargetReady();
    std::ostringstream oss;
    oss << "RESET_REQUESTED unmatched topic=" << source << " CONTROL=" << control
        << " health=" << (health ? "ready" : "unhealthy")
        << " target=" << (target ? "ready" : "missing");
    if (control == "SelfCheck" && health) {
        oss << " reason=health-ready-but-stuck-in-SelfCheck";
    } else if (control == "SelfCheck") {
        oss << " reason=unhealthy-SelfCheck-has-no-Reset-edge";
    } else {
        oss << " reason=no-matching-transition";
    }
    return oss.str();
}

ControllerConfig UnicycleUgvController::config() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

void UnicycleUgvController::setConfig(const ControllerConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = config;
}

bool UnicycleUgvController::healthReady() const {
    const auto cfg = config();
    if (!stateFresh(state_, ros::Time(current_time_sec_), cfg.state_timeout) ||
        !insideFence(state_, cfg)) {
        return false;
    }
    if (cfg.state_source != StateSource::STATE_ESTIMATOR) {
        return finitePose(state_);
    }
    return rigidEstimateHealthy(state_.estimator_state, state_.estimator_flags);
}

bool UnicycleUgvController::referenceReady() const {
    const auto cfg = config();
    if (cfg.tracking_strategy == TrackingStrategy::FLATNESS) {
        return worldPvaReady();
    }
    return reference_cache_.valid();
}

bool UnicycleUgvController::commandReady() const {
    constexpr double kFutureStampTolerance = 0.05;
    const auto cfg = config();
    const auto current_command = command();
    const double age = current_time_sec_ - current_command.stamp.toSec();
    return current_command.valid && std::isfinite(current_command.linear_speed) &&
           std::isfinite(current_command.angular_speed) && std::isfinite(age) &&
           age >= -kFutureStampTolerance && age <= cfg.result_timeout;
}

void UnicycleUgvController::setCommand(ControlCommand command) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = command;
}

ControlCommand UnicycleUgvController::command() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return command_;
}

void UnicycleUgvController::clearCommand() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = ControlCommand{};
}

bool UnicycleUgvController::resetTargetReady() const {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    return reset_target_.valid && std::isfinite(reset_target_.x) &&
           std::isfinite(reset_target_.y) && std::isfinite(reset_target_.yaw);
}

void UnicycleUgvController::setResetTarget(ResetTarget target) {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    reset_target_ = target;
}

ResetTarget UnicycleUgvController::resetTarget() const {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    return reset_target_;
}

bool UnicycleUgvController::worldPvaReady() const {
    std::lock_guard<std::mutex> lock(pva_mutex_);
    return ::unicycle_ugv_controller::worldPvaReady(world_pva_);
}

void UnicycleUgvController::setWorldPva(WorldPvaReference reference) {
    std::lock_guard<std::mutex> lock(pva_mutex_);
    world_pva_ = reference;
}

WorldPvaReference UnicycleUgvController::worldPva() const {
    std::lock_guard<std::mutex> lock(pva_mutex_);
    return world_pva_;
}

WorldPvaReference UnicycleUgvController::liftedWorldPva() const {
    std::lock_guard<std::mutex> lock(pva_mutex_);
    return liftWorldPva(world_pva_, current_time_sec_);
}

void UnicycleUgvController::maybeUpdatePoseVelocity() {
    const auto cfg = config();
    if (!usesPoseVelocityFilter(cfg.state_source)) {
        return;
    }
    if (!state_.received || !finitePose(state_)) {
        return;
    }
    const double stamp = state_.stamp.toSec();
    if (have_pose_stamp_ && stamp == last_pose_stamp_) {
        return;
    }
    (void)updatePoseVelocityEstimator(pose_velocity_, stamp, state_.x, state_.y, state_.yaw, cfg);
    last_pose_stamp_ = stamp;
    have_pose_stamp_ = true;
}

UgvState UnicycleUgvController::controlState() const {
    UgvState snapshot = state_;
    const auto cfg = config();
    if (usesPoseVelocityFilter(cfg.state_source)) {
        snapshot.vx = pose_velocity_.vx;
        snapshot.vy = pose_velocity_.vy;
        snapshot.yaw_rate = pose_velocity_.yaw_rate;
        snapshot.speed = bodySpeedFromWorld(snapshot.yaw, snapshot.vx, snapshot.vy);
        snapshot.velocity_valid = pose_velocity_.velocity_valid;
    }
    return snapshot;
}

bool UnicycleUgvController::velocityValid() const {
    if (usesPoseVelocityFilter(config().state_source)) {
        return pose_velocity_.velocity_valid;
    }
    return state_.velocity_valid;
}

void UnicycleUgvController::setupMachine() {
    auto builder = sm::StateMachine::builder("UnicycleUgvControllerStateMachine");
    builder.region(region_type::HEALTH)
        .name("health")
        .order(0)
        .initial(state_type::HealthMonitor)
        .state(state_type::HealthMonitor)
        .name("HealthMonitor")
        .impl(std::make_unique<HealthMonitorState>(*this))
        .endRegion()
        .region(region_type::CONTROL)
        .name("control")
        .order(10)
        .initial(state_type::SelfCheck)
        .state(state_type::SelfCheck)
        .name("SelfCheck")
        .impl(std::make_unique<SelfCheckState>(*this))
        .state(state_type::Ready)
        .name("Ready")
        .impl(std::make_unique<ReadyState>(*this))
        .state(state_type::Custom1)
        .name("Custom1")
        .impl(std::make_unique<Custom1State>(*this))
        .state(state_type::Reset)
        .name("Reset")
        .impl(std::make_unique<ResetState>(*this))
        .endRegion();

    builder.transition()
        .from(state_type::SelfCheck)
        .to(state_type::Ready)
        .on(event_type::HEALTH_READY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_UNHEALTHY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_UNHEALTHY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Custom1)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_UNHEALTHY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::Ready)
        .on(event_type::STOP_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Custom1)
        .to(state_type::Ready)
        .on(event_type::STOP_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Custom1)
        .on(event_type::CUSTOM1_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Reset)
        .on(event_type::RESET_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Custom1)
        .to(state_type::Reset)
        .on(event_type::RESET_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::Ready)
        .on(event_type::RESET_ARRIVED)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::Ready)
        .on(event_type::RESET_TIMEOUT)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::Ready)
        .on(event_type::RESET_REJECTED)
        .priority(transition_priority::AUTOMATIC);

    auto result = builder.build();
    requireOk(result.status, "build UGV controller state machine");
    machine_ = std::move(result.value);
    requireOk(machine_->start(), "start UGV controller state machine");
}

void UnicycleUgvController::maybeAutoStartCustom1() {
    const auto cfg = config();
    if (!cfg.auto_start_tracking || !healthReady() || !referenceReady() || !machine_) {
        return;
    }
    if (machine_->currentState(region_type::CONTROL) != state_type::Ready) {
        return;
    }
    ::state_machine::Event event(event_type::CUSTOM1_REQUESTED,
                                 ::state_machine::EventTimestamp{current_time_sec_});
    event.source = "auto_start_tracking";
    event.category = ::state_machine::EventCategory::kInput;
    const auto status = machine_->postEvent(std::move(event));
    if (!status.ok()) {
        ROS_WARN("[UnicycleUgvController] Failed to post auto Custom1 event: %s",
                 status.message.c_str());
    }
}

}  // namespace unicycle_ugv_controller
