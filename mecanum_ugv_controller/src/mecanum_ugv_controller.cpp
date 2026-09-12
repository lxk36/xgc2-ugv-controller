#include "mecanum_ugv_controller/mecanum_ugv_controller.h"

#include <ros/console.h>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "mecanum_ugv_controller/state_machine/custom1_state.h"
#include "mecanum_ugv_controller/state_machine/health_monitor_state.h"
#include "mecanum_ugv_controller/state_machine/ready_state.h"
#include "mecanum_ugv_controller/state_machine/reset_state.h"
#include "mecanum_ugv_controller/state_machine/self_check_state.h"

namespace mecanum_ugv_controller {
namespace {

namespace sm = ::state_machine;

void requireOk(const sm::Status& status, const char* operation) {
    if (!status.ok()) {
        throw std::runtime_error(std::string(operation) + ": " + status.message);
    }
}

}  // namespace

MecanumUgvController::MecanumUgvController(const UgvState& state) : state_(state) {
    setupMachine();
}

void MecanumUgvController::update(double now_sec) {
    current_time_sec_ = now_sec;
    maybeAutoStartCustom1();
    if (machine_) {
        (void)machine_->update();
        noteResetAdmissionAfterUpdate();
    }
}

::state_machine::Status MecanumUgvController::postEvent(::state_machine::Event event) {
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
        ROS_ERROR("[MecanumUgvController] %s", last_reset_admission_miss_.c_str());
    }
    return status;
}

void MecanumUgvController::setResetHoldReason(std::string reason) {
    last_reset_hold_reason_ = std::move(reason);
}

void MecanumUgvController::noteResetAdmissionAfterUpdate() {
    if (!pending_reset_requested_) {
        return;
    }
    pending_reset_requested_ = false;
    if (machine_->currentState(region_type::CONTROL) == state_type::Reset) {
        last_reset_admission_miss_.clear();
        return;
    }
    last_reset_admission_miss_ = describeResetAdmissionMiss(pending_reset_source_);
    ROS_ERROR("[MecanumUgvController] %s", last_reset_admission_miss_.c_str());
}

std::string MecanumUgvController::describeResetAdmissionMiss(const std::string& source) const {
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

ControllerConfig MecanumUgvController::config() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

void MecanumUgvController::setConfig(const ControllerConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = config;
}

bool MecanumUgvController::healthReady() const {
    const auto cfg = config();
    return stateFresh(state_, ros::Time(current_time_sec_), cfg.state_timeout) &&
           insideFence(state_, cfg);
}

bool MecanumUgvController::resetTargetReady() const {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    return reset_target_.valid && std::isfinite(reset_target_.x) &&
           std::isfinite(reset_target_.y) && std::isfinite(reset_target_.yaw);
}

void MecanumUgvController::setResetTarget(ResetTarget target) {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    reset_target_ = target;
}

ResetTarget MecanumUgvController::resetTarget() const {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    return reset_target_;
}

bool MecanumUgvController::worldReferenceReady() const {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    return world_reference_.valid && std::isfinite(world_reference_.vx) &&
           std::isfinite(world_reference_.vy);
}

void MecanumUgvController::setWorldReference(WorldVelocityReference reference) {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    world_reference_ = reference;
}

WorldVelocityReference MecanumUgvController::worldReference() const {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    return world_reference_;
}

void MecanumUgvController::setCommand(ControlCommand command) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = command;
}

ControlCommand MecanumUgvController::command() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return command_;
}

void MecanumUgvController::clearCommand() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = ControlCommand{};
}

void MecanumUgvController::setupMachine() {
    auto builder = sm::StateMachine::builder("MecanumUgvControllerStateMachine");
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
    requireOk(result.status, "build Mecanum controller state machine");
    machine_ = std::move(result.value);
    requireOk(machine_->start(), "start Mecanum controller state machine");
}

void MecanumUgvController::maybeAutoStartCustom1() {
    const auto cfg = config();
    if (!cfg.auto_start_tracking || !healthReady() || !worldReferenceReady() || !machine_) {
        return;
    }
    if (machine_->currentState(region_type::CONTROL) != state_type::Ready) {
        return;
    }
    ::state_machine::Event event(event_type::CUSTOM1_REQUESTED,
                                 ::state_machine::EventTimestamp{current_time_sec_});
    event.source = "auto_start_tracking";
    event.category = ::state_machine::EventCategory::kInput;
    (void)machine_->postEvent(std::move(event));
}

}  // namespace mecanum_ugv_controller
