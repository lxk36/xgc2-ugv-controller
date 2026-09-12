#include "unicycle_ugv_controller/state_machine/reset_state.h"

#include <ros/console.h>

#include <cmath>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

ResetState::ResetState(UnicycleUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult ResetState::onEnter(::state_machine::StateContext& ctx) {
    controller_.clearCommand();
    command_gate_.reset();
    enter_time_ = controller_.currentTime();
    enter_wall_ = ugv_reset_safety::monotonicSeconds();
    const auto target = controller_.resetTarget();
    if (controller_.resetTargetReady()) {
        controller_.resetSession().begin({target.x, target.y, target.yaw});
        controller_.setResetHoldReason({});
    } else {
        controller_.setResetHoldReason(
            "no target: reset_pose cache and reset_initial_* missing");
        ROS_ERROR("[UnicycleUgvController] Reset entered without a valid goal; "
                  "holding Reset until timeout/Stop or a cached initialPose");
    }
    emitZero(ctx);
    return {};
}

::state_machine::ActionResult ResetState::onTick(::state_machine::StateContext& ctx) {
    const auto cfg = controller_.config();
    const double now = controller_.currentTime();
    const double wall = ugv_reset_safety::monotonicSeconds();
    if (cfg.reset_timeout > 0.0 &&
        (now - enter_time_ >= cfg.reset_timeout || wall - enter_wall_ >= cfg.reset_timeout)) {
        emitZero(ctx);
        postDone(ctx, event_type::RESET_TIMEOUT);
        return {};
    }
    if (!controller_.healthReady()) {
        emitZero(ctx);
        return {};
    }
    if (!controller_.resetSession().active()) {
        if (controller_.resetTargetReady()) {
            const auto target = controller_.resetTarget();
            controller_.resetSession().begin({target.x, target.y, target.yaw});
            controller_.setResetHoldReason({});
        } else {
            emitZero(ctx);
            ROS_ERROR_THROTTLE(
                1.0, "[UnicycleUgvController] Reset holding with no target (topic=/command "
                     "CONTROL=Reset reject=missing-initialPose)");
            return {};
        }
    }
    const auto feedback = controller_.resetSession().feedback(ros::Time(now).toNSec(), wall);
    if (feedback.valid && feedback.status != ugv_reset_safety::ResetSession::RUNNING) {
        emitZero(ctx);
        postDone(ctx, feedback.status == ugv_reset_safety::ResetSession::ARRIVED
                          ? event_type::RESET_ARRIVED
                          : event_type::RESET_REJECTED);
        return {};
    }
    ControlCommand command;
    command.stamp = ros::Time(now);
    command.valid = true;
    if (feedback.valid) {
        // Refuse a command outside chassis limits; do not silently saturate.
        if (std::abs(feedback.command.x) > cfg.chassis_max_linear_speed ||
            feedback.command.y != 0.0 ||
            std::abs(feedback.command.yaw) > cfg.chassis_max_yaw_rate) {
            emitZero(ctx);
            postDone(ctx, event_type::RESET_REJECTED);
            return {};
        }
        command.linear_speed = feedback.command.x;
        command.angular_speed = feedback.command.yaw;
    }
    // Missing/expired coordinator response commands zero, including /clock pause.
    emitCommand(ctx, command);
    return {};
}

::state_machine::ActionResult ResetState::onExit(::state_machine::StateContext& ctx) {
    controller_.resetSession().cancel();
    emitZero(ctx);
    command_gate_.reset();
    return {};
}

void ResetState::emitCommand(::state_machine::StateContext& ctx, const ControlCommand& command) {
    const auto cfg = controller_.config();
    controller_.setCommand(command);
    if (!command_gate_.due(ugv_reset_safety::monotonicSeconds(),
                           1.0 / cfg.command_publish_rate_hz)) {
        return;
    }
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

void ResetState::emitZero(::state_machine::StateContext& ctx) {
    controller_.clearCommand();
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_ZERO_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

void ResetState::postDone(::state_machine::StateContext& ctx, ::state_machine::EventId id) {
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{controller_.currentTime()});
    event.source = "reset_state";
    event.category = ::state_machine::EventCategory::kInternal;
    (void)ctx.postInternalEvent(std::move(event));
}

}  // namespace unicycle_ugv_controller
