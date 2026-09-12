#include "unicycle_ugv_controller/state_machine/custom1_state.h"

#include <cmath>
#include <utility>
#include <variant>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

Custom1State::Custom1State(UnicycleUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult Custom1State::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    controller_.clearCommand();
    solve_gate_.reset();
    command_gate_.reset();
    zero_gate_.reset();
    // Request IDs belong to this State instance, not one Custom1 activation.
    // Reusing them would let an old worker result authorize a new session.
    in_flight_sequence_ = 0U;
    request_in_flight_ = false;
    request_deadline_ = 0.0;
    const UgvState snapshot = controller_.controlState();
    body_speed_ = snapshot.speed;
    last_tick_time_ = controller_.currentTime();
    have_tick_time_ = true;
    return {};
}

::state_machine::ActionResult Custom1State::onEvent(::state_machine::StateContext& ctx,
                                                    const ::state_machine::Event& event) {
    if (controller_.config().tracking_strategy != TrackingStrategy::NMPC) {
        return {};
    }
    const bool succeeded = event.id == event_type::INPUT_NMPC_SOLVE_SUCCEEDED;
    if ((!succeeded && event.id != event_type::INPUT_NMPC_SOLVE_FAILED) || !request_in_flight_ ||
        event.correlation_id != in_flight_sequence_) {
        return {};
    }
    request_in_flight_ = false;
    if (controller_.currentTime() > request_deadline_) {
        return {};
    }

    // Only the FSM thread may commit a command. The worker supplies a value
    // snapshot, never a write to the command shared by Reset and Custom1.
    if (succeeded) {
        const auto stamp_it = event.payload.find("command_stamp");
        const auto speed_it = event.payload.find("linear_speed");
        const auto yaw_rate_it = event.payload.find("angular_speed");
        if (stamp_it != event.payload.end() && speed_it != event.payload.end() &&
            yaw_rate_it != event.payload.end()) {
            const auto* stamp = std::get_if<double>(&stamp_it->second);
            const auto* speed = std::get_if<double>(&speed_it->second);
            const auto* yaw_rate = std::get_if<double>(&yaw_rate_it->second);
            const auto cfg = controller_.config();
            if (stamp && speed && yaw_rate && std::isfinite(*stamp) && *stamp >= 0.0 &&
                std::isfinite(*speed) && std::isfinite(*yaw_rate)) {
                const double age = controller_.currentTime() - *stamp;
                if (age >= -0.05 && age <= cfg.result_timeout) {
                    ControlCommand command;
                    command.stamp = ros::Time(*stamp);
                    command.linear_speed = *speed;
                    command.angular_speed = *yaw_rate;
                    command.valid = true;
                    controller_.setCommand(command);
                    emitCommandIfDue(ctx);
                    return {};
                }
            }
        }
    }
    // A rejected result must not overwrite a still-fresh prior command.
    if (!hasCommand()) {
        emitZero(ctx);
    }
    return {};
}

::state_machine::ActionResult Custom1State::onTick(::state_machine::StateContext& ctx) {
    if (controller_.config().tracking_strategy == TrackingStrategy::FLATNESS) {
        tickFlatness(ctx);
        return {};
    }
    if (request_in_flight_ && controller_.currentTime() > request_deadline_) {
        request_in_flight_ = false;
    }
    if (!controller_.referenceReady()) {
        emitZero(ctx);
        return {};
    }
    requestSolveIfDue(ctx);
    publishNmpcCommandIfDue(ctx);
    return {};
}

void Custom1State::tickFlatness(::state_machine::StateContext& ctx) {
    const double now = controller_.currentTime();
    if (!controller_.worldPvaReady()) {
        emitZero(ctx);
        return;
    }
    const UgvState snapshot = controller_.controlState();
    double dt = 0.0;
    if (have_tick_time_) {
        dt = now - last_tick_time_;
    }
    // A simulation clock may repeat across event-pump iterations. Accumulate
    // elapsed time instead of overwriting a valid command with a zero command.
    if (have_tick_time_ && dt >= 0.0 && dt <= controller_.config().velocity_dt_min) {
        emitCommandIfDue(ctx);
        return;
    }
    last_tick_time_ = now;
    have_tick_time_ = true;
    const WorldPvaReference lifted = controller_.liftedWorldPva();
    const FlatnessCommandOutput output =
        computeFlatnessCommand(snapshot, lifted, body_speed_, dt, controller_.config());
    if (!output.valid) {
        emitZero(ctx);
        return;
    }
    body_speed_ = output.linear_speed;
    ControlCommand command;
    command.stamp = ros::Time(now);
    command.linear_speed = output.linear_speed;
    command.angular_speed = output.angular_speed;
    command.valid = true;
    controller_.setCommand(command);
    emitCommandIfDue(ctx);
}

void Custom1State::requestSolveIfDue(::state_machine::StateContext& ctx) {
    const auto cfg = controller_.config();
    if (request_in_flight_) {
        return;
    }
    if (!solve_gate_.due(controller_.currentTime(), 1.0 / cfg.nmpc_request_rate_hz)) {
        return;
    }
    ++request_sequence_;
    in_flight_sequence_ = request_sequence_;
    request_in_flight_ = true;
    request_deadline_ = controller_.currentTime() + cfg.solve_timeout;

    ::state_machine::Event event(output_event_type::REQUEST_NMPC_SOLVE,
                                 ::state_machine::EventTimestamp{controller_.currentTime()});
    event.source = "custom1_state";
    event.category = ::state_machine::EventCategory::kOutput;
    event.correlation_id = request_sequence_;
    ctx.emitOutput(std::move(event));
}

void Custom1State::publishNmpcCommandIfDue(::state_machine::StateContext& ctx) {
    if (hasCommand()) {
        emitCommandIfDue(ctx);
    } else {
        emitZero(ctx);
    }
}

bool Custom1State::hasCommand() const {
    return controller_.commandReady();
}

void Custom1State::emitCommandIfDue(::state_machine::StateContext& ctx) {
    if (!hasCommand()) {
        emitZero(ctx);
        return;
    }
    const auto cfg = controller_.config();
    if (!command_gate_.due(controller_.currentTime(), 1.0 / cfg.command_publish_rate_hz)) {
        return;
    }
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

void Custom1State::emitZero(::state_machine::StateContext& ctx, bool force) {
    // Stop immediately when losing a command, then keep publishing idle zeros
    // at their own cadence. Missing references and failed results may reach
    // this path on every event-pump tick; they must not bypass the output gate.
    if (force || controller_.command().valid) {
        zero_gate_.reset();
    }
    controller_.clearCommand();
    const auto cfg = controller_.config();
    if (!zero_gate_.due(controller_.currentTime(), 1.0 / cfg.idle_cmd_rate_hz)) {
        return;
    }
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_ZERO_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

::state_machine::ActionResult Custom1State::onExit(::state_machine::StateContext& ctx) {
    emitZero(ctx, true);
    solve_gate_.reset();
    command_gate_.reset();
    zero_gate_.reset();
    request_in_flight_ = false;
    have_tick_time_ = false;
    return {};
}

}  // namespace unicycle_ugv_controller
