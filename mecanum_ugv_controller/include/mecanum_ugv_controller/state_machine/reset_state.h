#pragma once

#include <state_machine/state_machine.hpp>
#include <string>

#include "mecanum_ugv_controller/common/types.h"
#include "mecanum_ugv_controller/state_machine/periodic_gate.h"

namespace mecanum_ugv_controller {

class MecanumUgvController;

class ResetState final : public ::state_machine::State {
   public:
    explicit ResetState(MecanumUgvController& controller);
    std::string name() const override {
        return "Reset";
    }
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void emitCommand(::state_machine::StateContext& ctx, const ControlCommand& command);
    void emitZero(::state_machine::StateContext& ctx);
    void postDone(::state_machine::StateContext& ctx, ::state_machine::EventId id);

    MecanumUgvController& controller_;
    PeriodicGate command_gate_;
    double enter_time_{0.0};
    double enter_wall_{0.0};
};

}  // namespace mecanum_ugv_controller
