#pragma once

#include <state_machine/state_machine.hpp>
#include <string>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/state_machine/periodic_gate.h"

namespace unicycle_ugv_controller {

class UnicycleUgvController;

class ResetState final : public ::state_machine::State {
   public:
    explicit ResetState(UnicycleUgvController& controller);
    std::string name() const override {
        return "Reset";
    }
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void emitCommand(::state_machine::StateContext& ctx, const ControlCommand& command);
    void emitZero(::state_machine::StateContext& ctx, bool force = false);
    void postDone(::state_machine::StateContext& ctx, ::state_machine::EventId id);

    UnicycleUgvController& controller_;
    PeriodicGate command_gate_;
    double enter_time_{0.0};
    double enter_wall_{0.0};
};

}  // namespace unicycle_ugv_controller
