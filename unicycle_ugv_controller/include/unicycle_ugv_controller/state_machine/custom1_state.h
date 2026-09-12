#pragma once

#include <state_machine/state_machine.hpp>
#include <string>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/state_machine/periodic_gate.h"

namespace unicycle_ugv_controller {

class UnicycleUgvController;

class Custom1State final : public ::state_machine::State {
   public:
    explicit Custom1State(UnicycleUgvController& controller);
    std::string name() const override {
        return "Custom1";
    }
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onEvent(::state_machine::StateContext& ctx,
                                          const ::state_machine::Event& event) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void requestSolveIfDue(::state_machine::StateContext& ctx);
    void publishNmpcCommandIfDue(::state_machine::StateContext& ctx);
    void tickFlatness(::state_machine::StateContext& ctx);
    bool hasCommand() const;
    void emitCommandIfDue(::state_machine::StateContext& ctx);
    void emitZero(::state_machine::StateContext& ctx, bool force = false);

    UnicycleUgvController& controller_;
    PeriodicGate solve_gate_;
    PeriodicGate command_gate_;
    PeriodicGate zero_gate_;
    uint64_t request_sequence_{0U};
    uint64_t in_flight_sequence_{0U};
    bool request_in_flight_{false};
    double request_deadline_{0.0};
    double body_speed_{0.0};
    double last_tick_time_{0.0};
    bool have_tick_time_{false};
};

}  // namespace unicycle_ugv_controller
