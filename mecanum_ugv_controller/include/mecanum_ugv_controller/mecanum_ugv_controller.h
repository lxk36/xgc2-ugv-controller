#pragma once

#include <ugv_reset_safety/reset_session.h>

#include <memory>
#include <mutex>
#include <state_machine/state_machine.hpp>
#include <string>

#include "mecanum_ugv_controller/common/types.h"

namespace mecanum_ugv_controller {

class MecanumUgvController {
   public:
    explicit MecanumUgvController(const UgvState& state);
    void update(double now_sec);
    ::state_machine::Status postEvent(::state_machine::Event event);
    const UgvState& state() const {
        return state_;
    }
    double currentTime() const {
        return current_time_sec_;
    }
    ControllerConfig config() const;
    void setConfig(const ControllerConfig& config);
    ::state_machine::StateMachine& stateMachine() {
        return *machine_;
    }
    bool healthReady() const;
    bool resetTargetReady() const;
    void setResetTarget(ResetTarget target);
    ResetTarget resetTarget() const;
    bool worldReferenceReady() const;
    void setWorldReference(WorldVelocityReference reference);
    WorldVelocityReference worldReference() const;
    void setCommand(ControlCommand command);
    ControlCommand command() const;
    void clearCommand();
    ugv_reset_safety::ResetSession& resetSession() {
        return reset_session_;
    }
    const ugv_reset_safety::ResetSession& resetSession() const {
        return reset_session_;
    }
    const std::string& lastResetAdmissionMiss() const {
        return last_reset_admission_miss_;
    }
    const std::string& lastResetHoldReason() const {
        return last_reset_hold_reason_;
    }
    void setResetHoldReason(std::string reason);

   private:
    void setupMachine();
    void maybeAutoStartCustom1();
    void noteResetAdmissionAfterUpdate();
    std::string describeResetAdmissionMiss(const std::string& source) const;

    ugv_reset_safety::ResetSession reset_session_;
    const UgvState& state_;
    mutable std::mutex config_mutex_;
    ControllerConfig config_;
    mutable std::mutex command_mutex_;
    ControlCommand command_;
    mutable std::mutex reset_mutex_;
    ResetTarget reset_target_;
    mutable std::mutex reference_mutex_;
    WorldVelocityReference world_reference_;
    std::unique_ptr<::state_machine::StateMachine> machine_;
    double current_time_sec_{0.0};
    bool pending_reset_requested_{false};
    std::string pending_reset_source_;
    std::string last_reset_admission_miss_;
    std::string last_reset_hold_reason_;
};

}  // namespace mecanum_ugv_controller
