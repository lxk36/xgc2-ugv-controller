#pragma once

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>

#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

class CmdVelOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    CmdVelOutputConsumer(ros::NodeHandle& nh,
                         ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
                         UnicycleUgvController& controller, const std::string& cmd_vel_topic,
                         uint32_t queue_size);

    std::string name() const override {
        return "CmdVelOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;

   private:
    geometry_msgs::Twist makeTwist(const ControlCommand& command) const;

    UnicycleUgvController& controller_;
    ros::Publisher cmd_vel_pub_;
};

}  // namespace unicycle_ugv_controller
