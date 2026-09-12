#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <functional>
#include <state_machine/state_machine.hpp>

namespace unicycle_ugv_controller {

class CommandInputProducer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;

    CommandInputProducer(ros::NodeHandle& nh, EventSink event_sink, uint32_t queue_size);

   private:
    void namespacedCommandCallback(const std_msgs::String::ConstPtr& msg);
    void publicCommandCallback(const std_msgs::String::ConstPtr& msg);
    void handleCommand(const std_msgs::String::ConstPtr& msg, const char* source);
    void post(::state_machine::EventId id, const char* source);

    EventSink event_sink_;
    ros::Subscriber command_sub_;
    ros::Subscriber namespaced_command_sub_;
};

}  // namespace unicycle_ugv_controller
