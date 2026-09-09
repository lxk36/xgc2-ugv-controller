#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <ugv_reset_safety/reset_client.h>

#include <memory>
#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <state_machine/state_machine.hpp>
#include <string>
#include <vector>

#include "unicycle_ugv_controller/input/command_input_producer.h"
#include "unicycle_ugv_controller/input/pva_reference_input_producer.h"
#include "unicycle_ugv_controller/input/reference_input_producer.h"
#include "unicycle_ugv_controller/input/reset_target_input_producer.h"
#include "unicycle_ugv_controller/input/state_input_producer.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

class UnicycleUgvRosNode {
   public:
    explicit UnicycleUgvRosNode(ros::NodeHandle& nh);
    ~UnicycleUgvRosNode();

    void run(double frequency_hz);

   private:
    void loadParams();
    void seedResetTarget();
    void updateOnce();
    void dispatchOutputEvents(const std::vector<::state_machine::Event>& events);
    void publishStatusIfDue(const ros::Time& now);
    void logStateChanges(::state_machine::StateId control_state,
                         ::state_machine::StateId health_state);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    UgvState state_;
    UnicycleUgvController controller_;
    ugv_reset_safety::ResetClient reset_client_;
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> output_executor_;
    ::state_machine::runtime::EventDispatcher output_dispatcher_;
    std::unique_ptr<CommandInputProducer> command_input_;
    std::unique_ptr<StateInputProducer> state_input_;
    std::unique_ptr<ReferenceInputProducer> reference_input_;
    std::unique_ptr<PvaReferenceInputProducer> pva_reference_input_;
    std::unique_ptr<ResetTargetInputProducer> reset_target_input_;

    ControllerConfig config_{};
    uint32_t queue_size_{10U};
    std::string state_topic_{"alg/state_estimator/state"};
    std::string platform_pose_topic_{"pose"};
    std::string reset_pose_topic_{"reset_pose"};
    std::string active_analytic_topic_{"alg/unicycle_reference_trajectory/active/analytic"};
    std::string active_polynomial_topic_{"alg/unicycle_reference_trajectory/active/polynomial"};
    std::string active_sampled_topic_{"alg/unicycle_reference_trajectory/active/sampled"};
    std::string pva_reference_topic_{"alg/reference/pva"};
    std::string cmd_vel_topic_{"cmd_vel"};
    std::string control_state_topic_{"alg/unicycle_ugv_controller/status/control_state"};
    std::string health_state_topic_{"alg/unicycle_ugv_controller/status/health_state"};
    double status_publish_rate_hz_{10.0};
    ros::Publisher control_state_pub_;
    ros::Publisher health_state_pub_;
    ros::Time last_status_stamp_;
    ::state_machine::StateId last_logged_control_state_{0U};
    ::state_machine::StateId last_logged_health_state_{0U};
};

}  // namespace unicycle_ugv_controller
