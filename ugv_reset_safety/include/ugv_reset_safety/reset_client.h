#pragma once

#include <ros/ros.h>
#include <ugv_reset_safety/ResetRequest.h>
#include <ugv_reset_safety/ResetResponse.h>
#include <ugv_reset_safety/reset_session.h>

namespace ugv_reset_safety {

class ResetClient {
   public:
    ResetClient(ros::NodeHandle& nh, ResetSession& session) : session_(session) {
        request_pub_ = nh.advertise<ResetRequest>("reset/request", 1);
        response_sub_ = nh.subscribe("reset/response", 1, &ResetClient::receive, this);
    }
    void update(ResetSession::Pose pose, ros::Time pose_stamp, bool healthy) {
        if (!healthy) {
            return;
        }
        const auto request = session_.issue(pose, ros::Time::now().toNSec(), monotonicSeconds());
        if (!request.valid) {
            return;
        }
        ResetRequest message;
        message.generation = request.generation;
        message.header.stamp.fromNSec(request.stamp);
        message.header.frame_id = "world";
        message.pose_stamp = pose_stamp;
        message.applied_stamp.fromNSec(request.applied_stamp);
        message.applied_command.linear.x = request.applied_command.x;
        message.applied_command.linear.y = request.applied_command.y;
        message.applied_command.angular.z = request.applied_command.yaw;
        message.pose.x = request.pose.x;
        message.pose.y = request.pose.y;
        message.pose.theta = request.pose.yaw;
        message.target.x = request.target.x;
        message.target.y = request.target.y;
        message.target.theta = request.target.yaw;
        request_pub_.publish(message);
    }

   private:
    void receive(const ResetResponse::ConstPtr& message) {
        if (!message || message->header.frame_id != "world" || message->command.linear.z != 0 ||
            message->command.angular.x != 0 || message->command.angular.y != 0) {
            return;
        }
        session_.accept(
            message->generation, message->header.stamp.toNSec(), message->status,
            {message->command.linear.x, message->command.linear.y, message->command.angular.z},
            ros::Time::now().toNSec(), monotonicSeconds());
    }
    ResetSession& session_;
    ros::Publisher request_pub_;
    ros::Subscriber response_sub_;
};

}  // namespace ugv_reset_safety
