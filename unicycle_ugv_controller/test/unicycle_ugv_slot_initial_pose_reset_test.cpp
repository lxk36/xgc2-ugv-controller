#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ugv_reset_safety/ResetRequest.h>

#include <cmath>
#include <functional>
#include <string>

namespace {

geometry_msgs::PoseStamped makePose(double x, double y, double yaw, const ros::Time& stamp) {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.orientation.z = std::sin(yaw * 0.5);
    msg.pose.orientation.w = std::cos(yaw * 0.5);
    return msg;
}

bool waitFor(const std::function<bool()>& pred, double seconds) {
    const ros::Time deadline = ros::Time::now() + ros::Duration(seconds);
    ros::Rate rate(50.0);
    while (ros::ok() && ros::Time::now() < deadline) {
        ros::spinOnce();
        if (pred()) {
            return true;
        }
        rate.sleep();
    }
    return pred();
}

}  // namespace

TEST(UnicycleSlotInitialPoseReset, LaunchArgSlotPoseBecomesResetTarget) {
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ugv_test/pose", 10);
    ros::Publisher public_command_pub = nh.advertise<std_msgs::String>("/command", 10);
    std::string control_state;
    ugv_reset_safety::ResetRequest request;
    bool have_request = false;
    ros::Subscriber state_sub = nh.subscribe<std_msgs::String>(
        "/ugv_test/custom/statustext", 10,
        [&](const std_msgs::String::ConstPtr& msg) { control_state = msg->data; });
    ros::Subscriber request_sub = nh.subscribe<ugv_reset_safety::ResetRequest>(
        "/ugv_test/reset/request", 10, [&](const ugv_reset_safety::ResetRequest::ConstPtr& msg) {
            request = *msg;
            have_request = true;
        });

    ASSERT_TRUE(waitFor(
        [&]() {
            return pose_pub.getNumSubscribers() > 0 && public_command_pub.getNumSubscribers() > 0 &&
                   state_sub.getNumPublishers() > 0;
        },
        5.0));

    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return control_state == "Ready";
        },
        3.0))
        << "expected Ready, got " << control_state;

    std_msgs::String reset;
    reset.data = "reset";
    public_command_pub.publish(reset);
    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return control_state == "Reset" && have_request;
        },
        1.0))
        << "slot-backed Reset must publish a request, CONTROL=" << control_state;

    EXPECT_NEAR(request.target.x, 1.25, 1e-6);
    EXPECT_NEAR(request.target.y, -0.4, 1e-6);
    EXPECT_NEAR(request.target.theta, 0.3, 1e-6);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "unicycle_ugv_slot_initial_pose_reset_test");
    return RUN_ALL_TESTS();
}
