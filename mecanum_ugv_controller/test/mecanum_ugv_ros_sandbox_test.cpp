#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <ugv_reset_safety/ResetRequest.h>
#include <ugv_reset_safety/ResetResponse.h>

#include <cmath>
#include <functional>

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

TEST(MecanumRosSandbox, PoseCommandFenceAndDrop) {
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ugv_test/pose", 10);
    ros::Publisher command_pub = nh.advertise<std_msgs::String>("/ugv_test/command", 10);
    ros::Publisher reset_pub = nh.advertise<geometry_msgs::Pose2D>("/ugv_test/reset_pose", 10);

    ros::Publisher safe_pub =
        nh.advertise<ugv_reset_safety::ResetResponse>("/ugv_test/reset/response", 1);
    bool grant_response = false;
    ros::Subscriber request_sub = nh.subscribe<ugv_reset_safety::ResetRequest>(
        "/ugv_test/reset/request", 1, [&](const ugv_reset_safety::ResetRequest::ConstPtr& request) {
            if (!grant_response) {
                return;
            }
            ugv_reset_safety::ResetResponse reply;
            reply.header = request->header;
            reply.generation = request->generation;
            reply.status = ugv_reset_safety::ResetResponse::RUNNING;
            reply.command.linear.x = 0.1;
            safe_pub.publish(reply);
        });
    uint32_t control_state = 0;
    geometry_msgs::Twist last_cmd;
    ros::Subscriber state_sub = nh.subscribe<std_msgs::UInt32>(
        "/ugv_test/alg/mecanum_ugv_controller/status/control_state", 10,
        [&](const std_msgs::UInt32::ConstPtr& msg) { control_state = msg->data; });
    ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>(
        "/ugv_test/cmd_vel", 10,
        [&](const geometry_msgs::Twist::ConstPtr& msg) { last_cmd = *msg; });

    ASSERT_TRUE(waitFor(
        [&]() {
            return pose_pub.getNumSubscribers() > 0 && command_pub.getNumSubscribers() > 0 &&
                   reset_pub.getNumSubscribers() > 0 && state_sub.getNumPublishers() > 0 &&
                   cmd_sub.getNumPublishers() > 0;
        },
        5.0));

    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return control_state == 2u;
        },
        3.0))
        << "expected Ready";

    geometry_msgs::Pose2D goal;
    goal.x = 0.8;
    goal.y = 0.0;
    reset_pub.publish(goal);
    ros::Duration(0.05).sleep();
    ros::spinOnce();
    EXPECT_EQ(control_state, 2u) << "reset_pose must not drive the state machine";

    std_msgs::String reset;
    reset.data = "reset";
    command_pub.publish(reset);
    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return control_state == 5u;
        },
        2.0))
        << "expected Reset";
    EXPECT_DOUBLE_EQ(last_cmd.linear.x, 0.0) << "missing coordinator must never drive";
    grant_response = true;
    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return last_cmd.linear.x > 0.0;
        },
        2.0));
    grant_response = false;
    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return last_cmd.linear.x == 0.0;
        },
        1.0))
        << "expired coordinator lease must stop";

    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(50.0, 0.0, 0.0, ros::Time::now()));
            return control_state == 1u;
        },
        2.0))
        << "fence should SelfCheck";

    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return control_state == 2u;
        },
        3.0))
        << "back inside fence -> Ready";

    ros::Duration(0.5).sleep();
    ros::spinOnce();
    EXPECT_EQ(control_state, 1u) << "dropped pose should SelfCheck";
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "mecanum_ugv_ros_sandbox_test");
    return RUN_ALL_TESTS();
}
