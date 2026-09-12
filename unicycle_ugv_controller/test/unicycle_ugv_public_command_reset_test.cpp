#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

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

TEST(UnicyclePublicCommandReset, PublicCommandEntersAndHoldsResetFromReadyAndCustom1) {
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ugv_test/pose", 10);
    ros::Publisher public_command_pub = nh.advertise<std_msgs::String>("/command", 10);
    std::string control_state;
    ros::Subscriber state_sub = nh.subscribe<std_msgs::String>(
        "/ugv_test/custom/statustext", 10,
        [&](const std_msgs::String::ConstPtr& msg) { control_state = msg->data; });

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
            return control_state == "Reset";
        },
        0.2))
        << "public /command reset must enter Reset within 200 ms, got " << control_state;

    const ros::Time hold_until = ros::Time::now() + ros::Duration(0.2);
    while (ros::ok() && ros::Time::now() < hold_until) {
        pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
        ros::spinOnce();
        EXPECT_EQ(control_state, "Reset") << "Reset must hold until a real terminal";
        ros::Duration(0.02).sleep();
    }

    std_msgs::String stop;
    stop.data = "stop";
    public_command_pub.publish(stop);
    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return control_state == "Ready";
        },
        1.0))
        << "stop must return Ready, got " << control_state;

    std_msgs::String custom1;
    custom1.data = "custom1";
    public_command_pub.publish(custom1);
    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return control_state == "Custom1";
        },
        1.0))
        << "expected Custom1, got " << control_state;

    public_command_pub.publish(reset);
    ASSERT_TRUE(waitFor(
        [&]() {
            pose_pub.publish(makePose(0.0, 0.0, 0.0, ros::Time::now()));
            return control_state == "Reset";
        },
        0.2))
        << "public /command reset from Custom1 must enter Reset within 200 ms, got "
        << control_state;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "unicycle_ugv_public_command_reset_test");
    return RUN_ALL_TESTS();
}
