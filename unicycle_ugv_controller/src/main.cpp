#include <ros/ros.h>

#include <cmath>
#include <exception>

#include "unicycle_ugv_controller/unicycle_ugv_ros_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "unicycle_ugv_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double control_rate_hz = 500.0;
    private_nh.param("control_rate_hz", control_rate_hz, control_rate_hz);
    if (!std::isfinite(control_rate_hz) || control_rate_hz <= 0.0) {
        ROS_WARN("[UnicycleUgvController] Invalid control_rate_hz; using 500 Hz");
        control_rate_hz = 500.0;
    }

    try {
        unicycle_ugv_controller::UnicycleUgvRosNode node(nh);
        node.run(control_rate_hz);
    } catch (const std::exception& error) {
        ROS_FATAL("[UnicycleUgvController] %s", error.what());
        return 1;
    }
    return 0;
}
