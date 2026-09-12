#include <ugv_reset_safety/reset_geometry.h>

#include <cmath>

namespace ugv_reset_safety {
namespace {
bool positive(double v) {
    return std::isfinite(v) && v > 0.0;
}
}  // namespace

std::vector<FootprintDisk> coveringDisks(const Robot& robot, int count) {
    std::vector<FootprintDisk> disks;
    if (count < 1 || !positive(robot.half_length) || !positive(robot.half_width) ||
        !std::isfinite(robot.yaw) || !robot.position.allFinite() ||
        !robot.body_center_offset.allFinite()) {
        return disks;
    }
    const double c = std::cos(robot.yaw);
    const double s = std::sin(robot.yaw);
    Eigen::Matrix2d rotation;
    rotation << c, -s, s, c;
    const double slice = robot.half_length / static_cast<double>(count);
    for (int k = 0; k < count; ++k) {
        const Eigen::Vector2d body_offset =
            robot.body_center_offset +
            Eigen::Vector2d(-robot.half_length + (2 * k + 1) * slice, 0.0);
        const Eigen::Vector2d offset = rotation * body_offset;
        FootprintDisk disk;
        disk.center = robot.position + offset;
        disk.radius = std::hypot(slice, robot.half_width);
        disks.push_back(disk);
    }
    return disks;
}

}  // namespace ugv_reset_safety
