#include <gtest/gtest.h>
#include <ugv_reset_safety/scene_projection.h>

#include <algorithm>
#include <limits>
namespace {
xgc2_geometry_msgs::SceneSnapshot fixture(const std::string& type) {
    xgc2_geometry_msgs::SceneSnapshot scene;
    scene.epoch = "test";
    scene.header.frame_id = "world";
    xgc2_geometry_msgs::SceneObstacle obstacle;
    obstacle.id = "gate";
    obstacle.motion_type = "hold";
    obstacle.pose.orientation.w = 1;
    xgc2_geometry_msgs::ScenePart part;
    part.id = "left";
    part.pose.orientation.w = 1;
    part.geometry.type = type;
    part.geometry.radius = .5;
    part.geometry.height = 1;
    part.geometry.size.x = .5;
    part.geometry.size.y = .5;
    part.geometry.size.z = 2;
    obstacle.parts.push_back(part);
    scene.obstacles.push_back(obstacle);
    return scene;
}
xgc2_geometry_msgs::SceneState liveState(const xgc2_geometry_msgs::SceneSnapshot& scene,
                                           double x = 0, double vx = 0, double omega = 0) {
    xgc2_geometry_msgs::SceneState state;
    state.header.frame_id = scene.header.frame_id;
    state.epoch = scene.epoch;
    state.revision = scene.revision;
    for (const auto& obstacle : scene.obstacles) {
        xgc2_geometry_msgs::SceneObstacleState body;
        body.id = obstacle.id;
        body.pose = obstacle.pose;
        body.pose.position.x += x;
        body.twist.linear.x = vx;
        body.twist.angular.z = omega;
        state.obstacles.push_back(body);
    }
    return state;
}
TEST(SceneProjection, CurvesAreCircumscribedInsteadOfInscribed) {
    for (const auto& type : {"sphere", "cylinder", "capsule"}) {
        auto scene = fixture(type);
        auto projected = ugv_reset_safety::scene_projection::project(scene, "world");
        ASSERT_EQ(projected.size(), 1U);
        for (const auto& vertex : projected[0].vertices) {
            EXPECT_GE(vertex.norm(), .5 - 1e-12);
            EXPECT_LE(vertex.norm(), .5 / std::cos(M_PI / 32) + 1e-10);
        }
    }
}
TEST(SceneProjection, KeepsCompoundOpeningAndStableIdentities) {
    auto scene = fixture("box");
    scene.obstacles[0].parts[0].pose.position.x = -1;
    auto right = scene.obstacles[0].parts[0];
    right.id = "right";
    right.pose.position.x = 1;
    scene.obstacles[0].parts.push_back(right);
    const auto parts = ugv_reset_safety::scene_projection::project(scene, "world");
    ASSERT_EQ(parts.size(), 2U);
    for (const auto& vertex : parts[0].vertices)
        EXPECT_LT(vertex.x(), -.7);
    for (const auto& vertex : parts[1].vertices)
        EXPECT_GT(vertex.x(), .7);
    scene.obstacles[0].parts.erase(scene.obstacles[0].parts.begin());
    const auto remaining = ugv_reset_safety::scene_projection::project(scene, "world");
    EXPECT_EQ(remaining[0].id, parts[1].id);
}
TEST(SceneProjection, RejectsUnknownGeometryNonfiniteDataAndFrameMismatch) {
    auto scene = fixture("sphere");
    scene.obstacles[0].parts[0].geometry.type = "torus";
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    scene = fixture("sphere");
    scene.obstacles[0].pose.position.x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    scene = fixture("sphere");
    scene.header.frame_id = "map";
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    scene = fixture("sphere");
    scene.obstacles[0].id.clear();
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    scene = fixture("sphere");
    scene.obstacles.clear();
    EXPECT_TRUE(ugv_reset_safety::scene_projection::project(scene, "world").empty());
}
TEST(SceneProjection, LiveStateMovesDynamicGeometryWithoutUsingRestPose) {
    auto scene = fixture("box");
    scene.obstacles[0].dynamic = true;
    scene.obstacles[0].motion_type = "constant_twist";
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    auto state = liveState(scene, 1.5, 0.4);
    const auto geometry = ugv_reset_safety::scene_projection::live(scene, state, "world");
    ASSERT_EQ(geometry.live.size(), 1U);
    double xmin = std::numeric_limits<double>::infinity();
    for (const auto& vertex : geometry.live[0].vertices)
        xmin = std::min(xmin, vertex.x());
    EXPECT_GT(xmin, 1.0);
    EXPECT_NEAR(geometry.live[0].velocity.x(), 0.4, 1e-12);
    EXPECT_GT(geometry.occupancy[0].vertices.size(), 2U);
    double occupancy_span = 0;
    for (const auto& vertex : geometry.occupancy[0].vertices)
        occupancy_span = std::max(occupancy_span, (vertex - geometry.live[0].origin).norm());
    double live_span = 0;
    for (const auto& vertex : geometry.live[0].vertices)
        live_span = std::max(live_span, (vertex - geometry.live[0].origin).norm());
    EXPECT_GT(occupancy_span, live_span + 0.5);
}
TEST(SceneProjection, PausedDynamicUsesLivePoseWithZeroTwist) {
    auto scene = fixture("cylinder");
    scene.obstacles[0].dynamic = true;
    scene.obstacles[0].motion_type = "ping_pong";
    auto state = liveState(scene, 2.0, 0.0);
    const auto geometry = ugv_reset_safety::scene_projection::live(scene, state, "world");
    ASSERT_EQ(geometry.live.size(), 1U);
    EXPECT_NEAR(geometry.live[0].origin.x(), 2.0, 1e-9);
    EXPECT_EQ(geometry.live[0].velocity, Eigen::Vector2d::Zero());
    EXPECT_NEAR(geometry.occupancy[0].vertices.front().x(), geometry.live[0].vertices.front().x(),
                1e-8);
}
TEST(SceneProjection, RejectsOldEpochMissingIdentitiesAndUnknownMotion) {
    auto scene = fixture("box");
    scene.obstacles[0].dynamic = true;
    scene.obstacles[0].motion_type = "circle";
    auto state = liveState(scene, 0.2, 0.1);
    state.epoch = "old";
    EXPECT_THROW(ugv_reset_safety::scene_projection::live(scene, state, "world"),
                 std::invalid_argument);
    state = liveState(scene, 0.2, 0.1);
    state.obstacles.clear();
    EXPECT_THROW(ugv_reset_safety::scene_projection::live(scene, state, "world"),
                 std::invalid_argument);
    scene.obstacles[0].motion_type = "spiral";
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
}
}  // namespace
