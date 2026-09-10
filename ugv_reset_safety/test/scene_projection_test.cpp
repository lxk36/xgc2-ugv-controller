#include <gtest/gtest.h>
#include <ugv_reset_safety/scene_projection.h>
namespace {
xgc2_geometry_msgs::SceneSnapshot fixture(const std::string& type) {
    xgc2_geometry_msgs::SceneSnapshot scene;
    scene.epoch = "test";
    scene.header.frame_id = "world";
    xgc2_geometry_msgs::SceneObstacle obstacle;
    obstacle.id = "gate";
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
TEST(SceneProjection, ExplicitlyRejectsDynamicOrUnknownGeometry) {
    auto scene = fixture("sphere");
    scene.obstacles[0].dynamic = true;
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    scene.obstacles[0].dynamic = false;
    scene.obstacles[0].parts[0].geometry.type = "torus";
    EXPECT_THROW(ugv_reset_safety::scene_projection::project(scene, "world"),
                 std::invalid_argument);
    scene.obstacles.clear();
    EXPECT_TRUE(ugv_reset_safety::scene_projection::project(scene, "world").empty());
}
}  // namespace
