#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mecanum_ugv_controller/common/types.h"
#include "mecanum_ugv_controller/mecanum_ugv_controller.h"

namespace mecanum_ugv_controller {
namespace {

bool hasOutputEvent(MecanumUgvController& controller, ::state_machine::EventId id) {
    const auto& events = controller.stateMachine().currentOutputEvents();
    return std::any_of(events.begin(), events.end(),
                       [id](const ::state_machine::Event& event) { return event.id == id; });
}

void setPose(UgvState& state, double t, double x, double y, double yaw) {
    state.received = true;
    state.stamp = ros::Time(t);
    state.x = x;
    state.y = y;
    state.yaw = yaw;
}

void postCommand(MecanumUgvController& controller, ::state_machine::EventId id, double t) {
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{t});
    event.source = "test";
    event.category = ::state_machine::EventCategory::kInput;
    ASSERT_TRUE(controller.postEvent(std::move(event)).ok());
}

void goReady(MecanumUgvController& controller, UgvState& state, double t) {
    setPose(state, t, 0.0, 0.0, 0.0);
    controller.update(t);
    controller.update(t + 0.002);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

// FLU holonomic kinematics: body vx, vy, ω. No wheel or motor dynamics.
void plantWorldVelocity(const UgvState& state, const ControlCommand& command, double& vwx,
                        double& vwy) {
    const double c = std::cos(state.yaw);
    const double s = std::sin(state.yaw);
    const double vx = command.valid ? command.linear_x : 0.0;
    const double vy = command.valid ? command.linear_y : 0.0;
    vwx = c * vx - s * vy;
    vwy = s * vx + c * vy;
}

void stepHolonomicPlant(UgvState& state, const ControlCommand& command, double dt) {
    const double c = std::cos(state.yaw);
    const double s = std::sin(state.yaw);
    const double vx = command.valid ? command.linear_x : 0.0;
    const double vy = command.valid ? command.linear_y : 0.0;
    const double omega = command.valid ? command.angular_z : 0.0;
    const double vwx = c * vx - s * vy;
    const double vwy = s * vx + c * vy;
    state.x += vwx * dt;
    state.y += vwy * dt;
    state.yaw = wrapAngle(state.yaw + omega * dt);
    state.stamp = ros::Time(state.stamp.toSec() + dt);
}

void enterCustom1(MecanumUgvController& controller, UgvState& state,
                  WorldVelocityReference& reference, double t) {
    goReady(controller, state, t);
    controller.setWorldReference(reference);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, t + 0.01);
    setPose(state, t + 0.01, state.x, state.y, state.yaw);
    controller.update(t + 0.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
}

}  // namespace

TEST(MecanumLaw, BoxSaturateClampsEachFluAxis) {
    UgvState state;
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 3.0;
    reference.vy = -3.0;
    ControllerConfig config;
    const HolonomicTrackOutput output = computeHolonomicTrackCommand(state, reference, config);
    EXPECT_NEAR(output.linear_x, 1.0, 1.0e-9);
    EXPECT_NEAR(output.linear_y, -1.0, 1.0e-9);
}

TEST(MecanumLaw, TrackRotatesWorldVelocityWithoutAssumingYawZero) {
    UgvState state;
    state.yaw = 1.5707963267948966;
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 1.0;
    const HolonomicTrackOutput output =
        computeHolonomicTrackCommand(state, reference, ControllerConfig{});
    EXPECT_NEAR(output.linear_x, 0.0, 1.0e-6);
    EXPECT_NEAR(output.linear_y, -1.0, 1.0e-6);
}

TEST(MecanumLaw, HeadingPUsesShortestAngleAcrossPi) {
    EXPECT_NEAR(headingRateToTarget(3.1, -3.1, 1.0, 1.0), wrapAngle(-3.1 - 3.1), 1.0e-6);
}

TEST(MecanumSm, SelfCheckPublishesFiveHertzZero) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    controller.update(1.0);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
    EXPECT_TRUE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
}

TEST(MecanumSm, FreshPoseInsideFenceMovesSelfCheckToReady) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
}

TEST(MecanumSm, MissingPoseStaysSelfCheck) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    controller.update(1.0);
    controller.update(2.0);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(MecanumSm, StalePoseReturnsToSelfCheck) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    controller.update(1.3);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(MecanumSm, NonFinitePoseIsDirty) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    setPose(state, 1.05, std::nan(""), 0.0, 0.0);
    controller.update(1.05);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(MecanumSm, OutsideFenceGoesSelfCheck) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    setPose(state, 1.05, 50.0, 0.0, 0.0);
    controller.update(1.05);
    EXPECT_FALSE(controller.healthReady());
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(MecanumSm, ReadyStopIsNotAtInitialPose) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    setPose(state, 1.0, 1.5, -0.8, 0.4);
    controller.update(1.0);
    controller.update(1.002);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(MecanumSm, ResetStopReturnsReady) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    ResetTarget goal;
    goal.x = 3.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    postCommand(controller, event_type::STOP_REQUESTED, 1.02);
    setPose(state, 1.02, 0.1, 0.0, 0.0);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(MecanumSm, ResetDoesNotJumpToCustom1) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    ResetTarget goal;
    goal.x = 3.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.02);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
}

TEST(MecanumSm, Custom1StopReturnsReady) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.2;
    reference.stamp = ros::Time(1.0);
    controller.setWorldReference(reference);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
    postCommand(controller, event_type::STOP_REQUESTED, 1.02);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(MecanumSm, Custom1DoesNotValidateAlgorithmTimestamp) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.2;
    reference.stamp = ros::Time(1001.0);
    enterCustom1(controller, state, reference, 1.0);

    setPose(state, 1.1, state.x, state.y, state.yaw);
    controller.update(1.1);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
}

TEST(MecanumSm, Custom1CanReset) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.2;
    reference.stamp = ros::Time(1.0);
    controller.setWorldReference(reference);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
    ResetTarget goal;
    goal.x = 1.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.02);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
}

TEST(MecanumSm, ResetPoseDoesNotDriveStateMachine) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    ResetTarget goal;
    goal.x = 2.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(MecanumSm, ResetTimeoutReturnsReady) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.reset_timeout = 0.05;
    controller.setConfig(config);
    goReady(controller, state, 1.0);
    ResetTarget goal;
    goal.x = 4.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    setPose(state, 1.08, 0.0, 0.0, 0.0);
    controller.update(1.08);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(MecanumSm, InvalidFenceStaysSelfCheck) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.fence_x_min = 1.0;
    config.fence_x_max = -1.0;
    controller.setConfig(config);
    setPose(state, 1.0, 0.0, 0.0, 0.0);
    controller.update(1.0);
    controller.update(1.002);
    EXPECT_FALSE(controller.healthReady());
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(MecanumSm, SelfCheckDoesNotJumpToCustom1OrReset) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    controller.update(1.0);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    controller.update(1.01);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
    postCommand(controller, event_type::RESET_REQUESTED, 1.02);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(MecanumLaw, IdealPlantCustom1HeadingAndWorldVelocity) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.command_publish_rate_hz = 500.0;
    controller.setConfig(config);
    goReady(controller, state, 1.0);
    setPose(state, 1.0, 0.0, 0.0, 0.6);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.4;
    reference.vy = 0.0;
    reference.stamp = ros::Time(1.0);
    controller.setWorldReference(reference);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
    double t = 1.01;
    const double dt = 0.02;
    for (int i = 0; i < 250; ++i) {
        t += dt;
        reference.stamp = ros::Time(t);
        controller.setWorldReference(reference);
        stepHolonomicPlant(state, controller.command(), dt);
        controller.update(t);
    }
    EXPECT_NEAR(wrapAngle(state.yaw), 0.0, 0.08);
    EXPECT_GT(state.x, 0.5);
}

TEST(MecanumLaw, IdealPlantCustom1TracksSettledWorldVelocity) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.command_publish_rate_hz = 500.0;
    controller.setConfig(config);
    setPose(state, 1.0, 0.0, 0.0, 0.0);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.5;
    reference.vy = -0.3;
    reference.stamp = ros::Time(1.0);
    enterCustom1(controller, state, reference, 1.0);
    double t = 1.01;
    const double dt = 0.02;
    double vwx = 0.0;
    double vwy = 0.0;
    for (int i = 0; i < 25; ++i) {
        t += dt;
        reference.stamp = ros::Time(t);
        controller.setWorldReference(reference);
        plantWorldVelocity(state, controller.command(), vwx, vwy);
        stepHolonomicPlant(state, controller.command(), dt);
        controller.update(t);
    }
    EXPECT_NEAR(wrapAngle(state.yaw), 0.0, 1.0e-3);
    EXPECT_NEAR(vwx, 0.5, 1.0e-6);
    EXPECT_NEAR(vwy, -0.3, 1.0e-6);
}

TEST(MecanumLaw, IdealPlantCustom1TracksWorldVelocityStep) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.command_publish_rate_hz = 500.0;
    controller.setConfig(config);
    setPose(state, 1.0, 0.0, 0.0, 0.0);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.4;
    reference.vy = 0.0;
    reference.stamp = ros::Time(1.0);
    enterCustom1(controller, state, reference, 1.0);
    double t = 1.01;
    const double dt = 0.02;
    for (int i = 0; i < 10; ++i) {
        t += dt;
        reference.stamp = ros::Time(t);
        controller.setWorldReference(reference);
        stepHolonomicPlant(state, controller.command(), dt);
        controller.update(t);
    }
    reference.vx = 0.2;
    reference.vy = 0.35;
    double vwx = 0.0;
    double vwy = 0.0;
    for (int i = 0; i < 10; ++i) {
        t += dt;
        reference.stamp = ros::Time(t);
        controller.setWorldReference(reference);
        plantWorldVelocity(state, controller.command(), vwx, vwy);
        stepHolonomicPlant(state, controller.command(), dt);
        controller.update(t);
    }
    EXPECT_NEAR(vwx, 0.2, 1.0e-6);
    EXPECT_NEAR(vwy, 0.35, 1.0e-6);
}

TEST(MecanumLaw, IdealPlantCustom1TracksSlowWorldVelocitySine) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.command_publish_rate_hz = 500.0;
    controller.setConfig(config);
    setPose(state, 1.0, 0.0, 0.0, 0.0);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.4;
    reference.vy = 0.0;
    reference.stamp = ros::Time(1.0);
    enterCustom1(controller, state, reference, 1.0);
    double t = 1.01;
    const double dt = 0.02;
    double max_err = 0.0;
    for (int i = 0; i < 250; ++i) {
        t += dt;
        reference.vx = 0.4 * std::cos(0.4 * t);
        reference.vy = 0.4 * std::sin(0.4 * t);
        reference.stamp = ros::Time(t);
        controller.setWorldReference(reference);
        controller.update(t);
        double vwx = 0.0;
        double vwy = 0.0;
        plantWorldVelocity(state, controller.command(), vwx, vwy);
        max_err = std::max(max_err, std::hypot(vwx - reference.vx, vwy - reference.vy));
        stepHolonomicPlant(state, controller.command(), dt);
    }
    EXPECT_NEAR(wrapAngle(state.yaw), 0.0, 0.02);
    EXPECT_LT(max_err, 0.02);
}

TEST(MecanumLaw, IdealPlantCustom1TracksCircleWithHeadingDisturbance) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.command_publish_rate_hz = 500.0;
    controller.setConfig(config);
    constexpr double kRadius = 0.8;
    constexpr double kOmega = 0.5;
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.0;
    reference.vy = kRadius * kOmega;
    reference.stamp = ros::Time(1.0);
    enterCustom1(controller, state, reference, 1.0);
    setPose(state, 1.01, kRadius, 0.0, 0.0);
    controller.update(1.01);
    double t = 1.01;
    const double dt = 0.02;
    double max_radial = 0.0;
    for (int i = 0; i < 650; ++i) {
        t += dt;
        const double angle = kOmega * (t - 1.01);
        reference.vx = -kRadius * kOmega * std::sin(angle);
        reference.vy = kRadius * kOmega * std::cos(angle);
        reference.stamp = ros::Time(t);
        controller.setWorldReference(reference);
        controller.update(t);
        stepHolonomicPlant(state, controller.command(), dt);
        if (i % 75 == 37) {
            state.yaw = wrapAngle(state.yaw + 0.1);
        }
        max_radial = std::max(max_radial, std::abs(std::hypot(state.x, state.y) - kRadius));
    }
    EXPECT_LT(max_radial, 0.05);
    EXPECT_LT(std::abs(wrapAngle(state.yaw)), 0.15);
}

TEST(MecanumLaw, IdealPlantCustom1SaturatesWorldVelocityOnFluBox) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.command_publish_rate_hz = 500.0;
    controller.setConfig(config);
    setPose(state, 1.0, 0.0, 0.0, 0.0);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 2.0;
    reference.vy = -2.0;
    reference.stamp = ros::Time(1.0);
    enterCustom1(controller, state, reference, 1.0);
    double t = 1.01;
    const double dt = 0.02;
    t += dt;
    reference.stamp = ros::Time(t);
    controller.setWorldReference(reference);
    controller.update(t);
    double vwx = 0.0;
    double vwy = 0.0;
    plantWorldVelocity(state, controller.command(), vwx, vwy);
    EXPECT_NEAR(vwx, 1.0, 1.0e-9);
    EXPECT_NEAR(vwy, -1.0, 1.0e-9);
}

TEST(MecanumSm, ResetHasNoUnfilteredFallbackAndCancelsLateResponses) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    ResetTarget goal;
    goal.x = 1.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    controller.update(1.012);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    EXPECT_DOUBLE_EQ(controller.command().linear_x, 0.0);
    EXPECT_DOUBLE_EQ(controller.command().linear_y, 0.0);
    auto& session = controller.resetSession();
    const double wall = ugv_reset_safety::monotonicSeconds();
    const auto issued = session.issue({0.0, 0.0, 0.0}, ros::Time(1.012).toNSec(), wall);
    ASSERT_TRUE(issued.valid);
    ASSERT_TRUE(
        session.accept(issued.generation, issued.stamp, 0, {0.1, 0.0, 0.0}, issued.stamp, wall));
    controller.update(1.014);
    EXPECT_DOUBLE_EQ(controller.command().linear_x, 0.1);
    postCommand(controller, event_type::STOP_REQUESTED, 1.016);
    controller.update(1.016);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
    EXPECT_FALSE(session.active());
    EXPECT_FALSE(
        session.accept(issued.generation, issued.stamp, 0, {0.1, 0.0, 0.0}, issued.stamp, wall));
    EXPECT_DOUBLE_EQ(controller.command().linear_x, 0.0);
    postCommand(controller, event_type::RESET_REQUESTED, 1.018);
    controller.update(1.018);
    EXPECT_GT(session.generation(), issued.generation);
    EXPECT_FALSE(
        session.accept(issued.generation, issued.stamp, 0, {0.1, 0.0, 0.0}, issued.stamp, wall));
}

TEST(MecanumSm, ResetRequiresCoordinatorArrivalEvenAtTarget) {
    ros::Time::init();
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    ResetTarget goal;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    controller.update(1.012);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    auto& session = controller.resetSession();
    const double wall = ugv_reset_safety::monotonicSeconds();
    const auto issued = session.issue({0.0, 0.0, 0.0}, ros::Time(1.012).toNSec(), wall);
    ASSERT_TRUE(session.accept(issued.generation, issued.stamp, 1, {}, issued.stamp, wall));
    controller.update(1.014);
    controller.update(1.016);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
    EXPECT_FALSE(session.active());
}
}  // namespace mecanum_ugv_controller
