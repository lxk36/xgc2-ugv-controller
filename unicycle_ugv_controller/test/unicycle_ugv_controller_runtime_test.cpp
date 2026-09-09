#include <gtest/gtest.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

#include "unicycle_ugv_controller/common/reference_cache.h"
#include "unicycle_ugv_controller/common/rigid_to_unicycle.h"
#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/nmpc/unicycle_nmpc_solver.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {
namespace {

bool hasOutputEvent(UnicycleUgvController& controller, ::state_machine::EventId id) {
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

void postCommand(UnicycleUgvController& controller, ::state_machine::EventId id, double t) {
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{t});
    event.source = "test";
    event.category = ::state_machine::EventCategory::kInput;
    ASSERT_TRUE(controller.postEvent(std::move(event)).ok());
}

void goReadyEstimator(UnicycleUgvController& controller, UgvState& state, double t) {
    state.received = true;
    state.stamp = ros::Time(t);
    state.estimator_state = rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
    state.estimator_flags = 0U;
    controller.update(t);
    controller.update(t + 0.002);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

void goReadyPose(UnicycleUgvController& controller, UgvState& state, double t) {
    auto config = controller.config();
    config.state_source = StateSource::PLATFORM_POSE;
    controller.setConfig(config);
    setPose(state, t, 0.0, 0.0, 0.0);
    controller.update(t);
    controller.update(t + 0.002);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

void makeCustom1Ready(UnicycleUgvController& controller, UgvState& state) {
    goReadyEstimator(controller, state, 1.0);
    unicycle_reference_trajectory_msgs::AnalyticReference reference;
    reference.trajectory_id = 1U;
    reference.revision = 1U;
    reference.analytic_type = unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_HOLD;
    reference.start_time = ros::Time(1.0);
    reference.duration = 10.0;
    reference.origin.orientation.w = 1.0;
    ASSERT_TRUE(controller.referenceCache().updateAnalytic(reference));
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    state.stamp = ros::Time(1.01);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
}

void stepUnicyclePlant(UgvState& state, const ControlCommand& command, double dt) {
    const double v = command.valid ? command.linear_speed : 0.0;
    const double omega = command.valid ? command.angular_speed : 0.0;
    state.x += v * std::cos(state.yaw) * dt;
    state.y += v * std::sin(state.yaw) * dt;
    state.yaw = wrapAngle(state.yaw + omega * dt);
    state.stamp = ros::Time(state.stamp.toSec() + dt);
}

TEST(UnicycleSm, StartsInSelfCheckAndPublishesZero) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    controller.update(1.0);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
    EXPECT_TRUE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
}

TEST(UnicycleSm, RunningEstimatorMovesSelfCheckToReady) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyEstimator(controller, state, 1.0);
}

TEST(UnicycleSm, CanonicalPoseDoesNotNeedEstimatorOrTwist) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
    EXPECT_TRUE(controller.healthReady());
}

TEST(UnicycleSm, FaultFlagReturnsToSelfCheck) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyEstimator(controller, state, 1.0);
    state.estimator_flags = rigid_state_estimator_msgs::RigidStateEstimate::FLAG_FAULT;
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(UnicycleSm, OutsideFenceGoesSelfCheck) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
    setPose(state, 1.05, 50.0, 0.0, 0.0);
    controller.update(1.05);
    EXPECT_FALSE(controller.healthReady());
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(UnicycleSm, ResetStopReturnsReady) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
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

TEST(UnicycleSm, ResetDoesNotJumpToCustom1) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
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

TEST(UnicycleSm, ResetTimeoutReturnsReady) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.state_source = StateSource::PLATFORM_POSE;
    config.reset_timeout = 0.05;
    controller.setConfig(config);
    goReadyPose(controller, state, 1.0);
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

TEST(UnicycleSm, Custom1StopReturnsReady) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    makeCustom1Ready(controller, state);
    postCommand(controller, event_type::STOP_REQUESTED, 1.02);
    state.stamp = ros::Time(1.02);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(UnicycleSm, SelfCheckDoesNotJumpToCustom1OrReset) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    controller.update(1.0);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    controller.update(1.01);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
    postCommand(controller, event_type::RESET_REQUESTED, 1.02);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(UnicycleSm, HoldStateIdIsGone) {
    EXPECT_NE(state_type::Custom1, 4U);
    EXPECT_EQ(state_type::Custom1, 3U);
    EXPECT_EQ(state_type::Reset, 5U);
}

TEST(UnicycleLaw, FilterRejectsNonPositiveDt) {
    PoseVelocityFilter filter;
    ControllerConfig config;
    EXPECT_FALSE(updatePoseVelocityFilter(filter, 1.0, 0.0, config.filter_wn, config.filter_zeta,
                                          config.velocity_dt_min, config.velocity_dt_max));
    EXPECT_FALSE(updatePoseVelocityFilter(filter, 1.0, 1.0e-6, config.filter_wn, config.filter_zeta,
                                          config.velocity_dt_min, config.velocity_dt_max));
}

TEST(UnicycleLaw, FilterUsesThisSampleDtNotFixedPeriod) {
    ControllerConfig config;
    PoseVelocityFilter fast;
    ASSERT_FALSE(updatePoseVelocityFilter(fast, 0.0, 0.01, config.filter_wn, config.filter_zeta,
                                          config.velocity_dt_min, config.velocity_dt_max));
    ASSERT_TRUE(updatePoseVelocityFilter(fast, 0.1, 0.01, config.filter_wn, config.filter_zeta,
                                         config.velocity_dt_min, config.velocity_dt_max));
    PoseVelocityFilter slow;
    updatePoseVelocityFilter(slow, 0.0, 0.05, config.filter_wn, config.filter_zeta,
                             config.velocity_dt_min, config.velocity_dt_max);
    updatePoseVelocityFilter(slow, 0.1, 0.05, config.filter_wn, config.filter_zeta,
                             config.velocity_dt_min, config.velocity_dt_max);
    EXPECT_GT(std::fabs(fast.x2 - slow.x2), 1.0e-6);
}

TEST(UnicycleLaw, ChassisBoxIsSeventyPercent) {
    double v = 2.0;
    double w = -2.0;
    boxSaturateUnicycle(v, w, 1.05, 1.05);
    EXPECT_NEAR(v, 1.05, 1.0e-12);
    EXPECT_NEAR(w, -1.05, 1.0e-12);
}

TEST(UnicycleLaw, FlatnessUsesWorldVelocityPd) {
    UgvState state;
    state.x = 0.0;
    state.y = 0.0;
    state.yaw = 0.0;
    state.vx = 0.0;
    state.vy = 0.0;
    state.speed = 0.2;
    state.velocity_valid = true;
    WorldPvaReference reference;
    reference.valid = true;
    reference.x = 1.0;
    reference.vx = 0.5;
    ControllerConfig config;
    const FlatnessCommandOutput output =
        computeFlatnessCommand(state, reference, 0.2, 0.02, config);
    ASSERT_TRUE(output.valid);
    EXPECT_GT(output.accel, 0.0);
    EXPECT_GT(output.linear_speed, 0.2);
}

TEST(UnicycleLaw, FlatnessYawIsContinuousAcrossZeroSpeed) {
    UgvState state;
    state.velocity_valid = true;
    WorldPvaReference reference;
    reference.valid = true;
    reference.y = 0.01;
    reference.ay = 0.2;
    ControllerConfig config;
    for (double speed : {-1e-6, 0.0, 1e-6}) {
        const auto output = computeFlatnessCommand(state, reference, speed, 0.004, config);
        ASSERT_TRUE(output.valid);
        EXPECT_LT(std::fabs(output.angular_speed), 1e-4);
    }
}

TEST(UnicycleSm, FlatnessRetainsCommandWhenClockDoesNotAdvance) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    auto cfg = controller.config();
    cfg.tracking_strategy = TrackingStrategy::FLATNESS;
    controller.setConfig(cfg);
    goReadyPose(controller, state, 1.0);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    WorldPvaReference reference;
    reference.stamp = ros::Time(1.02);
    reference.x = 0.1;
    reference.vx = 0.3;
    reference.valid = true;
    controller.setWorldPva(reference);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.02);
    controller.update(1.02);
    setPose(state, 1.04, 0.0, 0.0, 0.0);
    controller.update(1.04);
    ASSERT_TRUE(controller.command().valid);
    const double speed = controller.command().linear_speed;
    ASSERT_GT(speed, 0.0);
    controller.update(1.04);
    EXPECT_FALSE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
    EXPECT_TRUE(controller.command().valid);
    EXPECT_DOUBLE_EQ(controller.command().linear_speed, speed);
}

TEST(UnicycleLaw, FlatnessRejectsInvalidDt) {
    UgvState state;
    state.velocity_valid = true;
    WorldPvaReference reference;
    reference.valid = true;
    reference.x = 1.0;
    EXPECT_FALSE(computeFlatnessCommand(state, reference, 0.2, 0.0, ControllerConfig{}).valid);
}

TEST(UnicycleLaw, PvaDoesNotValidateAlgorithmTimestamp) {
    WorldPvaReference reference;
    reference.valid = true;
    reference.stamp = ros::Time(1.1);
    reference.x = 2.0;
    reference.vx = 0.5;
    ASSERT_TRUE(worldPvaReady(reference));
    const WorldPvaReference held = liftWorldPva(reference, 1.0);
    ASSERT_TRUE(held.valid);
    EXPECT_DOUBLE_EQ(held.x, reference.x);
    EXPECT_DOUBLE_EQ(held.vx, reference.vx);

    reference.stamp = ros::Time(1001.0);
    EXPECT_TRUE(worldPvaReady(reference));
    EXPECT_TRUE(liftWorldPva(reference, 1.0).valid);
}

TEST(UnicycleUgvControllerRuntime, StateFreshRejectsExcessivelyFutureStamp) {
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.04);
    EXPECT_TRUE(stateFresh(state, ros::Time(1.0), 0.2));
    state.stamp = ros::Time(1.051);
    EXPECT_FALSE(stateFresh(state, ros::Time(1.0), 0.2));
}

TEST(UnicycleUgvControllerRuntime, VrpnQuaternionValidationRejectsZeroAndNonFiniteValues) {
    double yaw = 0.0;
    EXPECT_TRUE(tryYawFromQuaternion(0.0, 0.0, 0.0, 1.0, yaw));
    EXPECT_NEAR(yaw, 0.0, 1.0e-12);
    EXPECT_FALSE(tryYawFromQuaternion(0.0, 0.0, 0.0, 0.0, yaw));
    EXPECT_FALSE(
        tryYawFromQuaternion(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 1.0, yaw));
}

TEST(UnicycleUgvControllerRuntime, ProjectsRigidEstimateToForwardSpeed) {
    rigid_state_estimator_msgs::RigidStateEstimate msg;
    msg.position.x = 1.0;
    msg.position.y = -2.0;
    msg.orientation.w = 1.0;
    msg.velocity.x = 0.3;
    msg.velocity.y = 0.4;
    msg.angular_velocity.z = -0.1;
    const UnicycleProjection planar = projectRigidToUnicycle(msg);
    EXPECT_NEAR(planar.x, 1.0, 1.0e-12);
    EXPECT_NEAR(planar.y, -2.0, 1.0e-12);
    EXPECT_NEAR(planar.yaw, 0.0, 1.0e-12);
    EXPECT_NEAR(planar.speed, 0.3, 1.0e-12);
    EXPECT_NEAR(planar.yaw_rate, -0.1, 1.0e-12);
}

TEST(UnicycleUgvControllerRuntime, AutoStartCustom1WhenStateAndReferenceAreReady) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.auto_start_tracking = true;
    controller.setConfig(config);
    unicycle_reference_trajectory_msgs::AnalyticReference reference;
    reference.trajectory_id = 1U;
    reference.revision = 1U;
    reference.analytic_type = unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_HOLD;
    reference.start_time = ros::Time(1.0);
    reference.duration = 10.0;
    reference.origin.orientation.w = 1.0;
    ASSERT_TRUE(controller.referenceCache().updateAnalytic(reference));
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.estimator_state = rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
    controller.update(1.0);
    controller.update(1.01);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
}

TEST(UnicycleUgvControllerRuntime, TransientNmpcFailureKeepsFreshCommand) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.result_timeout = 0.1;
    controller.setConfig(config);
    makeCustom1Ready(controller, state);
    ControlCommand command;
    command.stamp = ros::Time(1.02);
    command.linear_speed = 0.8;
    command.angular_speed = -0.2;
    command.valid = true;
    controller.setCommand(command);
    ::state_machine::Event failure(event_type::INPUT_NMPC_SOLVE_FAILED,
                                   ::state_machine::EventTimestamp{1.05});
    failure.source = "test";
    failure.correlation_id = 1U;
    ASSERT_TRUE(controller.postEvent(std::move(failure)).ok());
    controller.update(1.05);
    EXPECT_TRUE(controller.commandReady());
    EXPECT_TRUE(controller.command().valid);
    EXPECT_FALSE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
}

TEST(UnicycleUgvControllerRuntime, SustainedNmpcFailureInvalidatesStaleCommandAndPublishesZero) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.result_timeout = 0.1;
    controller.setConfig(config);
    makeCustom1Ready(controller, state);
    ControlCommand command;
    command.stamp = ros::Time(1.02);
    command.linear_speed = 0.8;
    command.angular_speed = -0.2;
    command.valid = true;
    controller.setCommand(command);
    ::state_machine::Event failure(event_type::INPUT_NMPC_SOLVE_FAILED,
                                   ::state_machine::EventTimestamp{1.13});
    failure.source = "test";
    failure.correlation_id = 1U;
    ASSERT_TRUE(controller.postEvent(std::move(failure)).ok());
    controller.update(1.13);
    EXPECT_FALSE(controller.commandReady());
    EXPECT_FALSE(controller.command().valid);
    EXPECT_TRUE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
}

TEST(UnicycleUgvControllerRuntime, SampledNmpcHorizonKeepsYawContinuousAcrossPi) {
    ros::Time::init();
    ReferenceCache cache;
    unicycle_reference_trajectory_msgs::AnalyticReference reference;
    reference.trajectory_id = 1U;
    reference.revision = 1U;
    reference.analytic_type =
        unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_CIRCLE;
    reference.start_time = ros::Time(0.0);
    reference.duration = 30.0;
    reference.origin.orientation.w = 1.0;
    reference.params = {3.0, 1.0, 0.0, 0.0, 0.0};
    ASSERT_TRUE(cache.updateAnalytic(reference));
    std::vector<xgc2_math::control::Se2Reference> refs;
    ASSERT_TRUE(cache.sampleHorizon(ros::Time(4.5), 0.1, 10, refs));
    ASSERT_EQ(refs.size(), 11U);
    for (size_t i = 1; i < refs.size(); ++i) {
        EXPECT_LT(std::abs(refs[i].state.yaw - refs[i - 1].state.yaw), 0.1);
        EXPECT_NEAR(refs[i].state.yaw - refs[i - 1].state.yaw, 1.0 / 30.0, 1.0e-6);
    }
}

TEST(UnicycleUgvControllerRuntime, ExplicitSampledPlanarKinematicsPreservesReverseSpeedBranch) {
    ros::Time::init();
    ReferenceCache cache;
    unicycle_reference_trajectory_msgs::SampledReference reference;
    reference.trajectory_id = 9U;
    reference.revision = 1U;
    reference.flags =
        unicycle_reference_trajectory_msgs::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS;
    reference.start_time = ros::Time(1.0);
    reference.sample_dt = 0.5;
    for (int index = 0; index < 3; ++index) {
        unicycle_reference_trajectory_msgs::PlanarReferencePoint point;
        point.t_from_start = 0.5 * static_cast<double>(index);
        point.x = -0.1 * static_cast<double>(index);
        point.y = 0.0;
        point.yaw = 0.0;
        point.speed = -0.2;
        point.vx = -0.2;
        point.vy = 0.0;
        reference.points.push_back(point);
    }
    ASSERT_TRUE(cache.updateSampled(reference));
    std::vector<xgc2_math::control::Se2Reference> refs;
    ASSERT_TRUE(cache.sampleHorizon(ros::Time(1.0), 0.25, 4, refs));
    ASSERT_EQ(refs.size(), 5U);
    for (const auto& ref : refs) {
        EXPECT_NEAR(ref.state.yaw, 0.0, 1.0e-12);
        EXPECT_NEAR(ref.state.linear_speed, -0.2, 1.0e-12);
    }
}

TEST(UnicycleUgvControllerRuntime, NmpcSolverUsesConfiguredPhysicalBounds) {
    UnicycleNmpcSolver solver;
    ASSERT_TRUE(solver.configureBounds(-1.5, 1.5, 2.0, 0.5235, 3.0));
    ASSERT_TRUE(solver.initialize());
    NmpcStateVector x0 = NmpcStateVector::Zero();
    std::vector<Se2Reference> refs(static_cast<size_t>(UnicycleNmpcSolver::horizonSteps() + 1));
    for (auto& ref : refs) {
        ref.state.position << 3.0, 3.0;
        ref.state.yaw = 1.5;
        ref.state.linear_speed = 3.0;
        ref.control.linear_acceleration = 2.0;
        ref.control.yaw_rate = 2.5;
    }
    ASSERT_TRUE(solver.solve(x0, refs));
    EXPECT_LE(std::abs(solver.predictedAngularSpeed()), 0.5235 + 1.0e-6);
    EXPECT_LE(std::abs(solver.optimalControl()(1)), 3.0 + 1.0e-6);
}

TEST(UnicycleUgvControllerRuntime, NmpcSolverRejectsInvalidBounds) {
    UnicycleNmpcSolver solver;
    EXPECT_FALSE(solver.configureBounds(1.5, 1.5, 2.0, 0.5235, 3.0));
    EXPECT_FALSE(solver.configureBounds(-1.5, 1.5, 0.0, 0.5235, 3.0));
    EXPECT_FALSE(solver.configureBounds(-1.5, 1.5, 2.0, 0.0, 3.0));
    EXPECT_FALSE(solver.configureBounds(-1.5, 1.5, 2.0, 0.5235, 0.0));
}

TEST(UnicycleUgvControllerRuntime, NmpcSolverLimitsOneStageYawRateChange) {
    constexpr double kMaxAngularAcceleration = 0.6;
    constexpr double kStageDt = 0.1;
    UnicycleNmpcSolver solver;
    ASSERT_TRUE(solver.configureBounds(-1.5, 1.5, 2.0, 1.5, kMaxAngularAcceleration));
    ASSERT_TRUE(solver.initialize());
    NmpcStateVector x0 = NmpcStateVector::Zero();
    std::vector<Se2Reference> refs(static_cast<size_t>(UnicycleNmpcSolver::horizonSteps() + 1));
    for (auto& ref : refs) {
        ref.state.position << 0.0, 3.0;
        ref.state.yaw = 1.5707963267948966;
        ref.state.linear_speed = 1.0;
        ref.control.yaw_rate = 1.0;
    }
    ASSERT_TRUE(solver.solve(x0, refs));
    EXPECT_LE(std::abs(solver.optimalControl()(1)), kMaxAngularAcceleration + 1.0e-6);
    EXPECT_LE(std::abs(solver.predictedAngularSpeed() - x0(4)),
              kMaxAngularAcceleration * kStageDt + 1.0e-5);
}

TEST(UnicycleUgvControllerRuntime, NmpcSolverHeavierOmegaWeightDampsExistingRailYawRate) {
    const double rail_yaw = 1.5707963267948966;
    auto make_refs = [rail_yaw]() {
        std::vector<Se2Reference> refs(static_cast<size_t>(UnicycleNmpcSolver::horizonSteps() + 1));
        for (int i = 0; i <= UnicycleNmpcSolver::horizonSteps(); ++i) {
            refs[static_cast<size_t>(i)].state.position << -13.0, 0.05 * static_cast<double>(i);
            refs[static_cast<size_t>(i)].state.yaw = rail_yaw;
            refs[static_cast<size_t>(i)].state.linear_speed = 0.5;
        }
        return refs;
    };
    NmpcStateVector x0;
    x0 << -12.97, 0.0, 1.610, 0.5, 0.5;
    NmpcCostWeights cheap;
    cheap.accel = 0.05;
    cheap.omega = 0.08;
    UnicycleNmpcSolver cheap_solver;
    ASSERT_TRUE(cheap_solver.configureBounds(-0.5, 0.5, 2.0, 0.8, 3.0));
    ASSERT_TRUE(cheap_solver.configureWeights(cheap));
    ASSERT_TRUE(cheap_solver.initialize());
    ASSERT_TRUE(cheap_solver.solve(x0, make_refs()));
    const double w_cheap = cheap_solver.predictedAngularSpeed();
    UnicycleNmpcSolver heavy_solver;
    ASSERT_TRUE(heavy_solver.configureBounds(-0.5, 0.5, 2.0, 0.8, 3.0));
    ASSERT_TRUE(heavy_solver.configureWeights(NmpcCostWeights{}));
    ASSERT_TRUE(heavy_solver.initialize());
    ASSERT_TRUE(heavy_solver.solve(x0, make_refs()));
    const double w_heavy = heavy_solver.predictedAngularSpeed();
    EXPECT_LT(std::abs(w_heavy), 0.35);
    EXPECT_LT(std::abs(w_heavy), std::abs(w_cheap));
}

TEST(UnicycleUgvControllerRuntime, NmpcSolverRejectsInvalidWeights) {
    UnicycleNmpcSolver solver;
    NmpcCostWeights weights;
    weights.omega = 0.0;
    EXPECT_FALSE(solver.configureWeights(weights));
    weights = NmpcCostWeights{};
    weights.angular_accel = 0.0;
    EXPECT_FALSE(solver.configureWeights(weights));
}

}  // namespace

TEST(UnicycleSm, ResetHasNoUnfilteredFallbackAndCancelsLateResponses) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
    ResetTarget goal;
    goal.x = 1.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    controller.update(1.012);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    EXPECT_DOUBLE_EQ(controller.command().linear_speed, 0.0);
    auto& session = controller.resetSession();
    const double wall = ugv_reset_safety::monotonicSeconds();
    const auto issued = session.issue({0.0, 0.0, 0.0}, ros::Time(1.012).toNSec(), wall);
    ASSERT_TRUE(issued.valid);
    ASSERT_TRUE(
        session.accept(issued.generation, issued.stamp, 0, {0.1, 0.0, 0.0}, issued.stamp, wall));
    controller.update(1.014);
    EXPECT_DOUBLE_EQ(controller.command().linear_speed, 0.1);
    postCommand(controller, event_type::STOP_REQUESTED, 1.016);
    controller.update(1.016);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
    EXPECT_FALSE(session.active());
    EXPECT_FALSE(
        session.accept(issued.generation, issued.stamp, 0, {0.1, 0.0, 0.0}, issued.stamp, wall));
    EXPECT_DOUBLE_EQ(controller.command().linear_speed, 0.0);
    postCommand(controller, event_type::RESET_REQUESTED, 1.018);
    controller.update(1.018);
    EXPECT_GT(session.generation(), issued.generation);
    EXPECT_FALSE(
        session.accept(issued.generation, issued.stamp, 0, {0.1, 0.0, 0.0}, issued.stamp, wall));
}

TEST(UnicycleSm, ResetRequiresCoordinatorArrivalEvenAtTarget) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
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

TEST(UnicycleSm, ResetRejectionReturnsReadyAndAcceptsFreshRetry) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
    ResetTarget goal;
    goal.x = 1.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    controller.update(1.012);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    auto& session = controller.resetSession();
    const double wall = ugv_reset_safety::monotonicSeconds();
    const auto rejected = session.issue({0.0, 0.0, 0.0}, ros::Time(1.012).toNSec(), wall);
    ASSERT_TRUE(rejected.valid);
    ASSERT_TRUE(session.accept(rejected.generation, rejected.stamp,
                               ugv_reset_safety::ResetSession::REJECTED, {}, rejected.stamp, wall));
    setPose(state, 1.014, 0.0, 0.0, 0.0);
    controller.update(1.014);
    controller.update(1.016);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
    EXPECT_TRUE(controller.healthReady());
    EXPECT_FALSE(session.active());
    EXPECT_DOUBLE_EQ(controller.command().linear_speed, 0.0);
    EXPECT_DOUBLE_EQ(controller.command().angular_speed, 0.0);

    // Health remains ready throughout: retry must not need a new HEALTH_READY edge.
    setPose(state, 1.018, 0.0, 0.0, 0.0);
    controller.update(1.018);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
    postCommand(controller, event_type::RESET_REQUESTED, 1.02);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    controller.update(1.022);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    ASSERT_TRUE(session.active());
    EXPECT_GT(session.generation(), rejected.generation);
    const auto retry = session.issue({0.0, 0.0, 0.0}, ros::Time(1.022).toNSec(), wall);
    ASSERT_TRUE(retry.valid);
    EXPECT_FALSE(session.accept(rejected.generation, rejected.stamp,
                                ugv_reset_safety::ResetSession::REJECTED, {}, retry.stamp, wall));
    ASSERT_TRUE(session.accept(retry.generation, retry.stamp,
                               ugv_reset_safety::ResetSession::RUNNING, {-0.1, 0.0, 0.0},
                               retry.stamp, wall));
    controller.update(1.024);
    EXPECT_DOUBLE_EQ(controller.command().linear_speed, -0.1);
}
}  // namespace unicycle_ugv_controller
