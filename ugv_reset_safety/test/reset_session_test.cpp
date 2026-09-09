#include <gtest/gtest.h>
#include <ugv_reset_safety/reset_session.h>

#include <cmath>

using ugv_reset_safety::ResetSession;

TEST(ResetSessionContract, OwnershipReplayAndDualClockLease) {
    ResetSession session;
    constexpr uint64_t t = 1000000000ULL;
    ASSERT_TRUE(!session.issue({}, t, 1.0).valid);
    session.begin({2.0, 3.0, 0.5});
    session.noteApplied({}, t);
    auto request = session.issue({}, t, 1.0);
    ASSERT_TRUE(request.valid && request.target.x == 2.0 && request.target.y == 3.0);
    ASSERT_TRUE(request.applied_stamp == t && request.applied_command.x == 0.0);
    ASSERT_TRUE(!session.feedback(t, 1.0).valid);
    ASSERT_TRUE(!session.accept(request.generation + 1, t, 0, {0.1, 0, 0}, t, 1.0));
    ASSERT_TRUE(!session.accept(request.generation, t + 1, 0, {0.1, 0, 0}, t + 1, 1.0));
    ASSERT_TRUE(!session.accept(request.generation, t, 3, {}, t, 1.0));
    ASSERT_TRUE(!session.accept(request.generation, t, 0, {NAN, 0, 0}, t, 1.0));
    ASSERT_TRUE(!session.accept(request.generation, t, 1, {0.1, 0, 0}, t, 1.0));
    ASSERT_TRUE(session.accept(request.generation, t, 0, {0.1, 0, 0.2}, t, 1.01));
    ASSERT_TRUE(session.feedback(t + 10000000, 1.02).valid);
    // Replays cannot renew the lease. Both clocks independently expire it.
    ASSERT_TRUE(!session.accept(request.generation, t, 0, {0.1, 0, 0}, t, 1.02));
    ASSERT_TRUE(!session.feedback(t, 1.151).valid);
    ASSERT_TRUE(!session.feedback(t + 150000001, 1.02).valid);
    ASSERT_TRUE(!session.feedback(t - 1, 1.02).valid);
    ASSERT_TRUE(!session.issue({}, t, 1.2).valid);  // paused clock cannot renew
    ASSERT_TRUE(!session.issue({}, t - 1, 1.2).valid);

    auto newer = session.issue({}, t + 20000000, 1.03);
    ASSERT_TRUE(newer.valid);
    // A received proposal that has not been published is never acknowledged.
    ASSERT_TRUE(newer.applied_command.x == 0.0 && newer.applied_stamp == t);
    session.noteApplied({0.1, 0, 0.2}, t + 20000000);
    ASSERT_TRUE(!session.accept(newer.generation, newer.stamp, 0, {}, newer.stamp, 1.181));
    ASSERT_TRUE(session.accept(newer.generation, newer.stamp, 1, {}, newer.stamp, 1.04));
    ASSERT_TRUE(session.feedback(newer.stamp, 1.04).status == ResetSession::ARRIVED);

    session.cancel();
    ASSERT_TRUE(!session.feedback(newer.stamp, 1.04).valid);
    ASSERT_TRUE(!session.accept(newer.generation, newer.stamp, 0, {}, newer.stamp, 1.04));
    session.begin({4.0, 5.0, 0.0});
    ASSERT_TRUE(session.generation() > request.generation);
    ASSERT_TRUE(!session.accept(newer.generation, newer.stamp, 0, {}, newer.stamp, 1.04));
    const auto next = session.issue({}, t + 40000000, 1.05);
    ASSERT_TRUE(next.valid && next.target.x == 4.0);
    // Begin/cancel do not pretend that an un-emitted zero has been executed.
    ASSERT_TRUE(next.applied_stamp == t + 20000000 && next.applied_command.x == 0.1);
    ASSERT_TRUE(session.accept(next.generation, next.stamp, 2, {}, next.stamp, 1.05));
    ASSERT_TRUE(session.feedback(next.stamp, 1.05).status == ResetSession::REJECTED);
    session.begin({NAN, 0, 0});
    ASSERT_TRUE(!session.active());
}
