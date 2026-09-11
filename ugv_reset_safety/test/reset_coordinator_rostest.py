#!/usr/bin/env python3
"""Real ROS coordinator + native owner; parameterized synthetic vehicle plant.

This proves command routing/session leases and simple nominal convergence.
It makes no wheel-contact, braking, Gazebo, or physical-robot claim.
"""

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point, Pose2D, PoseStamped, Twist
from std_msgs.msg import String
from ugv_reset_safety.msg import ResetRequest, ResetResponse
from xgc2_geometry_msgs.msg import SceneSnapshot, SceneState, SceneObstacle, ScenePart, SceneObstacleState


class ResetCoordinatorTransportTest(unittest.TestCase):
    NS = "/reset_fixture_robot"

    def setUp(self):
        self.robot_type = rospy.get_param("~robot_type", "mecanum")
        self.assertIn(self.robot_type, ("mecanum", "scout"))
        self.scout = self.robot_type == "scout"
        self.owner_type = "unicycle_ugv_controller" if self.scout else "mecanum_ugv_controller"
        self.max_vx = 0.35 if self.scout else 0.4
        self.max_vy = 0.0 if self.scout else 0.4
        self.lock = threading.RLock()
        self.pose = [-0.6, 0.25, 0.35]
        self.command = [0.0, 0.0, 0.0]
        self.actual = [0.0, 0.0, 0.0]
        self.state = ""
        self.publish_pose = True
        self.deliver = True
        self.requests = []
        self.responses = []
        self.commands = []
        self.quit = threading.Event()
        self.pose_pub = rospy.Publisher(self.NS + "/pose", PoseStamped, queue_size=1)
        self.command_pub = rospy.Publisher(self.NS + "/command", String, queue_size=1)
        self.goal_pub = rospy.Publisher(self.NS + "/reset_pose", Pose2D, queue_size=1, latch=True)
        self.reply_pub = rospy.Publisher("/reset_fixture/controller_response", ResetResponse, queue_size=1)
        self.snapshot = None
        self.scene_pub = rospy.Publisher("/reset_fixture/scene/snapshot", SceneSnapshot, queue_size=1, latch=True)
        self.scene_state_pub = rospy.Publisher("/reset_fixture/scene/state", SceneState, queue_size=1)
        self.scene_timer = rospy.Timer(rospy.Duration(.03), self.publish_scene_state)
        self.subscribers = [
            rospy.Subscriber(self.NS + "/cmd_vel", Twist, self.on_command, queue_size=1),
            rospy.Subscriber(self.NS + "/reset/request", ResetRequest, self.on_request, queue_size=1),
            rospy.Subscriber(self.NS + "/reset/response", ResetResponse, self.on_response, queue_size=1),
            rospy.Subscriber(self.NS + "/custom/statustext", String,
                             self.on_state, queue_size=1),
        ]
        self.thread = threading.Thread(target=self.plant, daemon=True)
        self.thread.start()
        self.wait(lambda: self.state == "Ready" and self.goal_pub.get_num_connections() > 0 and
                  self.command_pub.get_num_connections() > 0 and self.reply_pub.get_num_connections() > 0,
                  5.0, "native owner must become Ready from canonical poses")
        self.goal_pub.publish(Pose2D(x=0.0, y=0.0, theta=0.0))
        time.sleep(0.15)

    def tearDown(self):
        self.command_pub.publish(String(data="stop"))
        self.scene_timer.shutdown()
        self.quit.set()
        self.thread.join(timeout=1.0)

    def wait(self, condition, seconds, message):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            with self.lock:
                if condition():
                    return
            time.sleep(0.01)
        with self.lock:
            self.assertTrue(condition(), message + "; state=" + str(self.state) +
                            "; pose=" + str(self.pose) + "; command=" + str(self.command) +
                            "; last responses=" + str([(r.generation, r.status, r.reason,
                              r.command.angular.z) for r in self.responses[-5:]]))

    def on_command(self, msg):
        with self.lock:
            self.command = [msg.linear.x, msg.linear.y, msg.angular.z]
            self.commands.append(tuple(self.command))

    def on_request(self, msg):
        with self.lock:
            self.requests.append(msg)

    def on_response(self, msg):
        with self.lock:
            self.responses.append(msg)
            deliver = self.deliver
        if deliver:
            self.reply_pub.publish(msg)

    def on_state(self, msg):
        with self.lock:
            self.state = msg.data

    def plant(self):
        previous = time.monotonic()
        while not self.quit.is_set() and not rospy.is_shutdown():
            now = time.monotonic()
            dt = min(now - previous, 0.03)
            previous = now
            with self.lock:
                x, y, yaw = self.pose
                vx, vy, omega = self.command
                if self.scout:
                    # Explicit tested surrogate, not a calibrated tire model:
                    # negative yaw-coupled lateral velocity and actuator lag.
                    self.actual[0] += (1.0 - math.exp(-dt / 0.12)) * (vx - self.actual[0])
                    self.actual[2] += (1.0 - math.exp(-dt / 0.16)) * (omega - self.actual[2])
                    self.actual[1] = -0.229 * self.actual[2]
                else:
                    self.actual = [vx, vy, omega]
                vx, vy, omega = self.actual
                midpoint_yaw = yaw + omega * dt / 2.0
                x += (math.cos(midpoint_yaw) * vx - math.sin(midpoint_yaw) * vy) * dt
                y += (math.sin(midpoint_yaw) * vx + math.cos(midpoint_yaw) * vy) * dt
                yaw = math.atan2(math.sin(yaw + omega * dt), math.cos(yaw + omega * dt))
                self.pose = [x, y, yaw]
                publish = self.publish_pose
            if publish:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "world"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                self.pose_pub.publish(pose)
            self.quit.wait(0.01)

    def reset(self):
        with self.lock:
            count = len(self.requests)
            previous_generation = self.requests[-1].generation if self.requests else None
        self.command_pub.publish(String(data="reset"))
        self.wait(lambda: len(self.requests) > count and
                  self.requests[-1].generation != previous_generation,
                  2.0, "Reset must emit a request for a new generation")
        with self.lock:
            return self.requests[-1].generation

    def moving(self):
        return max(abs(value) for value in self.command) > 0.015

    def stopped(self):
        return max(abs(value) for value in self.command) < 1e-9

    def settled(self):
        return self.stopped() and max(abs(value) for value in self.actual) < 0.01

    def publish_scene_state(self, _event):
        if self.snapshot is None:
            return
        state = SceneState(epoch=self.snapshot.epoch, revision=self.snapshot.revision)
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = "world"
        state.obstacles = [SceneObstacleState(id=o.id, pose=o.pose) for o in self.snapshot.obstacles]
        self.scene_state_pub.publish(state)

    def publish_scene(self, shape="cube", x=2.0):
        scene = SceneSnapshot(epoch="fixture", revision=(self.snapshot.revision + 1 if self.snapshot else 1))
        scene.header.stamp = rospy.Time.now()
        scene.header.frame_id = "world"
        obstacle = SceneObstacle(id="far-box", name="far fixture box", motion_type="hold")
        obstacle.pose.position.x = x
        obstacle.pose.orientation.w = 1
        part = ScenePart(id="body")
        part.pose.orientation.w = 1
        part.geometry.type = "box" if shape == "cube" else shape
        part.geometry.size.x = part.geometry.size.y = part.geometry.size.z = .2
        obstacle.parts = [part]
        scene.obstacles = [obstacle]
        self.snapshot = scene
        self.scene_pub.publish(scene)
        self.publish_scene_state(None)

    def replay_cannot_restart(self, response):
        for _ in range(6):
            self.reply_pub.publish(response)
            time.sleep(0.025)
            with self.lock:
                self.assertTrue(self.stopped(), "a replayed response restarted a stopped owner")

    def test_coordinator_and_native_owner_contract(self):
        # Absence of scene data is not an implicitly empty obstacle map.
        generation = self.reset()
        self.wait(lambda: any(r.generation == generation and r.status == ResetResponse.REJECTED
                              for r in self.responses) and self.state == "Ready",
                  3.0, "missing scene must reject Reset")
        with self.lock:
            self.assertTrue(self.stopped())
            self.assertTrue(all(max(abs(v) for v in c) == 0.0 for c in self.commands))

        self.publish_scene()
        time.sleep(0.2)

        # Both publisher hops rewrite Header.seq; the explicit generation must
        # survive and match the request, while the echoed stamp stays exact.
        generation = self.reset()
        self.wait(self.moving, 3.0, "valid coordinator response must reach native cmd_vel")
        self.wait(lambda: self.state == "Ready" and any(r.generation == generation and
                  r.status == ResetResponse.ARRIVED for r in self.responses),
                  45.0 if self.scout else 15.0,
                  "real coordinator and native owner should reach the simple target")
        with self.lock:
            known = {(r.generation, r.header.stamp.to_nsec()) for r in self.requests}
            accepted = [r for r in self.responses if r.generation == generation]
            self.assertTrue(accepted)
            self.assertTrue(all((r.generation, r.header.stamp.to_nsec()) in known for r in accepted))
            sent = [r for r in self.requests if r.generation == generation]
            self.assertTrue(all(0 < r.applied_stamp.to_nsec() <= r.header.stamp.to_nsec() for r in sent))
            self.assertEqual(sent[0].applied_command.linear.x, 0.0)
            self.assertEqual(sent[0].applied_command.linear.y, 0.0)
            self.assertEqual(sent[0].applied_command.angular.z, 0.0)
            self.assertTrue(any(abs(r.applied_command.linear.x) > 0.01 for r in sent),
                            "requests must acknowledge commands the native owner actually published")
            self.assertLessEqual(math.hypot(self.pose[0], self.pose[1]), 0.055)
            if not self.scout:
                self.assertLessEqual(abs(self.pose[2]), 0.12)
            self.assertTrue(self.stopped())
            self.assertTrue(all(all(math.isfinite(v) for v in c) for c in self.commands))
            self.assertTrue(all(abs(c[0]) <= self.max_vx + 1e-9 and abs(c[1]) <= self.max_vy + 1e-9 and
                                abs(c[2]) <= 0.5 + 1e-9 for c in self.commands))

        if not self.scout:
            # XY arrival must not suppress the Mecanum's independent heading
            # control. Keep the actual position and request only a yaw change.
            with self.lock:
                turn_x, turn_y, turn_yaw = self.pose
            turn_goal = turn_yaw + 1.0
            self.goal_pub.publish(Pose2D(x=turn_x, y=turn_y, theta=turn_goal))
            time.sleep(0.2)
            generation = self.reset()
            self.wait(lambda: self.command[2] > 0.015, 3.0,
                      "Mecanum at target XY must still turn toward its target yaw")
            self.wait(lambda: self.state == "Ready" and any(r.generation == generation and
                      r.status == ResetResponse.ARRIVED for r in self.responses),
                      12.0, "heading-only Mecanum Reset must arrive and stop")
            with self.lock:
                yaw_error = math.atan2(math.sin(turn_goal - self.pose[2]),
                                       math.cos(turn_goal - self.pose[2]))
                self.assertLessEqual(abs(yaw_error), 0.055)
                self.assertLessEqual(math.hypot(self.pose[0] - turn_x, self.pose[1] - turn_y), 0.055)
                self.assertTrue(self.stopped())

        # Unsupported scene geometry and a valid geometric revision must both
        # terminate the active generation, including delayed RUNNING replays.
        self.goal_pub.publish(Pose2D(x=-0.8, y=0.0, theta=0.0))
        time.sleep(0.2)
        for shape, x, expected in (("unsupported_fixture_shape", 2.0, "scene unavailable"),
                                   ("cube", 2.2, "scene changed")):
            self.wait(self.settled, 1.5, "plant must settle before a fresh reset")
            generation = self.reset()
            self.wait(self.moving, 3.0, "new scene test generation must first execute")
            with self.lock:
                previous_running = next(r for r in reversed(self.responses)
                                        if r.generation == generation and r.status == ResetResponse.RUNNING)
            self.publish_scene(shape=shape, x=x)
            self.wait(lambda: self.state == "Ready" and self.stopped() and
                      any(r.generation == generation and r.status == ResetResponse.REJECTED and
                          expected in r.reason for r in self.responses),
                      2.0, "invalid or changed scene must reject active reset")
            self.replay_cannot_restart(previous_running)
            self.publish_scene(x=x)
            time.sleep(0.2)

        # Start a separate target from a stationary pose, then drop only the
        # coordinator-to-owner link. Its previous command lease must expire.
        self.goal_pub.publish(Pose2D(x=-0.8, y=0.0, theta=0.0))
        time.sleep(0.2)
        self.wait(self.settled, 1.5, "plant must settle after scene rejection")
        generation = self.reset()
        self.wait(self.moving, 3.0, "second Reset should move before response drop")
        with self.lock:
            late = next(r for r in reversed(self.responses) if r.generation == generation and
                        r.status == ResetResponse.RUNNING)
            self.deliver = False
        self.wait(self.stopped, 0.4, "expired safety response must stop native output")
        self.wait(lambda: self.requests[-1].generation == generation and
                  self.requests[-1].applied_command.linear.x == 0.0 and
                  self.requests[-1].applied_command.linear.y == 0.0 and
                  self.requests[-1].applied_command.angular.z == 0.0,
                  0.2, "expired response must acknowledge the emitted zero, not a pending proposal")
        self.command_pub.publish(String(data="stop"))
        self.wait(lambda: self.state == "Ready", 2.0, "Stop must cancel Reset")
        self.replay_cannot_restart(late)

        # A new generation may run; loss of canonical pose then fails closed.
        with self.lock:
            self.deliver = True
        self.wait(self.settled, 1.5, "plant must settle after command lease expiration")
        next_generation = self.reset()
        self.assertNotEqual(next_generation, generation)
        self.wait(self.moving, 3.0, "new Reset generation should run after a measured stop")
        with self.lock:
            self.publish_pose = False
        self.wait(lambda: self.stopped() and self.state == "Reset", 0.6,
                  "loss of canonical pose must stop and leave Reset")


if __name__ == "__main__":
    rospy.init_node("reset_coordinator_kinematic_transport_test")
    rostest.rosrun("ugv_reset_safety", "reset_coordinator_kinematic_transport", ResetCoordinatorTransportTest)
