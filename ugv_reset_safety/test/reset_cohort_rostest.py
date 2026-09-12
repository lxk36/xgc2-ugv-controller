#!/usr/bin/env python3
"""Exercise repeated fleet admission through ROS on the isolated rostest master."""
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ugv_reset_safety.msg import ResetRequest, ResetResponse
from xgc2_geometry_msgs.msg import SceneSnapshot, SceneState


class ResetCohortTest(unittest.TestCase):
    def test_repeated_staggered_generations(self):
        lock = threading.RLock()
        poses = [(0., 0.), (2., 0.), (2., 2.), (0., 2.)]
        # Each requested target starts under a different fleet member.
        goals = poses[1:] + poses[:1]
        generations = [0] * 4
        enabled = [False] * 4
        responses = []
        last_state = [0.0]
        pubs = []
        subs = []
        for i in range(4):
            ns = '/ugv' + str(i + 1)
            pubs.append((rospy.Publisher(ns + '/pose', PoseStamped, queue_size=1),
                         rospy.Publisher(ns + '/custom/statustext', String, queue_size=1),
                         rospy.Publisher(ns + '/reset/request', ResetRequest, queue_size=1)))
            def receive(msg, index=i):
                with lock:
                    responses.append((index, msg.generation, msg.status, msg.reason))
            subs.append(rospy.Subscriber(ns + '/reset/response', ResetResponse, receive))
        snapshot_pub = rospy.Publisher('/cohort_scene/snapshot', SceneSnapshot, queue_size=1, latch=True)
        scene_pub = rospy.Publisher('/cohort_scene/state', SceneState, queue_size=1)
        snapshot = SceneSnapshot(epoch='cohort', revision=1)
        snapshot.header.frame_id = 'world'
        snapshot.header.stamp = rospy.Time.now()
        snapshot_pub.publish(snapshot)

        def publish(_event):
            now = rospy.Time.now()
            scene = SceneState(epoch='cohort', revision=1)
            scene.header.frame_id = 'world'
            scene.header.stamp = now
            scene_pub.publish(scene)
            with lock:
                publish_state = time.monotonic() - last_state[0] >= .2
                if publish_state:
                    last_state[0] = time.monotonic()
                for i, (pose_pub, state_pub, request_pub) in enumerate(pubs):
                    pose = PoseStamped()
                    pose.header.frame_id = 'world'
                    pose.header.stamp = now
                    pose.pose.position.x, pose.pose.position.y = poses[i]
                    pose.pose.orientation.w = 1.
                    pose_pub.publish(pose)
                    if publish_state:
                        state_pub.publish(String(data='Reset' if enabled[i] else 'Ready'))
                    if not enabled[i]:
                        continue
                    request = ResetRequest()
                    request.header = pose.header
                    request.generation = generations[i]
                    request.pose_stamp = now
                    request.applied_stamp = now
                    request.pose.x, request.pose.y = poses[i]
                    request.target.x, request.target.y = goals[i]
                    request_pub.publish(request)
        timer = rospy.Timer(rospy.Duration(.01), publish)
        try:
            # Publishing before TCPROS has connected tests startup loss, not
            # staggered admission. Establish every transport before commands.
            connected_deadline = time.monotonic() + 5.0
            while (not all(pub.get_num_connections() for group in pubs for pub in group)
                   and time.monotonic() < connected_deadline):
                time.sleep(.01)
            self.assertTrue(all(pub.get_num_connections() for group in pubs for pub in group))
            time.sleep(.5)
            for generation, count in ((1, 4), (2, 3), (3, 4), (4, 4)):
                with lock:
                    enabled[:] = [False] * 4
                time.sleep(.3)
                for i in range(count):
                    with lock:
                        generations[i] = generation
                        enabled[i] = True
                    # Transport joins cannot be assumed simultaneous.
                    time.sleep(.06)
                time.sleep(.4)
                with lock:
                    observed = [r for r in responses if r[1] == generation]
                self.assertTrue(observed)
                if count == 3:
                    self.assertTrue(any(r[2] == ResetResponse.REJECTED and
                                        'stationary nonparticipant' in r[3] for r in observed),
                                    'a real nonparticipant must still block an occupied goal: ' + str(observed))
                else:
                    self.assertFalse(any(r[2] == ResetResponse.REJECTED for r in observed),
                                     'prior requests/rejections must not poison the new fleet: ' + str(observed))
                    self.assertEqual({r[0] for r in observed if r[2] == ResetResponse.RUNNING and not r[3]},
                                     set(range(4)), 'all joined owners must enter DWA')
        finally:
            timer.shutdown()
            for sub in subs:
                sub.unregister()


if __name__ == '__main__':
    rospy.init_node('reset_cohort_test')
    rostest.rosrun('ugv_reset_safety', 'reset_cohort', ResetCohortTest)
