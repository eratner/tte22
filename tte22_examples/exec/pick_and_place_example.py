#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Pose


class PickAndPlaceTask:
    def __init__(self):
        self._package_pose_sub = rospy.Subscriber(
            '/package_pose', Pose, self.package_pose_callback)
        self._package_pose = None

    def run(self):
        state = self.waiting_for_package_pose

        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            state = state()
            rate.sleep()

    def waiting_for_package_pose(self):
        rospy.loginfo("[PickAndPlaceTask] Waiting for package pose...")
        if self._package_pose is None:
            return self.waiting_for_package_pose
        else:
            # We've got a package pose, so let's go to the next step
            return self.move_to_grasping_pose

    def move_to_grasping_pose(self):
        rospy.loginfo("[PickAndPlaceTask] Moving to grasping pose...")
        # 1) Compute desired EE pose to grasp the object, based on the package pose
        # 2) Compute desired joint angles to get the arm to that pose using IK
        # 3) Execute a path to move the arm to the desired joint angles
        return self.waiting_for_place_pose

    def waiting_for_place_pose(self):
        rospy.loginfo("[PickAndPlaceTask] Waiting for place pose...")
        # ...
        return self.waiting_for_place_pose

    def package_pose_callback(self, msg):
        self._package_pose = msg



def main():
    rospy.init_node('pick_and_place_example')

    task = PickAndPlaceTask()
    task.run()


if __name__ == '__main__':
    main()
