#!/usr/bin/env python

# NOTE: Must run `chmod +x ur5_example.py` in order to make executable before
# running via `rosrun tte22_examples ur5_example.py`
# To run this:
#  1) `roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.56.101` (NOTE: may need different IP)
#  2) (OPTIONAL) `rosrun rviz rviz`
#  3) `rosrun tte22_examples ur5_example.py`

import rospy
import sys
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    rospy.init_node('ur5_example')

    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ]

    # Create a client to interface with the UR5's joint trajectory action server.
    # See http://wiki.ros.org/actionlib for more info
    joint_traj_client = actionlib.SimpleActionClient(
        "scaled_pos_joint_traj_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction)

    if not joint_traj_client.wait_for_server(timeout=rospy.Duration(2.0)):
        rospy.logerr("Failed to connect to action server!")
        sys.exit(-1)

    ctrl_msg = FollowJointTrajectoryGoal()

    goal_point = JointTrajectoryPoint()
    goal_point.positions = [0, 0, 0, 0, 0, 0] # NOTE: Change this to the desired joint angles!
    goal_point.time_from_start = rospy.Duration(5.0) # Time in seconds.

    ctrl_msg.trajectory.points = [goal_point]
    ctrl_msg.trajectory.joint_names = joint_names
    ctrl_msg.path_tolerance = []
    ctrl_msg.goal_tolerance = []
    for name in joint_names:
        tol = JointTolerance()
        tol.name = name
        tol.position = 0
        tol.velocity = 0
        tol.acceleration = 0
        ctrl_msg.path_tolerance.append(tol)
        ctrl_msg.goal_tolerance.append(tol)

    joint_traj_client.send_goal(ctrl_msg)

    rospy.spin()


if __name__ == '__main__':
    main()

