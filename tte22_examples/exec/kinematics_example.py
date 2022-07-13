#!/usr/bin/env python

# NOTE: Must run `chmod +x kinematics_example.py` in order to make executable before
# running via `rosrun tte22_examples kinematics_example.py`

import rospy
import sys
import kdl_parser_py.urdf # sudo apt-get install ros-kinetic-kdl-parser-py
from PyKDL import *
from urdf_parser_py.urdf import URDF


def main():
    rospy.init_node('kinematics_example')

    # Load the robot's kinematics from a URDF on the ROS parameter server.
    tree = None
    try:
        ok, tree = kdl_parser_py.urdf.treeFromParam('robot_description')
        if ok:
            rospy.loginfo('Ok! Parsed robot with {} joints and {} links'.
                          format(
                              tree.getNrOfJoints(),
                              tree.getNrOfSegments())) # NOTE: `links` are sometimes called `segments`
            chain = tree.getChain(
                'base_link', # Root of the kinematic chain (`base_link`)
                'ee_link')   # Tip of the kinematic chain (`ee_link`)
        else:
            rospy.logerr('Not ok! Failed to load robot from URDF')
            sys.exit(-1)
    except Exception as err:
        rospy.logerr('Error: {}'.format(err))
        sys.exit(-1)

    # Construct a forward kinematics (FK) solver.
    fk_solver = ChainFkSolverPos_recursive(chain)

    # Construct an inverse kinematics (IK) solver.
    ik_vel_solver = ChainIkSolverVel_pinv(chain)
    ik_solver = ChainIkSolverPos_NR(
        chain, fk_solver, ik_vel_solver)

    q = JntArray(6)
    ee_in_world = Frame.Identity()

    # FK examples.
    fk_solver.JntToCart(q, ee_in_world)
    print("position: \n{}, \norientation: \n{}".format(
        ee_in_world.p, ee_in_world.M))
    print("-----")

    q[0] = 0.2
    fk_solver.JntToCart(q, ee_in_world)
    print("position: \n{}, \norientation: \n{}".format(
        ee_in_world.p, ee_in_world.M))
    print("-----")

    # IK examples.
    ee_in_world = Frame.Identity()
    ee_in_world.p = Vector(0.2, 0, 1.0)
    q_seed = JntArray(6)
    q_soln = JntArray(6)
    res = ik_solver.CartToJnt(q_seed, ee_in_world, q_soln)
    if res >= 0:
        print("Soln: {}".format(q_soln))
    else:
        print("IK failed!")


    # Examples of IK failures? Why does IK fail?


if __name__ == '__main__':
    main()

