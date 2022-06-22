#!/usr/bin/env python

# NOTE: Must run `chmod +x frames_example.py` in order to make executable before
# running via `rosrun tte22_examples frames_example.py`

import rospy
import sys
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
import math


def get_vehicle_marker(x, y, angle, frame_id):
    msg = Marker()
    msg.header.frame_id = frame_id
    msg.id = 0
    msg.type = Marker.CUBE
    msg.ns = 'vehicle'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.08

    orien = quaternion_from_euler(
        0.0,    # Roll (radians).
        0.0,    # Pitch (radians).
        angle)  # Yaw (radians).
    msg.pose.orientation.x = orien[0]
    msg.pose.orientation.y = orien[1]
    msg.pose.orientation.z = orien[2]
    msg.pose.orientation.w = orien[3]

    msg.scale.x = 0.1
    msg.scale.y = 0.3
    msg.scale.z = 0.1

    msg.color.r = 0.5
    msg.color.g = 0.5
    msg.color.b = 0
    msg.color.a = 0.75

    return msg


def main():
    rospy.init_node('frames_example')

    # Vehicle parameters.
    radius = 0.5
    velocity = 0.1

    # To "broadcast" (i.e., send out) transforms.
    broadcaster = tf2_ros.TransformBroadcaster()

    # To "listen" for transforms (unused, for now).
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # To publish visualization "markers".
    vis_pub = rospy.Publisher(
        'visualization_marker', Marker, queue_size=25)

    # Run this loop at 10 Hz (i.e., 10 times per second).
    freq = 10.0
    rate = rospy.Rate(freq)
    t = 0.0
    while not rospy.is_shutdown():
        # Broadcast the transform from the "map" frame to the "vehicle" frame.
        map_to_vehicle = TransformStamped()
        map_to_vehicle.header.stamp = rospy.Time.now()
        map_to_vehicle.header.frame_id = 'map'    # "Parent" frame id (name).
        map_to_vehicle.child_frame_id = 'vehicle' # "Child" frame id (name).

        angle = velocity * t / radius
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)

        map_to_vehicle.transform.translation.x = x
        map_to_vehicle.transform.translation.y = y
        map_to_vehicle.transform.translation.z = 0.0

        orien = quaternion_from_euler(
            0.0,    # Roll (radians).
            0.0,    # Pitch (radians).
            angle)  # Yaw (radians).
        map_to_vehicle.transform.rotation.x = orien[0]
        map_to_vehicle.transform.rotation.y = orien[1]
        map_to_vehicle.transform.rotation.z = orien[2]
        map_to_vehicle.transform.rotation.w = orien[3]

        broadcaster.sendTransform(map_to_vehicle)

        # How to publish the vehicle at the correct position and orientation?
        # vis_pub.publish(
        #     get_vehicle_marker(0, 0, 0, frame_id='map'))
        # vis_pub.publish(
        #     get_vehicle_marker(x, y, angle, frame_id='map'))
        vis_pub.publish(
            get_vehicle_marker(0, 0, 0, frame_id='vehicle'))

        t += (1.0 / freq)
        rate.sleep()


if __name__ == '__main__':
    main()
