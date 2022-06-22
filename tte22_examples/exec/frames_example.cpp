#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker GetVehicleMarker(double x, double y, double angle,
                                            const std::string &frame_id) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_id;
  msg.id = 0;
  msg.type = visualization_msgs::Marker::CUBE;
  msg.ns = "vehicle";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0.05;

  tf2::Quaternion q;
  q.setRPY(0,      // Roll (radians).
           0,      // Pitch (radians).
           angle); // Yaw (radians).
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();

  msg.scale.x = 0.1;
  msg.scale.y = 0.3;
  msg.scale.z = 0.1;

  msg.color.r = 0.5;
  msg.color.g = 0.5;
  msg.color.b = 0;
  msg.color.a = 0.75;

  return msg;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "frames_example");

  ros::NodeHandle nh;

  // Vehicle parameters.
  const double radius = 0.5;
  const double velocity = 0.1;

  // To "broadcast" (i.e., send out) transforms.
  tf2_ros::TransformBroadcaster broadcaster;

  // TODO Listener...

  // To publish visualization "markers".
  ros::Publisher vis_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);

  // Run this loop at 10 Hz (i.e., 10 times per second).
  const double freq = 10.0;
  ros::Rate rate(freq);
  double t = 0.0;
  while (ros::ok()) {
    // Broadcast the transform from the "map" frame to the "vehicle" frame.
    geometry_msgs::TransformStamped map_to_vehicle;
    map_to_vehicle.header.stamp = ros::Time::now();
    map_to_vehicle.header.frame_id = "map";    // "Parent" frame id (name).
    map_to_vehicle.child_frame_id = "vehicle"; // "Child" frame id (name).

    double angle = velocity * t / radius;
    double x = radius * std::cos(angle);
    double y = radius * std::sin(angle);

    map_to_vehicle.transform.translation.x = x;
    map_to_vehicle.transform.translation.y = y;
    map_to_vehicle.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0,      // Roll (radians).
             0,      // Pitch (radians).
             angle); // Yaw (radians).
    map_to_vehicle.transform.rotation.x = q.x();
    map_to_vehicle.transform.rotation.y = q.y();
    map_to_vehicle.transform.rotation.z = q.z();
    map_to_vehicle.transform.rotation.w = q.w();

    broadcaster.sendTransform(map_to_vehicle);

    // How to publish the vehicle at the correction position and orientation?
    // vis_pub.publish(GetVehicleMarker(0, 0, 0, "map"));
    // vis_pub.publish(GetVehicleMarker(x, y, angle, "map"));
    // vis_pub.publish(GetVehicleMarker(0, 0, 0, "vehicle"));

    ros::spinOnce();
    t += (1.0 / freq);
    rate.sleep();
  }

  return 0;
}
