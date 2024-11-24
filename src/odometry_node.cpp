#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include "Odometry.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "odometry_node");
  odometry::Odometry odometry;
  ros::NodeHandle node_handle;
  ros::Publisher publisher = node_handle.advertise<nav_msgs::Odometry>("/odom", 2);
  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}

