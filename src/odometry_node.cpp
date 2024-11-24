#include <ros/ros.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "odometry_node");
  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}

