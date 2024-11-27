#include <ros/ros.h>
#include <Controller.h>

controller_ns::Controller controller;
double linear_velocity = 0.0;
double angular_velocity = 0.0;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "motor_controller_node");
  while (ros::ok()) {
    ros::spinOnce();
  }
}

