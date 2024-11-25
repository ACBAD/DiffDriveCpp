#ifndef ODOMETRY_H
#define ODOMETRY_H

#include<ros/ros.h>
#include <Encoder.h>
#include <Pose.h>
#include <ctime>

namespace odometry_namespace {

class Odometry {
  encoder::Encoder left_encoder, right_encoder;
  int32_t tick_per_meter;
  ros::Time last_time;
  Pose pose;
  double wheel_separation;
public:
  Odometry();
  void setWheelSeparation(double wheel_separation_);
  void setTicksPerMeter(int32_t ticks);
  void setEncoderRange(int32_t low, int32_t high);
  void setTime(ros::Time time);
  void updateLeftWheel(int32_t newCount);
  void updateRightWheel(int32_t newCount);
  Pose getPose() const;
  void setPose(const Pose& new_pose);
  void updatePose(ros::Time new_time);
};

} // odometry

#endif //ODOMETRY_H
