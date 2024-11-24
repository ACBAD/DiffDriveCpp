#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Encoder.h"
#include "Pose.h"
#include <ctime>

namespace odometry {

class Odometry {
  encoder::Encoder left_encoder, right_encoder;
  int32_t tick_per_meter;
  time_t last_time;
  Pose pose;
  double wheel_separation;
public:
  Odometry();
  void setWheelSeparation(double wheel_separation_);
  void setTicksPerMeter(int32_t ticks);
  void setEncoderRange(int32_t low, int32_t high);
  void setTime(time_t time);
  void updateLeftWheel(int32_t newCount);
  void updateRightWheel(int32_t newCount);
  Pose getPose() const;
  void setPose(const Pose& new_pose);
  void updatePose(time_t new_time);
};

} // odometry

#endif //ODOMETRY_H
