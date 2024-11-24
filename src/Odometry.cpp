#include "Odometry.h"

#include <cmath>
#define PI 3.14159216535

namespace odometry {
  Odometry::Odometry() {
    left_encoder = encoder::Encoder();
    right_encoder = encoder::Encoder();
    pose = {0,0,0,0,0,0};
    wheel_separation = 0;
    last_time = 0;
    tick_per_meter = 0;
  }
  void Odometry::setWheelSeparation(const double wheel_separation_) {wheel_separation = wheel_separation_;}
  void Odometry::setTicksPerMeter(const int32_t ticks){tick_per_meter = ticks;}
  void Odometry::setEncoderRange(const int32_t low, const int32_t high) {
    left_encoder.setEncoderRange(low, high);
    right_encoder.setEncoderRange(low, high);
  }
  void Odometry::setTime(const time_t time){last_time = time;}
  void Odometry::updateLeftWheel(const int32_t newCount){left_encoder.update(newCount);}
  void Odometry::updateRightWheel(const int32_t newCount){right_encoder.update(newCount);}
  Pose Odometry::getPose() const {return pose;}
  void Odometry::setPose(const Pose& new_pose){pose = new_pose;}
  void Odometry::updatePose(const time_t new_time) {
    const double leftTravel = left_encoder.getDelta() / static_cast<double>(tick_per_meter);
    const double rightTravel = right_encoder.getDelta() / static_cast<double>(tick_per_meter);
    const time_t deltaTime = new_time - last_time;
    const double deltaTravel = (rightTravel + leftTravel) / 2;
    const double deltaTheta = (rightTravel - leftTravel) / wheel_separation;
    double deltaX, deltaY;
    if (rightTravel == leftTravel) {
      deltaX = leftTravel * cos(pose.theta);
      deltaY = leftTravel * sin(pose.theta);
    }
    else {
      const double radius = deltaTravel / deltaTheta;
      const double iccX = pose.x - radius * sin(pose.theta);
      const double iccY = pose.y + radius * cos(pose.theta);
      deltaX = cos(deltaTheta) * (pose.x - iccX) - sin(deltaTheta) * (pose.y - iccY) + iccX - pose.x;
      deltaY = sin(deltaTheta) * (pose.x - iccX) + cos(deltaTheta) * (pose.y - iccY) + iccY - pose.y;
    }
    pose.x += deltaX;
    pose.y += deltaY;
    pose.theta = fmod(pose.theta + deltaTheta , 2 * PI);
    pose.xVel = deltaTime > 0 ? deltaTravel / static_cast<double>(deltaTime) : 0;
    pose.yVel = 0;
    pose.thetaVel = deltaTime > 0 ? deltaTheta / static_cast<double>(deltaTime) : 0;
    last_time = new_time;
  }
} // odometry