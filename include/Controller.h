#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cstdint>
#include <algorithm>

namespace controller_ns {
struct MotorCommandType {
  double left = 0, right = 0;
};
class Controller {
  double ticks_per_meter{0}, wheel_separation{0}, max_motor_speed = 1e9;
public:
  Controller() = default;
  void setWheelSeparation(double wheel_separation_);
  void setTicksPerMeter(double ticks_per_meter_);
  void setMaxMotorSpeed(double max_motor_speed_);
  MotorCommandType getSpeeds(double linear_speed, double angular_speed) const;
};

} // controller

#endif //CONTROLLER_H
