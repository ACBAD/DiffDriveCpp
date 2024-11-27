#include <Controller.h>

namespace controller_ns {
  void Controller::setWheelSeparation(const double wheel_separation_) {wheel_separation = wheel_separation_;}
  void Controller::setTicksPerMeter(const double ticks_per_meter_){ticks_per_meter = ticks_per_meter_;}
  void Controller::setMaxMotorSpeed(const double max_motor_speed_ ) {max_motor_speed = max_motor_speed_;}
  MotorCommandType Controller::getSpeeds(const double linear_speed, const double angular_speed) const{
    const double tickRate = linear_speed * ticks_per_meter;
    const double diffTicks = angular_speed * wheel_separation * ticks_per_meter;
    MotorCommandType speeds;
    speeds.left = tickRate - diffTicks;
    speeds.right = tickRate + diffTicks;
    if (std::max(std::abs(speeds.left), std::abs(speeds.right)) > max_motor_speed) {
      const double factor = max_motor_speed / std::max(speeds.left, speeds.right);
      speeds.left *= factor;
      speeds.right *= factor;
    }
    speeds.left = static_cast<uint32_t>(speeds.left);
    speeds.right = static_cast<uint32_t>(speeds.right);
    return speeds;
  }

} // controller