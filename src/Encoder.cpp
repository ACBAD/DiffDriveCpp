#include "Encoder.h"

namespace encoder {
  Encoder::Encoder() {
    setEncoderRange(-32768, 32767);
    initCount(0);
    isReverse = false;
  }
  void Encoder::setEncoderRange(const int32_t low, const int32_t high) {
    range = high - low + 1;
    lowThresh = low + range * 30 / 100;
    highThresh = low + range * 70 / 100;
  }
  void Encoder::initCount(const int32_t startCount) {
    delta = 0;
    last = startCount;
  }
  void Encoder::setReversed(const bool Reverse) {isReverse = Reverse;}
  int32_t Encoder::getDelta() {
    const int32_t idelta = delta;
    delta = 0;
    if(isReverse)
      return -idelta;
    return idelta;
  }
  void Encoder::update(int32_t newCount) {
    int32_t increment;
    if (last > highThresh && newCount < lowThresh)
      increment = newCount + range - last;
    else if(last < lowThresh && newCount > highThresh)
      increment = newCount - range - last;
    else
      increment = newCount - last;
    delta += increment;
    last = newCount;
  }
} // encoder