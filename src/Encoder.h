#ifndef ENCODER_H
#define ENCODER_H
#include <cstdint>

namespace encoder {

class Encoder {
  int32_t range{}, lowThresh{}, highThresh{}, delta{}, last{};
  bool isReverse{};
public:
  Encoder();
  void setEncoderRange(int32_t low, int32_t high);
  void initCount(int32_t startCount);
  void setReversed(bool Reverse);
  int32_t getDelta();
  void update(int32_t newCount);
};

} // encoder

#endif //ENCODER_H
