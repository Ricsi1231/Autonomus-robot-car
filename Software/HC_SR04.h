#include <stdint.h>

class HC_SR04 {
  public:
  HC_SR04(uint8_t TriggerPin, uint8_t EchoPin);
  bool Init();
  int Read();

  private:
  uint8_t triggerPin = 0;
  uint8_t echoPin = 0;
};