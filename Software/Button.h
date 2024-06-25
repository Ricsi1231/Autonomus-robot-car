#include <stdint.h>

class Button {
  public:
  Button(uint8_t Pin, uint8_t PullResistor);
  bool Init();
  int Read(int Time);

  private:
  uint8_t pin = 0;
  uint8_t pullResistor = 0;
};