#include <stdint.h>

class Motor {
  public:
  Motor(uint8_t Motor1Pin1, uint8_t Motor1Pin2, uint8_t Motor1PWM, uint8_t Motor2Pin1, uint8_t Motor2Pin2, uint8_t Motor2PWM);
  bool Init();
  void Forward(int speed);
  void TurnLeft(int speed);
  void TurnRight(int speed);
  void Backward(int speed);
  void Stop(); 
  
  private:
  uint8_t motor1PWM = 0;
  uint8_t motor1_Pin1 = 0;
  uint8_t motor1_Pin2 = 0;

  uint8_t motor2PWM = 0;
  uint8_t motor2_Pin1 = 0;
  uint8_t motor2_Pin2 = 0;
};