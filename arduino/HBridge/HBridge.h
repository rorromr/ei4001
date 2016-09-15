#ifndef HBridge_h
#define HBridge_h

#include "Arduino.h"

class HBridge
{
private:
  uint8_t _lPwm;
  uint8_t _rPwm;
  uint8_t _pwm;

public:
  
  /**
   * @brief HBridge Class
   * 
   * @param pwmPin PWM pin
   * @param pinA
   * @param pinB
   */
  HBridge(uint8_t rPwm, uint8_t lPwm);
  
  void forward();

  void backward();

  void brake();

  void activeBrake();

  void setPwm(const uint8_t pwm);

  void set(int16_t target);

  void setPwmFrequency(int divisor);


};

#endif