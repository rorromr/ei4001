#ifndef HBridge_h
#define HBridge_h

#include "Arduino.h"

class HBridge
{
private:
  uint8_t _pinPwm;
  uint8_t _pinA;
  uint8_t _pinB;

public:
  
  /**
   * @brief HBridge Class
   * 
   * @param pwmPin PWM pin
   * @param pinA
   * @param pinB
   */
  HBridge(uint8_t pinPwm, uint8_t pinA, uint8_t pinB);
  
  void forward();

  void backward();

  void brake();

  void activeBrake();

  void setPwm(uint8_t pwm);


};

#endif