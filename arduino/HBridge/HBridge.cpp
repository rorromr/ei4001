#include <HBridge.h>

#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

#define PWM_MAX 200
#define PWM_MIN 50
#define SAT(x) ( ((x) > PWM_MAX) ? PWM_MAX : ( ((x) < PWM_MIN) ? PWM_MIN : (x) ) )

//#define DEBUG

HBridge::HBridge(uint8_t pinPwm, uint8_t pinA, uint8_t pinB):
  _pinPwm(pinPwm),
  _pinA(pinA),
  _pinB(pinB)
{
  pinMode(_pinPwm, OUTPUT);
  pinMode(_pinA, OUTPUT);
  pinMode(_pinB, OUTPUT);
  brake();
}

void HBridge::forward()
{
  digitalWrite(_pinA, HIGH);
  digitalWrite(_pinB, LOW);
}

void HBridge::backward()
{
  digitalWrite(_pinA, LOW);
  digitalWrite(_pinB, HIGH);
}

void HBridge::brake()
{
  digitalWrite(_pinA, LOW);
  digitalWrite(_pinB, LOW);
}

void HBridge::activeBrake()
{
  digitalWrite(_pinA, HIGH);
  digitalWrite(_pinB, HIGH);
}

void HBridge::setPwm(uint8_t pwm)
{
  analogWrite(_pinPwm, pwm);
}

void HBridge::set(int16_t target)
{
  uint8_t pwm_target;
  #ifdef DEBUG
  Serial.print(target);
  #endif

  if (target > 0)
  {
    forward();
    pwm_target = SAT(target);
    #ifdef DEBUG
    Serial.print(" |Forward ");
    Serial.println(pwm_target);
    #endif
  }
  else
  {
    backward();
    pwm_target = SAT(-target);
    #ifdef DEBUG
    Serial.print(" |Backward ");
    Serial.println(pwm_target);
    #endif
  }
  setPwm(pwm_target);
}

