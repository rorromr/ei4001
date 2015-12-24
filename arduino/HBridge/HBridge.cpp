#include <HBridge.h>

#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)


HBridge::HBridge(uint8_t pinPwm, uint8_t pinA, uint8_t pinB):
  _pinPwm(pinPwm),
  _pinA(pinA),
  _pinB(pinB)
{
  pinMode(_pinPwm, OUTPUT);
  pinMode(_pinA, OUTPUT);
  pinMode(_pinB, OUTPUT);
  //brake();
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

