#include <HBridge.h>

#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

#define PWM_MAX 255
#define PWM_MIN 0
#define SAT(x) ( ((x) > PWM_MAX) ? PWM_MAX : ( ((x) < PWM_MIN) ? 0 : (x) ) )

//#define DEBUG
#undef DEBUG

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
  Serial.print(" |Target ");
  Serial.print(target);
  #endif

  if (target > 0)
  {
    forward();
    pwm_target = SAT(target);
    #ifdef DEBUG
    Serial.print(" |Forward ");
    Serial.print(pwm_target);
    #endif
  }
  else
  {
    backward();
    pwm_target = SAT(-target);
    #ifdef DEBUG
    Serial.print(" |Backward ");
    Serial.print(pwm_target);
    #endif
  }
  setPwm(pwm_target);
}

// Set PWM frequency  
void HBridge::setPwmFrequency(int divisor)
{
  byte mode;
  if(_pinPwm == 5 || _pinPwm == 6 || _pinPwm == 9 || _pinPwm == 10) {
    switch(divisor)
    {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(_pinPwm == 5 || _pinPwm == 6)
    {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    }
    else
    {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  }
  else if(_pinPwm == 3 || _pinPwm == 11)
  {
    switch(divisor)
    {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
