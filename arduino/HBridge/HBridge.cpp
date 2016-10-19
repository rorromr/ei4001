#include <HBridge.h>

#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

#define PWM_MAX 255
#define PWM_MIN 0
#define SAT(x) ( ((x) > PWM_MAX) ? PWM_MAX : ( ((x) < PWM_MIN) ? 0 : (x) ) )

//#define DEBUG
#undef DEBUG

HBridge::HBridge(uint8_t rPwm, uint8_t lPwm):
  _lPwm(lPwm),
  _rPwm(rPwm),
  _pwm(0U)
{
  pinMode(_lPwm, OUTPUT);
  pinMode(_rPwm, OUTPUT);
  brake();
}

void HBridge::forward()
{
  analogWrite(_lPwm, 0U);
  analogWrite(_rPwm, _pwm);
}

void HBridge::backward()
{
  analogWrite(_lPwm, _pwm);
  analogWrite(_rPwm, 0U);
}

void HBridge::brake()
{
  analogWrite(_lPwm, 0U);
  analogWrite(_rPwm, 0U);
}

void HBridge::activeBrake()
{
  analogWrite(_lPwm, 255U);
  analogWrite(_rPwm, 255U);
}

void HBridge::setPwm(const uint8_t pwm)
{
  _pwm = pwm;
}

void HBridge::set(int16_t target)
{ 
  #ifdef DEBUG
  uint8_t pwm_target;
  Serial.print(" |Target ");
  Serial.print(target);
  #endif

  if (target > 0)
  {
    _pwm = SAT(target);
    forward();
    #ifdef DEBUG
    Serial.print(" |Forward ");
    Serial.print(SAT(target));
    #endif
  }
  else
  {
    _pwm = SAT(-target);
    backward();
    #ifdef DEBUG
    Serial.print(" |Backward ");
    Serial.print(SAT(-target));
    #endif
  }
}

// Set PWM frequency  
void HBridge::setPwmFrequency(int divisor)
{
  #if  defined(__AVR_ATmega2560__)
    uint8_t mode;
    switch(divisor)
    {
      case 1: mode = 0x01; break;
      case 2: mode = 0x02; break;
      case 3: mode = 0x03; break;
      case 4: mode = 0x04; break;
      case 5: mode = 0x05; break;
      case 6: mode = 0x06; break;
      case 7: mode = 0x07; break;
      default: return;
    }
    switch(_lPwm)
    {   
      case 2:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 3:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 4:  TCCR0B = TCCR0B  & 0b11111000 | mode; break;
      case 5:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 6:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 7:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 8:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 9:  TCCR2B = TCCR2B  & 0b11111000 | mode; break;
      case 10: TCCR2B = TCCR2B  & 0b11111000 | mode; break;
      case 11: TCCR1B = TCCR1B  & 0b11111000 | mode; break;  
      case 12: TCCR1B = TCCR1B  & 0b11111000 | mode; break;  
      case 13: TCCR0B = TCCR0B  & 0b11111000 | mode; break;
      default: return;
    }
    switch(_rPwm)
    {   
      case 2:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 3:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 4:  TCCR0B = TCCR0B  & 0b11111000 | mode; break;
      case 5:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 6:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 7:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 8:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 9:  TCCR2B = TCCR2B  & 0b11111000 | mode; break;
      case 10: TCCR2B = TCCR2B  & 0b11111000 | mode; break;
      case 11: TCCR1B = TCCR1B  & 0b11111000 | mode; break;  
      case 12: TCCR1B = TCCR1B  & 0b11111000 | mode; break;  
      case 13: TCCR0B = TCCR0B  & 0b11111000 | mode; break;
      default: return;
    }
  #endif
}
  
