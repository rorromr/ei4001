#include <SimpleEncoder.h>

#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

// Active Encoder object for ISR
Encoder* Encoder::_activeEncoder = NULL; 

Encoder::Encoder(uint8_t pinA, uint8_t pinB){
  // Config pins
  pinMode(pinA, INPUT);
  digitalWrite(pinA, HIGH);
  pinMode(pinB, INPUT);
  digitalWrite(pinB, HIGH);
  _pin = pinB;

  // Initial state
  //_pastA = digitalRead(pinA);
  _pastB = DIRECT_PIN_READ(PIN_TO_BASEREG(_pin),PIN_TO_BITMASK(_pin));
  
  // Set active object
  _activeEncoder = this;
  // Config interrupt routines with active object
  attachInterrupt(0,Encoder::isrEncoderA,RISING);
  //attachInterrupt(1,Encoder::isrEncoderB,CHANGE);
}

void Encoder::encoderA()
{
  _pastB = DIRECT_PIN_READ(PIN_TO_BASEREG(_pin),PIN_TO_BITMASK(_pin));
  _pastB ? _encoderPos--: _encoderPos++;
}

// void Encoder::encoderB()
// {
//   _pastB = !_pastB;
// }

int32_t Encoder::read()
{
  return _encoderPos;
}
