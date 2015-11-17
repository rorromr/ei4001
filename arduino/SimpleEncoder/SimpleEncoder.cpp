#include <SimpleEncoder.h>

// Active Encoder object for ISR
Encoder* Encoder::_activeEncoder = NULL; 

Encoder::Encoder(uint8_t pinA, uint8_t pinB){
  // Config pins
  pinMode(pinA, INPUT);
  digitalWrite(pinA, HIGH);
  pinMode(pinB, INPUT);
  digitalWrite(pinB, HIGH);

  // Initial state
  _pastA = digitalRead(pinA);
  _pastB = digitalRead(pinB);
  
  // Set active object
  _activeEncoder = this;
  // Config interrupt routines with active object
  attachInterrupt(0,Encoder::isrEncoderA,RISING);
  attachInterrupt(1,Encoder::isrEncoderB,CHANGE);
}

void Encoder::encoderA()
{
  _pastB ? _encoderPos--: _encoderPos++;
}

void Encoder::encoderB()
{
  _pastB = !_pastB;
}

int32_t Encoder::read()
{
  return _encoderPos;
}
