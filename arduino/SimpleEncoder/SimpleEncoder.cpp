#include <SimpleEncoder.h>

#define PIN_TO_INPUT_REG(pin)           (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_OUTPUT_REG(pin)          (portOutputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

// Active Encoder object for ISR
Encoder* Encoder::_activeEncoder = NULL;

Encoder::Encoder(uint8_t pinA, uint8_t pinB):
  _pin(pinB),
  _pinReg(PIN_TO_INPUT_REG(pinB)),
  _pinMask(PIN_TO_BITMASK(pinB))
{
#if ENCODER_USE_FLAG_PIN
  _flagReg = PIN_TO_OUTPUT_REG(ENCODER_FLAG_PIN);
  _flagMask = PIN_TO_BITMASK(ENCODER_FLAG_PIN);
  pinMode(ENCODER_FLAG_PIN, OUTPUT);
#endif
  // Config pins
  pinMode(pinA, INPUT);
  digitalWrite(pinA, HIGH); // Pullup
  pinMode(pinB, INPUT);
  digitalWrite(pinB, HIGH); // Pullup

  // Initial state
  _pastB = DIRECT_PIN_READ(_pinReg,_pinMask);
  
  // Set active object
  _activeEncoder = this;
  // Config interrupt routines with active object
  attachInterrupt(digitalPinToInterrupt(pinA),Encoder::isrEncoderA,RISING);

}

void Encoder::encoderA()
{
#if ENCODER_USE_FLAG_PIN
  *_flagReg |= _flagMask;
#endif

  _pastB = DIRECT_PIN_READ(_pinReg,_pinMask);
  _pastB ? _encoderPos--: _encoderPos++;

#if ENCODER_USE_FLAG_PIN
  *_flagReg &= ~_flagMask;
#endif
}

int32_t Encoder::read()
{
  return _encoderPos;
}

void Encoder::write(int32_t p)
{
  _encoderPos = p;
}
