#ifndef SimpleEncoder_h
#define SimpleEncoder_h

// Debug
#define ENCODER_USE_FLAG_PIN 0
#define ENCODER_FLAG_PIN 12

#include "Arduino.h"

class Encoder
{
private:
  volatile int32_t _encoderPos = 0;
  volatile bool _pastA = 0;
  volatile bool _pastB = 0;
  static Encoder* _activeEncoder;
  uint8_t _pin;

  volatile uint8_t* _pinReg;
  uint8_t _pinMask;

#ifdef ENCODER_USE_FLAG_PIN
  volatile uint8_t* _flagReg;
  uint8_t _flagMask;
#endif
  
public:
  
  /**
   * @brief Encoder Class
   * Encoder update interrupt ~2.5us
   * Tested at ~6kHz
   * 
   * @param pinA Signal A pin
   * @param pinB Signal B pin
   */
  Encoder(uint8_t pinA, uint8_t pinB);
  
  void encoderA();

  /**
   * @brief Interrupt routine for A signal
   */
  static inline void isrEncoderA()
  {
    if (_activeEncoder)
      _activeEncoder->encoderA();
  }

  /**
   * @brief Read encoder ticks count
   */
  int32_t read();

  /**
   * @brief Override current encoder ticks count
   */
  void write(int32_t p);

};



#endif