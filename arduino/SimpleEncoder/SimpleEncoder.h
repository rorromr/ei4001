#ifndef SimpleEncoder_h
#define SimpleEncoder_h

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

  
public:
  
  /**
   * @brief Encoder Class
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
   * @brief Read encoder tick
   */
  int32_t read();

};

#endif