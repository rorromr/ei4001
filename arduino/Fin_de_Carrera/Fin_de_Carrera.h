#ifndef Fin_de_Carrera_h
#define Fin_de_Carrera_h

#include "Arduino.h"

class Fin_de_Carrera
{
private:
  static Fin_de_Carrera* _activeFdc;

  volatile uint8_t* _pinReg1;
  volatile uint8_t* _pinReg2;
  volatile uint8_t* _pinReg3;
  volatile uint8_t* _pinReg4;

  uint8_t _pinMask1;
  uint8_t _pinMask2;
  uint8_t _pinMask3;
  uint8_t _pinMask4;

  int _index = 0;
  volatile bool _flag = 0;


public:

  Fin_de_Carrera(uint8_t fdc_1, uint8_t fdc_2, uint8_t fdc_3, uint8_t fdc_4);

  static inline void isrFdc()
  {
      _activeFdc->fdc();
  }

  void fdc();

  int getIndex();

  bool getState();

  };

#endif
