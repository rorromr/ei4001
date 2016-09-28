#include <Fin_de_Carrera.h>

#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

Fin_de_Carrera* Fin_de_Carrera::_activeFdc = NULL; 

Fin_de_Carrera::Fin_de_Carrera(uint8_t fdc_1, uint8_t fdc_2, uint8_t fdc_3, uint8_t fdc_4):
  _pinReg1(PIN_TO_BASEREG(fdc_1)),
  _pinReg2(PIN_TO_BASEREG(fdc_2)),
  _pinReg3(PIN_TO_BASEREG(fdc_3)),
  _pinReg4(PIN_TO_BASEREG(fdc_4)),
  _pinMask1(PIN_TO_BITMASK(fdc_1)),
  _pinMask2(PIN_TO_BITMASK(fdc_2)),
  _pinMask3(PIN_TO_BITMASK(fdc_3)),
  _pinMask4(PIN_TO_BITMASK(fdc_4))
{
  // Config pins
  pinMode(fdc_1, INPUT);
  pinMode(fdc_2, INPUT);
  pinMode(fdc_3, INPUT);
  pinMode(fdc_4, INPUT);
  pinMode(3,INPUT);  //Interrupt pin
  
  // Set active object
  _activeFdc = this;
  // Config interrupt routines with active object
  attachInterrupt(1,Fin_de_Carrera::isrFdc,FALLING);
}

void Fin_de_Carrera::fdc()
{
	if (!DIRECT_PIN_READ(_pinReg1,_pinMask1)){
		_index = 1;
	}
	else if (!DIRECT_PIN_READ(_pinReg2,_pinMask2)){
		_index = 2;
	}
	else if (!DIRECT_PIN_READ(_pinReg3, _pinMask3)){
		_index = 3;
	}
	else if (!DIRECT_PIN_READ(_pinReg4,_pinMask4)){
		_index = 4;
	}
  _flag = 1;

}

/* Si hubo interrupcion, vuelvo a false el indicador. 
Retorno el indice del pin que se acitvo*/
int Fin_de_Carrera::getIndex()
{
  _flag = 0;
  return _index;
}


// Indica si ha habido o no una interrupcion
bool Fin_de_Carrera::getState()
{
  return _flag;
}
