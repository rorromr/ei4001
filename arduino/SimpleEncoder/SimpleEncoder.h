#ifndef SimpleEncoder_h
#define SimpleEncoder_h

#include "Arduino.h"

class Encoder
{
private:
	volatile int32_t _encoderPos = 0;
	volatile bool _PastA = 0;
	volatile bool _PastB = 0;
	
public:
	Encoder(int pin1, int pin2);

	void init();

	void doEncoderA()
	{

		_PastB ? _encoderPos--: _encoderPos++;
	}

	void doEncoderB()
	{
		_PastB = !_PastB;
	}

	int32_t read()
	{
		return _encoderPos;
	}
};

#endif