#ifndef PID_h
#define PID_h

#include "Arduino.h"

template <typename T>
class PID
{
private:
  static const uint8_t _order = 2;
  T _num[_order];
  T _den[_order];

public:
  
  PID()
  {
    for(uint8_t i = 0; i < _order; ++i)
    {
      _num[i] = T(0);
      _den[i] = T(0);
    }
  }

  T sum() const;
  
};

typedef PID<double> PIDd;
typedef PID<float> PIDf;

template<typename T>
T PID<T>::sum() const
{
  T aux = T(0);
  for(uint8_t i = 0; i < _order; ++i)
  {
    aux += _num[i] + _den[i];
  }
  return aux;
}


#endif