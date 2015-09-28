#ifndef RS485_DXL_h
#define RS485_DXL_h

#include <avr/io.h>
#include <util/atomic.h>
#include <Arduino.h>

static inline __attribute__((always_inline))
void setTX()
{
  digitalWrite(DIR_PIN,HIGH);
}

static inline __attribute__((always_inline))
void setRX()
{
  digitalWrite(DIR_PIN,LOW);
}

#endif