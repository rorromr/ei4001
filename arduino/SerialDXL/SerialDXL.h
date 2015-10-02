#ifndef SerialDXL_h
#define SerialDXL_h

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