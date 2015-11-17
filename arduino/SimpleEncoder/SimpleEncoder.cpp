#include <SimpleEncoder.h>

Encoder *Encoder_ISR;

Encoder::Encoder(int pin1, int pin2){
  pinMode(pin1, INPUT);
  digitalWrite(pin1, HIGH);
  pinMode(pin2, INPUT);
  digitalWrite(pin2, HIGH);
  _PastA = digitalRead(pin1);
  _PastB = digitalRead(pin2);
  Encoder_ISR = this;
}

void isr_a()
{
  Encoder_ISR->doEncoderA();
}

void isr_b()
{
  Encoder_ISR->doEncoderB();
}

void Encoder::init()
{
  attachInterrupt(0,isr_a,RISING);
  attachInterrupt(1,isr_b,CHANGE);
}
