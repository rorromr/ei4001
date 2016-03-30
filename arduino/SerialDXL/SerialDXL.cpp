#include <SerialDXL.h>
//------------------------------------------------------------------------------
// Serial ISR
VirtualDeviceDXL *SerialDXL_ISR;
// Serial (ATmega328P)
#if defined(USART_RX_vect)
ISR(USART_RX_vect) {
  uint8_t data = *usart[0].udr;
  SerialDXL_ISR->process(data);
}
// Serial1 (ATmega2560)
#elif defined(USART1_RX_vect)
ISR(USART1_RX_vect) {
  uint8_t data = *usart[1].udr;
  SerialDXL_ISR->process(data);
}
#endif
