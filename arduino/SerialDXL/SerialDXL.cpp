#include <SerialDXL.h>
//------------------------------------------------------------------------------
/** @return The number of bytes in the ring buffer.
 *
 * @note This function must not be called with interrupts disabled.
 */
int SerialRingBuffer::available() {
  uint8_t s = SREG;
  cli();
  int n = head_ - tail_;
  SREG = s;
  return n < 0 ? size_ + n : n;
}
//------------------------------------------------------------------------------
/** Discard all data in the ring buffer.
 *
 * @note This function must not be called with interrupts disabled.
 */
void SerialRingBuffer::flush() {
  uint8_t s = SREG;
  cli();
  head_ = tail_ = 0;
  SREG = s;
}
//------------------------------------------------------------------------------
/** Get the next byte from the ring buffer.
 *
 * @param[in] b location for the returned byte
 * @return @c true if a byte was returned or @c false if the ring buffer is empty
 */
bool SerialRingBuffer::get(uint8_t* b) {
  buf_size_t t = tail_;
  if (head_ == t) return false;
  *b = buf_[t++];
  tail_ = t < size_ ? t : 0;
  return true;
}
//------------------------------------------------------------------------------
/**
 * Get the maximum number of contiguous bytes from the ring buffer
 * with one call to memcpy.
 *
 * @note This function must not be called with interrupts disabled.
 *
 * @param[in] b Pointer to the data.
 * @param[in] n Number of bytes to transfer from the ring buffer.
 * @return Number of bytes transferred.
 */
SerialRingBuffer::buf_size_t SerialRingBuffer::get(uint8_t* b, buf_size_t n) {
  buf_size_t nr;
  cli();
  buf_size_t h = head_;
  sei();
  buf_size_t t = tail_;
  if (h < t) {
    nr = size_ - t;
  } else if (t < h) {
    nr = h - t;
  } else {
    return 0;
  }
  if (nr > n) nr = n;
  memcpy(b, &buf_[t], nr);
  t += nr;
  tail_ = t < size_ ? t : t - size_;
  return nr;
}
//------------------------------------------------------------------------------
/** Initialize the ring buffer.
 * @param[in] b Buffer for the data.
 * @param[in] s Size of the buffer.
 */
void SerialRingBuffer::init(uint8_t* b, buf_size_t s) {
  buf_ = b;
  size_ = s;
  head_ = tail_ = 0;
}
//------------------------------------------------------------------------------
/** Peek at the next byte in the ring buffer.
 * @return The next byte that would be read or -1 if the ring buffer is empty.
 */
int SerialRingBuffer::peek() {
  return empty() ? -1 : buf_[tail_];
}
//------------------------------------------------------------------------------
/** Put a byte into the ring buffer.
 *
 * @param[in] b the byte
 * @return @c true if byte was transferred or
 *         @c false if the ring buffer is full.
 */
bool SerialRingBuffer::put(uint8_t b) {
  buf_size_t h = head_;
  // OK to store here even if ring is full.
  buf_[h++] = b;
  if (h >= size_) h = 0;
  if (h == tail_) return false;
  head_ = h;
  return true;
}
//------------------------------------------------------------------------------
/**
 * Put the maximum number of contiguous bytes into the ring buffer.
 * with one call to memcpy.
 *
 * @note This function must not be called with interrupts disabled.
 *
 * @param[in] b pointer to data.
 * @param[in] n number of bytes to transfer to the ring buffer.
 * @return number of bytes transferred.
 */
SerialRingBuffer::buf_size_t
  SerialRingBuffer::put(const uint8_t* b, buf_size_t n) {
  cli();
  buf_size_t t = tail_;
  sei();
  buf_size_t space;  // space in ring buffer
  buf_size_t h = head_;
  if (h < t) {
    space = t - h - 1;
  } else {
    space = size_ - h;
    if (t == 0) space -= 1;
  }
  if (n > space) n = space;
  memcpy(&buf_[h], b, n);
  h += n;
  head_ = h < size_ ? h : h - size_;
  return n;
}
//------------------------------------------------------------------------------
SerialRingBuffer rxRingBuf;
SerialRingBuffer txRingBuf;
//------------------------------------------------------------------------------
VirtualDeviceDXL *SerialDXL_ISR;

ISR(USART_RX_vect) {
  uint8_t data = UDR0;
  SerialDXL_ISR->process(data);
}

void toggleLed(uint8_t n)
{
  for (int i = 0; i<n; ++i)
  {
    digitalWrite(13, HIGH);
    _delay_us(100);
    digitalWrite(13, LOW);
    _delay_us(100);
  }
}
