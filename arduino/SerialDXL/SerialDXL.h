/**
 * @file
 * @brief Dynamixel Device Library
 */
#ifndef SerialDXL_h
#define SerialDXL_h
//------------------------------------------------------------------------------
/**
 * Debug macros
 */
#if(DEBUG == 0)
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif
//------------------------------------------------------------------------------
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <string.h>
//------------------------------------------------------------------------------
/**
 * @class DeviceDXL
 * @brief Virtual class for Device with DXL.
 */
class DeviceDXL {
  public:
    virtual void initRAM() {}

    virtual void initEEPROM() {}

    virtual inline __attribute__((always_inline))
    void setTX() {}

    virtual inline __attribute__((always_inline))
    void setRX() {}

    // ID
    uint8_t id_;

};
//------------------------------------------------------------------------------
/** Memory map max size */
static const uint8_t MMAP_MAX_SIZE = 16;
/** First bit LSB */
static const uint8_t MMAP_RW = 1;
static const uint8_t MMAP_R = 0;
/** Second bit */
static const uint8_t MMAP_RAM = 1 << 1;
static const uint8_t MMAP_EEPROM = 0;
//------------------------------------------------------------------------------
/** Cause error message for bad Size.
 * @return Never returns since it is never called.
 */
uint8_t badSize(void)
  __attribute__((error("MMAP size too large")));
//------------------------------------------------------------------------------
/**
 * @class mmap_entry_t
 * @brief Entry type for memory map.
 */
typedef struct
{
  uint8_t *value;
  uint8_t param;
} mmap_entry_t;
//------------------------------------------------------------------------------
// Set MMAP entry macro
#define MMAP_ENTRY(mmap, var, parameter) {(mmap).value = &(var); (mmap).param = (parameter);}
//------------------------------------------------------------------------------
/**
 * @class MMap
 * @brief Memory mapping.
 */
class MMap
{
  public:
    MMap(size_t N):
    N_(N)
    {
      // Check size
      if (N_ > MMAP_MAX_SIZE) badSize();
    }

    void setMMap(mmap_entry_t *mmap)
    {
      mmap_ = mmap;
    }

    inline __attribute__((always_inline))
    uint8_t setEEPROM(uint8_t address, uint8_t value)
    {
      DEBUG_PRINTLN("set eeprom");
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        eeprom_write_byte ( (uint8_t*)(uint16_t) address, value);
      }
      return (*mmap_[address].value)=value;
    }

    inline __attribute__((always_inline))
    uint8_t getEEPROM(uint8_t address)
    {
      DEBUG_PRINTLN("get eeprom");
      uint8_t read;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        read = eeprom_read_byte( (uint8_t*)(uint16_t) address );
      }
      return read;
    }

    void set(uint8_t address, uint8_t value)
    {
      // Check for size
      DEBUG_PRINTLN("set");
      if (address > N_)
      {
        DEBUG_PRINTLN("address error");
        return;
      }
      // Check for access
      if (!(mmap_[address].param & MMAP_RW))
      {
        DEBUG_PRINTLN(mmap_[address].param);
        DEBUG_PRINTLN("read only address");
        return;
      }

      
      mmap_[address].param & MMAP_RAM ? 
        (*mmap_[address].value)=value : setEEPROM(address, value);
    }

    void setFromEEPROM(uint8_t address)
    {
      // Check for size
      DEBUG_PRINTLN("set from epprom");
      if (address > N_)
      {
        DEBUG_PRINTLN("address error");
        return;
      }
      uint8_t value = 0;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        value = eeprom_read_byte( (uint8_t*)(uint16_t) address );
      }
      (*mmap_[address].value)=value;
    }

    uint8_t get(uint8_t address)
    {
      // Check for size
      DEBUG_PRINTLN("get");
      if (address > N_)
      {
        DEBUG_PRINTLN("address error");
        return 0;
      }
      return mmap_[address].param & MMAP_RAM ? 
        (*mmap_[address].value) : getEEPROM(address);
    }


  private:
    // Memory mapping with pointers to RAM and EEPROM data
    mmap_entry_t *mmap_;
    const size_t N_;
};
//------------------------------------------------------------------------------
/**
 * @class SerialRingBuffer
 * @brief Ring buffer for RX and TX data. Based on Bill Greiman Serial library
 * (https://github.com/greiman/SerialPort).
 */
class SerialRingBuffer {
 public:
  /** Define type for buffer indices */
  typedef uint8_t buf_size_t;
  int available();
  /** @return @c true if the ring buffer is empty else @c false. */
  bool empty() {return head_ == tail_;}
  void flush();
  bool get(uint8_t* b);
  buf_size_t get(uint8_t* b, buf_size_t n);
  void init(uint8_t* b, buf_size_t s);
  int peek();
  bool put(uint8_t b);
  buf_size_t put(const uint8_t* b, buf_size_t n);
 private:
  uint8_t* buf_;              /**< Pointer to start of buffer. */
  volatile buf_size_t head_;  /**< Index to next empty location. */
  volatile buf_size_t tail_;  /**< Index to last entry if head_ != tail_. */
  buf_size_t size_;           /**< Size of the buffer. Capacity is size -1. */
};
//------------------------------------------------------------------------------
/** RX ring buffers. */
extern SerialRingBuffer rxRingBuf;
/** TX ring buffers. */
extern SerialRingBuffer txRingBuf;
//------------------------------------------------------------------------------
/**
 * @class SerialDXL
 * @brief Serial port wrapper for DXL communication protocol.
 */
#define UBRR_VALUE(BAUDRATE) (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define RX_BUFFER_MAX 100
#define TX_BUFFER_MAX 100
/** Cause error message for RX buffer bad Size.
 * @return Never returns since it is never called.
 */
uint8_t badRxBufSize(void)
  __attribute__((error("RX buffer size too large or zero")));

/** Cause error message for TX buffer bad Size.
 * @return Never returns since it is never called.
 */
uint8_t badTxBufSize(void)
  __attribute__((error("TX buffer size too large or zero")));


template<size_t RxBufSize, size_t TxBufSize>
class SerialDXL
{
  public:
    SerialDXL(DeviceDXL *device):
      msgState_(0),
      msgParamIdx_(0),
      msgLen_(0) 
    {
      // Check buffer sizes
      if (RxBufSize > RX_BUFFER_MAX || !RxBufSize) badRxBufSize();
      if (TxBufSize > TX_BUFFER_MAX || !TxBufSize) badTxBufSize();
      // Init buffers
      rxRingBuf.init(rxBuffer_, sizeof(rxBuffer_));
      txRingBuf.init(txBuffer_, sizeof(txBuffer_));
      // Set DeviceDXL
      device_ = device;
    }

    void begin(uint8_t baud)
    {
      // Set baudrate using Dynamixel relation
      uint16_t baud_setting = UBRR_VALUE(2000000/(baud+1));

      /* Disable USART interrupts     
      RXCIE0 | RX Complete interrupt enable
      TXCIE0 | TX Complete interrupt enable
      UDRIE0 | USART Data register empty interrupt enable (UDR0 Ready)
      RXEN0  | RX Receiver enables
      TXEN0  | TX Transmitter enable
      */
      UCSR0B &= ~((1<<TXEN0)|(1<<UDRIE0)|(1<<RXEN0)|(1<<RXCIE0));

      // Set baud rate
      UBRR0H = (uint8_t)(baud_setting>>8);  // High bit
      UBRR0L = (uint8_t)baud_setting;       // Low bit

      // Set frame format to Dynamixel (8 data bits, no parity, 1 stop bit)
      UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);

      // Enable USART interrupts
      UCSR0B |= ((1<<TXEN0)|(1<<UDRIE0)|(1<<RXEN0)|(1<<RXCIE0));
    }

    void onRX(uint8_t data)
    {
      switch(msgState_)
      {
        case 0: // 0xFF
          if (data == 0xFF) msgState_ = 1;
          break;
          
        case 1: // 0XFF
          if (data == 0xFF) msgState_ = 2;
          break;
          
        case 2: // ID
          // Check error
          if (data == 0xFF)
          {
            msgState_ = 0;
            break;
          }
          
          if (data == device_->id_)
          {
            msgState_ = 3;
          }
          break;
          
        case 3: // Length
          msgLen_ = data;
          msgState_ = 4;
          break;

        case 4: // Instruction
          //instruction = data;
          msgState_ = 5;
          if (msgLen_ <= 1) msgState_ = 6; else msgState_ = 5;                      
          break;
        
        case 5: // Parameters
          msgParam_[msgParamIdx_++] = data;
          if (msgParamIdx_ >= msgLen_-2) msgState_ = 6;
          break;

        case 6: // Checksum
          msgState_ = 0;
          msgParamIdx_ = 0;
          //finish = 1;
          break;
      }
    }


  private:
    // DeviceDXL
    DeviceDXL *device_;
    // Mesage reception state
    uint8_t msgState_;
    uint8_t msgParam_[8]; 
    uint8_t msgParamIdx_;
    uint8_t msgLen_;

    // Error state
    uint8_t error_;

    // RX buffer with a capacity of RxBufSize.
    uint8_t rxBuffer_[RxBufSize + 1];
    // TX buffer with a capacity of TxBufSize.
    uint8_t txBuffer_[TxBufSize + 1];
};


#endif