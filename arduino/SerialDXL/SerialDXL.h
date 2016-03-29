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
#ifdef DEBUG
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
// Serial registers
#if defined(UCSR3A)
static const uint8_t SERIAL_PORT_COUNT = 4;
#elif defined(UCSR2A)
static const uint8_t SERIAL_PORT_COUNT = 3;
#elif defined(UCSR1A)
static const uint8_t SERIAL_PORT_COUNT = 2;
#elif defined(UCSR0A) || defined(UCSRA)
static const uint8_t SERIAL_PORT_COUNT = 1;
#else
#error no serial ports.
#endif
//------------------------------------------------------------------------------
#ifdef UCSR0A
// Bits in UCSRA.
static const uint8_t M_RXC  = 1 << RXC0;
static const uint8_t M_TXC  = 1 << TXC0;
static const uint8_t M_UDRE = 1 << UDRE0;
static const uint8_t M_FE   = 1 << FE0;
static const uint8_t M_DOR  = 1 << DOR0;
static const uint8_t M_UPE  = 1 << UPE0;
static const uint8_t M_U2X  = 1 << U2X0;
// Bits in UCSRB.
static const uint8_t M_RXCIE = 1 << RXCIE0;
static const uint8_t M_TXCIE = 1 << TXCIE0;
static const uint8_t M_UDRIE = 1 << UDRIE0;
static const uint8_t M_RXEN  = 1 << RXEN0;
static const uint8_t M_TXEN  = 1 << TXEN0;
// Bits in UCSRC.
static const uint8_t M_UPM0 = 1 << UPM00;
static const uint8_t M_UPM1 = 1 << UPM01;
static const uint8_t M_USBS = 1 << USBS0;
static const uint8_t M_UCSZ0 = 1 << UCSZ00;
static const uint8_t M_UCSZ1 = 1 << UCSZ01;
#elif defined(UCSRA)  // UCSR0A
// Bits in UCSRA.
static const uint8_t M_RXC  = 1 << RXC;
static const uint8_t M_TXC  = 1 << TXC;
static const uint8_t M_UDRE = 1 << UDRE;
static const uint8_t M_FE   = 1 << FE;
static const uint8_t M_DOR  = 1 << DOR;
static const uint8_t M_UPE  = 1 << PE;
static const uint8_t M_U2X  = 1 << U2X;
// Bits in UCSRB.
static const uint8_t M_RXCIE = 1 << RXCIE;
static const uint8_t M_TXCIE = 1 << TXCIE;
static const uint8_t M_UDRIE = 1 << UDRIE;
static const uint8_t M_RXEN  = 1 << RXEN;
static const uint8_t M_TXEN  = 1 << TXEN;
// Bits in UCSRC.
static const uint8_t M_UPM0 = 1 << UPM0;
static const uint8_t M_UPM1 = 1 << UPM1;
static const uint8_t M_USBS = 1 << USBS;
static const uint8_t M_UCSZ0 = 1 << UCSZ0;
static const uint8_t M_UCSZ1 = 1 << UCSZ1;
#elif defined(UCSR1A)  // UCSR0A
// Bits in UCSRA.
static const uint8_t M_RXC  = 1 << RXC1;
static const uint8_t M_TXC  = 1 << TXC1;
static const uint8_t M_UDRE = 1 << UDRE1;
static const uint8_t M_FE   = 1 << FE1;
static const uint8_t M_DOR  = 1 << DOR1;
static const uint8_t M_UPE  = 1 << UPE1;
static const uint8_t M_U2X  = 1 << U2X1;
// Bits in UCSRB.
static const uint8_t M_RXCIE = 1 << RXCIE1;
static const uint8_t M_TXCIE = 1 << TXCIE1;
static const uint8_t M_UDRIE = 1 << UDRIE1;
static const uint8_t M_RXEN  = 1 << RXEN1;
static const uint8_t M_TXEN  = 1 << TXEN1;
// Bits in UCSRC.
static const uint8_t M_UPM0 = 1 << UPM10;
static const uint8_t M_UPM1 = 1 << UPM11;
static const uint8_t M_USBS = 1 << USBS1;
static const uint8_t M_UCSZ0 = 1 << UCSZ10;
static const uint8_t M_UCSZ1 = 1 << UCSZ11;
#else  // UCSR0A
#error no serial ports
#endif  // UCSR0A
//------------------------------------------------------------------------------
/**
 * @class UsartRegister
 * @brief Addresses of USART registers.
 */
struct UsartRegister {
  volatile uint8_t* ucsra;  /**< USART Control and Status Register A. */
  volatile uint8_t* ucsrb;  /**< USART Control and Status Register B. */
  volatile uint8_t* ucsrc;  /**< USART Control and Status Register C. */
  volatile uint8_t* ubrrl;  /**< USART Baud Rate Register Low. */
  volatile uint8_t* ubrrh;  /**< USART Baud Rate Register High. */
  volatile uint8_t* udr;    /**< USART I/O Data Register. */
};
//------------------------------------------------------------------------------
/**
 * Pointers to USART registers.  This static const array allows the compiler
 * to generate very efficient code if the array index is a constant.
 */
static const UsartRegister usart[] = {
#ifdef UCSR0A
  {&UCSR0A, &UCSR0B, &UCSR0C, &UBRR0L, &UBRR0H, &UDR0},
#elif defined(UCSRA)
  {&UCSRA, &UCSRB, &UCSRC, &UBRRL, &UBRRH, &UDR},
#else  // UCSR0A
  {0, 0, 0, 0, 0, 0},
#endif  // UCSR0A

#ifdef UCSR1A
  {&UCSR1A, &UCSR1B, &UCSR1C, &UBRR1L, &UBRR1H, &UDR1},
#else  // UCSR1A
  {0, 0, 0, 0, 0, 0},
#endif  // UCSR1A

#ifdef UCSR2A
  {&UCSR2A, &UCSR2B, &UCSR2C, &UBRR2L, &UBRR2H, &UDR2},
#else  // UCSR2A
  {0, 0, 0, 0, 0, 0},
#endif  // UCSR2A

#ifdef UCSR3A
  {&UCSR3A, &UCSR3B, &UCSR3C, &UBRR3L, &UBRR3H, &UDR3}
#else  // UCSR3A
  {0, 0, 0, 0, 0, 0}
#endif  // UCSR3A
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
uint8_t badMMapLength(void)
  __attribute__((error("MMAP length too large")));
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
      if (N_ > MMAP_MAX_SIZE) badMMapLength();
    }

    void init(mmap_entry_t *mmap)
    {
      mmap_ = mmap;
    }

    /**
     * @brief Add entry to the memory map
     * 
     * @param address Parameter address
     * @param value Pointer to value
     * @param param Parameter type
     */
    inline __attribute__((always_inline))
    void setEntry(uint8_t address, uint8_t* value, uint8_t param)
    {
      mmap_[address].value = value;
      mmap_[address].param = param;
    }

    
    uint8_t setEEPROM(uint8_t address, uint8_t value)
    {
      DEBUG_PRINTLN("set eeprom");
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        eeprom_write_byte ( (uint8_t*)(uint16_t) address, value);
      }
      return (*mmap_[address].value)=value;
    }

    
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
 * @class DeviceDXL
 * @brief Virtual class for Device with DXL.
 */
class DeviceDXL {
  public:
    DeviceDXL(uint8_t id, uint8_t N):
    id_(id),
    mmap_(N)
    {}
    virtual void initRAM() {}

    virtual void initEEPROM() {}

    virtual inline __attribute__((always_inline))
    void setTX() {}

    virtual inline __attribute__((always_inline))
    void setRX() {}

    // ID
    uint8_t id_;
    // Memory mapping
    MMap mmap_;
};
//------------------------------------------------------------------------------
/**
 * @class VirtualDeviceDXL
 * @brief Virtual class for ISR.
 */
class VirtualDeviceDXL {
  public:
    virtual void process(uint8_t data) {}
};
extern VirtualDeviceDXL *SerialDXL_ISR;
//------------------------------------------------------------------------------
/**
 * @class SerialDXL
 * @brief Serial port wrapper for DXL communication protocol.
 */
#define UBRR_VALUE(BAUDRATE) (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define MAX_MSG_LENGTH 100
/** Cause error message for RX buffer bad Size.
 * @return Never returns since it is never called.
 */
uint8_t badMsgBufLength(void)
  __attribute__((error("Message buffer length too large or zero")));


template<size_t maxMsgLength>
class SerialDXL: public VirtualDeviceDXL
{
  public:
    SerialDXL():
      msgState_(0),
      msgParamIdx_(2),
      msgLen_(0),
      msgFinish_(0),
      msgChecksum_(0),
      error_()
    {
      // Check buffer sizes
      if (maxMsgLength > MAX_MSG_LENGTH || !maxMsgLength) badMsgBufLength();
      // Initialize global ptr for ISR
      SerialDXL_ISR = this;
    }

    /**
     * @brief Initialize SerialDXL. Set baudrate and target device.
     * @details 
     * 
     * @param baud Baudrate using Dynamixel format.
     * @param device Target device.
     */
    void init(uint8_t baud, DeviceDXL *device)
    {
      // Set DeviceDXL
      device_ = device;
      // Set baudrate using Dynamixel relation
      uint16_t baud_setting = UBRR_VALUE(2000000/(baud+1));

      // Set baud rate
      UBRR0H = (uint8_t)(baud_setting>>8);  // High bit
      UBRR0L = (uint8_t)baud_setting;       // Low bit

      /* USART interrupts     
      RXCIE0 | RX Complete interrupt enable
      TXCIE0 | TX Complete interrupt enable
      UDRIE0 | USART Data register empty interrupt enable (UDR0 Ready)
      RXEN0  | RX Receiver enables
      TXEN0  | TX Transmitter enable
      */

      // Set frame format to Dynamixel 8N1 (8 data bits, no parity, 1 stop bit)
      UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);

      // Enable USART interrupts
      //UCSR0B |= ((1<<TXEN0)|(1<<UDRIE0)|(1<<RXEN0)|(1<<RXCIE0));
      UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
    }

    /**
     * @brief Process data from Serial 
     * @details Add data to buffers, this function is called using ISR.
     * 
     * @param data Data from Serial port.
     */
    void process(uint8_t data)
    {
      switch(msgState_)
      {
        case 0: // 0xFF
          msgState_ = data == 0xFF ? 1 : 0;
          break;
          
        case 1: // 0XFF
          msgState_ = data == 0xFF ? 2 : 1;
          break;
          
        case 2: // ID
          // Check error
          msgState_ = data == device_->id_ ? 3 : 0;
          break;
          
        case 3: // Length
          msgLen_ = data;
          // Checksum
          msgChecksum_ += device_->id_ + data;
          // Save length in the RX message buffer
          rxMsgBuf_[0] = data;
          msgState_ = 4;
          break;

        case 4: // Instruction
          // Save instruction in the RX message buffer
          rxMsgBuf_[1] = data;
          // Checksum
          msgChecksum_ += data;
          // Check for short message
          msgState_ = msgLen_ <= 2 ? 6 : 5;
          break;
        
        case 5: // Parameters
          rxMsgBuf_[msgParamIdx_++] = data;
          // Checksum
          msgChecksum_ += data;
          // Check message length
          msgState_ = msgParamIdx_ >= msgLen_ ? 6 : msgState_;
          break;

        case 6: // Checksum
          rxMsgBuf_[msgParamIdx_] = data;
          // Check
          msgFinish_ = ((uint8_t)(~msgChecksum_))==data;
          // Reset states
          msgState_ = 0;
          msgParamIdx_ = 2;
          msgLen_ = 0;
          msgChecksum_ = 0;
          break;
      }
    }

    uint8_t* getMsg()
    {
      return (msgFinish_) ? rxMsgBuf_ : NULL;
    }

    void reset()
    {
      msgFinish_ = 0;
    }


  private:
    // DeviceDXL
    DeviceDXL *device_;
    // Mesage reception state
    uint8_t msgState_;
    uint8_t msgParamIdx_;
    uint8_t msgLen_;
    uint8_t msgFinish_;
    uint16_t msgChecksum_;

    // Error state
    uint8_t error_;

    // RX buffer
    uint8_t rxMsgBuf_[maxMsgLength];
    // TX buffer
    uint8_t txMsgBuf_[maxMsgLength];
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


#endif