#ifndef SerialDXL_h
#define SerialDXL_h

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <Arduino.h>

// Debug macros
#if(DEBUG == 0)
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#else
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#endif
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

};

// Memory map max size
static const uint8_t MMAP_MAX_SIZE = 16;

// First bit LSB
static const uint8_t MMAP_RW = 1;
static const uint8_t MMAP_R = 0;

// Second bit
static const uint8_t MMAP_RAM = 1 << 1;
static const uint8_t MMAP_EEPROM = 0;

/** Cause error message for bad Size.
 * @return Never returns since it is never called.
 */
uint8_t badSize(void)
  __attribute__((error("MMAP size too large")));


typedef struct
{
  uint8_t *value;
  uint8_t param;
} mmap_entry_t;

// Set MMAP entry macro
#define MMAP_ENTRY(mmap, var, parameter) {(mmap).value = &(var); (mmap).param = (parameter);}

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
        eeprom_write_byte ( (uint8_t*) address, value);
        return (*mmap_[address].value)=value;
      }
    }

    inline __attribute__((always_inline))
    uint8_t getEEPROM(uint8_t address)
    {
      DEBUG_PRINTLN("get eeprom");
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        return eeprom_read_byte( (uint8_t*) address );
      }
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
        value = eeprom_read_byte( (uint8_t*) address );
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

/**
 * @class SerialDXL
 * @brief Serial port wrapper for DXL communication protocol.
 */
class SerialDXL
{
  public:
    SerialDXL()
    {
      ;
    }

};

#endif