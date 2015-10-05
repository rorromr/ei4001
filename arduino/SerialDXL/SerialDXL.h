#ifndef SerialDXL_h
#define SerialDXL_h

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <Arduino.h>

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

// First bit
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

/**
 * @class MMap
 * @brief Memory mapping.
 */
template<size_t N>
class MMap
{
  public:
    MMap(mmap_entry_t *mmap)
    {
      // Check size
      if (N > MMAP_MAX_SIZE) badSize();
      // 
      mmap_ = mmap;
    }

    inline __attribute__((always_inline))
    void setEEPROM(uint8_t address, uint8_t value)
    {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        eeprom_write_byte ( (uint8_t*) address, value);
        (*mmap_[address].value)=value;
      }
    }

    inline __attribute__((always_inline))
    uint8_t getEEPROM(uint8_t address)
    {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        return eeprom_read_byte( (uint8_t*) address );
      }
    }

    void set(uint8_t address, uint8_t value)
    {
      // Check for size
      if (address > N) return;
      // Check for access
      if (~(mmap_[address].param & MMAP_RW)) return;


      mmap_[address].param & MMAP_RAM ? 
        (*mmap_[address].value)=value : setEEPROM(address, value);
    }

    uint8_t get(uint8_t address)
    {
      // Check for size
      if (address > N) return 0;
      return mmap_[address].param & MMAP_RAM ? 
        (*mmap_[address].value) : getEEPROM(address);
    }


  private:
    // Memory mapping with pointers to RAM and EEPROM data
    mmap_entry_t *mmap_;
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