
#ifndef MMap_h
#define MMap_h
//------------------------------------------------------------------------------
#include <avr/eeprom.h>
#include <util/atomic.h>
//------------------------------------------------------------------------------
/** Memory map max size */
static const uint8_t MMAP_MAX_SIZE = 64U;
/** First bit LSB */
static const uint8_t MMAP_RW = 1U;
static const uint8_t MMAP_R = 0U;
/** Second bit */
static const uint8_t MMAP_RAM = 1U << 1U;
static const uint8_t MMAP_EEPROM = 0U;


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
// Saturation function
#define MMAP_SAT(x, min, max) ( ((x) > max) ? max : ( ((x) < min) ? min : (x) ) )
//------------------------------------------------------------------------------
namespace MMap
{

enum
{
  RW = 0U,
  RO = 1U
};
//------------------------------------------------------------------------------
/**
 * @class Variable
 * @brief Virtual base class for MMap variables.
 */
class Variable
{
  public:
    Variable(uint8_t param = RW): param_(param) {};
    virtual uint8_t serialize(uint8_t *outbuffer) const =0;
    virtual uint8_t deserialize(uint8_t *inbuffer) =0;
  protected:
    uint8_t param_;
};

/**
 * @class Variable
 * @brief Virtual base class for MMap non-volatile variables.
 */
class VariableNV : public Variable
{
  public:
    VariableNV(uint8_t address, uint8_t param = RW): Variable(param), address_(address) {};
    virtual uint8_t serialize(uint8_t *outbuffer) const =0;
    virtual uint8_t deserialize(uint8_t *inbuffer) =0;
    virtual void load() =0;
    virtual void save(void *data) const =0;
  protected:
    uint8_t address_;
};

/**
 * @class UInt8
 * @brief MMap variable for uint8_t.
 */
class UInt8: public Variable
{
  public:
    uint8_t data;
    UInt8(uint8_t param = RW, uint8_t min = 0U, uint8_t max = 255U, uint8_t def = 0U):
      Variable(param), min_(min), max_(max), def_(def), data(def) {}
    
    virtual uint8_t serialize(uint8_t *outbuffer) const
    {
      uint8_t offset = 0;
      *(outbuffer + offset + 0) = (this->data >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual uint8_t deserialize(uint8_t *inbuffer)
    {
      // Check for read only
      if (param_ == RO)
        return sizeof(this->data);

      uint8_t offset = 0;
      // Get data from buffer
      this->data = MMAP_SAT((uint8_t) (*(inbuffer + offset)), this->min_, this->max_);
      offset += sizeof(this->data);
      return offset;
    }
  private:
    const uint8_t min_, max_, def_;
};

/**
 * @class UInt8NV
 * @brief MMap variable for non-volatile uint8_t (saved on EEPROM).
 */
class UInt8NV: public VariableNV
{
  public:
    uint8_t data;
    UInt8NV(uint8_t address, uint8_t param = RW, uint8_t min = 0U, 
      uint8_t max = 255U, uint8_t def = 0U):
      VariableNV(address, param), min_(min), max_(max), def_(def), data(def) {}
    
    virtual uint8_t serialize(uint8_t *outbuffer) const
    {
      uint8_t offset = 0;
      *(outbuffer + offset + 0) = (this->data >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual uint8_t deserialize(uint8_t *inbuffer)
    {
      // Check for read only
      if (param_ == RO)
        return sizeof(this->data);

      uint8_t offset = 0;
      // Get data from buffer
      uint8_t updated_data = ((uint8_t) (*(inbuffer + offset)));
      // Only update EEPROM when value has changed
      if (this->data != updated_data)
      {
        eeprom_write_byte ( (uint8_t*)(uint16_t) address_, updated_data);
        this->data = updated_data;
      }
      offset += sizeof(this->data);
      return offset;
    }

    virtual void load()
    {
      // Get data from EEPROM
      this->data = eeprom_read_byte( (uint8_t*)(uint16_t) address_);
    }

    virtual void save(void *data = NULL) const
    {
      // Load default or get data from ptr
      uint8_t updated_data = data == NULL ? def_ : *(uint8_t*)data;
      eeprom_write_byte ( (uint8_t*)(uint16_t) address_, updated_data);
    }
  private:
    const uint8_t min_, max_, def_;
};


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

}; // End MMap class
//------------------------------------------------------------------------------
} // End MMap namespace




#endif
