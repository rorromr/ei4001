#define DEBUG
#include <SerialDXL.h>
#include <Arduino.h>

// VERSION LedDXL
#define VERSION_LED_DXL 1

// LED MMAP
#define ID      0
#define VERSION 1
#define COMMAND 2
#define STATE   3

#define LED_ID 1

/**
 * @brief LED control using DXL communication protocol
 * @details LED control using Dynamixel communication protocol over RS485.
 * This implementation uses a 4 bytes (uint8_t) memory map (MMap).
 * Example:
 * For HIGH send {0xFF,0xFF, LED_ID, 3, 0x01, 0x01}
 * For LOW send {0xFF,0xFF, LED_ID, 3, 0x01, 0x00}
 * 
 * @param id ID of device.
 * @param dir_pin Toggle communication pin.
 * @param led_pin LED pin.
 */
class LedDXL: public DeviceDXL
{
  public:
    LedDXL(uint8_t id, uint8_t dir_pin, uint8_t led_pin):
    DeviceDXL(id, 4),
    dir_pin_(dir_pin),
    led_pin_(led_pin),
    eeprom_null_(0)
    {
      // MMAP Config
      MMAP_ENTRY(mmap_field_[ID], id_, MMAP_EEPROM | MMAP_RW);               // ID
      MMAP_ENTRY(mmap_field_[VERSION], eeprom_null_, MMAP_EEPROM | MMAP_R);  // Version
      MMAP_ENTRY(mmap_field_[COMMAND], command_, MMAP_RAM | MMAP_RW);        // Current command
      MMAP_ENTRY(mmap_field_[STATE], state_, MMAP_RAM | MMAP_R);             // Current state
      mmap_.init(mmap_field_);

      // Config LED pin
      pinMode(led_pin_, OUTPUT);

    }

    void initRAM()
    {
      // Set ID from EEPROM
      mmap_.setFromEEPROM(ID);
    }

    void initEEPROM()
    {
      // Set default ID
      mmap_.setEEPROM(ID, 15);
      // Set version
      mmap_.setEEPROM(VERSION, VERSION_LED_DXL);
    }

    void reset()
    {
      initEEPROM();
    }

    void proccessMsg(uint8_t *msg)
    {
      // Instruction
      if (msg[1] == 0x01)
      {
        // Parameter
        digitalWrite(led_pin_, msg[2]);
      }
    }

    inline __attribute__((always_inline))
    void setTX()
    {
      digitalWrite(dir_pin_,HIGH);
    }

    inline __attribute__((always_inline))
    void setRX()
    {
      digitalWrite(dir_pin_,LOW);
    }

  private:
    const uint8_t dir_pin_; // Toggle communication direction pin
    const uint8_t led_pin_; // LED pin

    uint8_t eeprom_null_;

    // Fields
    uint8_t state_, command_;

    // Memory map
    mmap_entry_t mmap_field_[4];
};


LedDXL led(LED_ID, 3, 10);
SerialDXL<32> serial;

void setup() {
  // Init serial communication using Dynamixel format
  serial.init(207, &led);

  led.setRX();
  if (digitalRead(5)==HIGH)
  {
    DEBUG_PRINTLN("RESET!");
    led.reset();
  }
  led.initRAM();
}

void loop() {
  uint8_t *msg = serial.getMsg();
  if (msg)
  {
    led.proccessMsg(msg);
  }
}