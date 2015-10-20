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


class LedDXL: public DeviceDXL
{
  public:
    LedDXL(uint8_t dir_pin, uint8_t led_pin):
    dir_pin_(dir_pin),
    led_pin_(led_pin),
    eeprom_null_(0),
    mmap_(4)
    {
      // MMAP Config

      MMAP_ENTRY(mmap_field_[ID], id_, MMAP_EEPROM | MMAP_RW);               // ID
      MMAP_ENTRY(mmap_field_[VERSION], eeprom_null_, MMAP_EEPROM | MMAP_R);  // Version
      MMAP_ENTRY(mmap_field_[COMMAND], command_, MMAP_RAM | MMAP_RW);        // Current command
      MMAP_ENTRY(mmap_field_[STATE], state_, MMAP_RAM | MMAP_R);             // Current state
      mmap_.setMMap(mmap_field_);

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
      if (*msg == 'H') digitalWrite(led_pin_, HIGH);
      else if (*msg == 'L') digitalWrite(led_pin_, LOW);
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
    MMap mmap_;
};



LedDXL led(3, 10);
SerialDXL<32> serial;

void setup() {
  // Serial communication:
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