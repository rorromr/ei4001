#include <SerialDXL.h>

// LED DXL basic config
#define LED_MODEL 100
#define LED_FIRMWARE 100
#define LED_MMAP_SIZE 2

// MMap position for command
#define LED_COMMAND 6

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
    LedDXL(uint8_t dir_pin, uint8_t reset_pin, uint8_t led_pin):
    DeviceDXL(LED_MODEL, LED_FIRMWARE, LED_MMAP),
    dir_pin_(dir_pin),
    reset_pin_(rest_pin),
    led_pin_(led_pin),
    {
      
      /*
      * MMAP Config
      * LED settings
      */
      // State
      MMAP_ENTRY(mmap_field_[STATE], state_, MMAP_RAM | MMAP_R);
      // Command
      MMAP_ENTRY(mmap_field_[COMMAND], command_, MMAP_RAM | MMAP_RW);
      
      // Init MMAP
      mmap_.init(mmap_field_);

      // Config LED pin

      pinMode(dir_pin_, OUTPUT);
    }

    void reset()
    {
      initEEPROM();
    }

    void proccessMsg(uint8_t *msg)
    {
      // Ping
      if (msg[1] == 0x02)
      {
        // Parameter
        digitalWrite(led_pin_, msg[2]);
        state_ = msg[2];
      }

    }

    void update()
    {
      if (command_ == 1)
      {
        digitalWrite(led_pin_, HIGH);
        state_ = analogRead(A0)/4;
      }
      else if (command_ == 0)
      {
        digitalWrite(led_pin_, LOW);
        state_ = analogRead(A0)/4;
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
    mmap_entry_t mmap_field_[LED_MMAP_SIZE];

};


LedDXL led(LED_ID, 3, 10);
SerialDXL serialDxl;

void setup() {
  // Init serial communication using Dynamixel format
  serialDxl.init(9600, &Serial1 ,&led);

  led.setRX();
  if (digitalRead(5)==HIGH)
  {
    DEBUG_PRINTLN("RESET DXL DEVICE");
    led.reset();
  }
  led.initRAM();
}

void loop() {
  // Update msg buffer
  if (Serial1.available())
    serialDxl.process(Serial1.read());
    
  uint8_t *msg = serialDxl.getMsg();
  if (msg)
  {
    led.proccessMsg(msg);
  }
}
