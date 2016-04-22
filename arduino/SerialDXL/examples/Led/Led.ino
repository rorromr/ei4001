#include <SerialDXL.h>

// LED DXL basic config
#define LED_MODEL 100
#define LED_FIRMWARE 100
// We add 2 UInt8 (2 bytes)
#define LED_MMAP_SIZE DEVICEDXL_MIN_BUFFER+1

// MMap position for command
#define LED_COMMAND DEVICEDXL_MIN_BUFFER

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
    DeviceDXL(LED_MODEL, LED_FIRMWARE, LED_MMAP_SIZE), // Call parent constructor
    dir_pin_(dir_pin),    // Direction pin for RS485
    reset_pin_(reset_pin), // Reset pin
    led_pin_(led_pin),    // LED pin
    command_(LED_COMMAND, MMap::Access::RW, 0U, 1U, 1U) // Led command
    {
      // Config pins
      pinMode(dir_pin_, OUTPUT);
      pinMode(reset_pin_, OUTPUT);
      pinMode(led_pin_, OUTPUT);
    }

    void init()
    {
      DEBUG_PRINTLN("INIT");
      /*
       * Register variables
       */
      DeviceDXL::init(msgBuf_, varList_);
      mmap_.registerVariable(&command_);
      
      /*
       * Load default values
       */
      DEBUG_PRINTLN("Load default");
      mmap_.load(); // Load values from EEPROM
      DEBUG_PRINT("data: ");DEBUG_PRINTLN(command_.data);
      
      /*
       * Read sensor data
       * e.g. Use ADC, check buttons, etc.
       */
    }

    void update()
    {
      DEBUG_PRINTLN("UPDATE");
      DEBUG_PRINT("data: ");DEBUG_PRINTLN(command_.data);
      if (command_.data == 1) digitalWrite(led_pin_, HIGH);
      else digitalWrite(led_pin_, LOW);
    }

    inline bool onReset()
    {
      DEBUG_PRINTLN("ON RESET");
      return digitalRead(reset_pin_) == HIGH ? true : false;
    }

    inline void setTX()
    {
      digitalWrite(dir_pin_,HIGH);
    }

    inline void setRX()
    {
      digitalWrite(dir_pin_,LOW);
    }

  private:
    const uint8_t dir_pin_; // Toggle communication direction pin
    const uint8_t reset_pin_; // Reset pin
    const uint8_t led_pin_; // LED pin
    
    // LED variable
    MMap::UInt8 command_;
    
    // Memory map
    uint8_t msgBuf_[LED_MMAP_SIZE];
    MMap::VariablePtr varList_[LED_MMAP_SIZE];
};


LedDXL led(6, 7, 13);
SerialDXL serialDxl;

void setup() {
  Serial.begin(115200);
  delay(50);
  
  // Init serial communication using Dynamixel format
  serialDxl.init(9600, &Serial1 ,&led);

  led.init();
  led.mmap_.serialize();
}

void loop() {
  // Update msg buffer
  while (Serial1.available())
    serialDxl.process(Serial1.read());

  led.mmap_.deserialize();
  led.update();
  led.mmap_.serialize();
  
  delay(500);
}
