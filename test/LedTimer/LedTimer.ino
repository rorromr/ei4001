#define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_DEBUG
#include <SerialDXL.h>

// LED DXL basic config
#define LED_MODEL 100
#define LED_FIRMWARE 100
#define LED_MMAP_SIZE 3 // Use 3 variables

typedef DeviceDXL<LED_MODEL, LED_FIRMWARE> LedType;


/**
 * @brief LED control using DXL communication protocol
 * @details LED control using Dynamixel communication protocol over RS485.
 * This implementation uses a 1 uint8_t variable for LED state in address 6 
 * of memory map (MMap).
 * 
 * @param dir_pin Toggle communication pin.
 * @param reset_pin Pin for reset device
 * @param led_pin LED pin.
 */
class LedDXL: public LedType
{
  public:
    LedDXL(uint8_t dataControlPin, uint8_t reset_pin, uint8_t led_pin):
    DeviceDXL(LED_MMAP_SIZE), // Call parent constructor
    reset_pin_(reset_pin),    // Reset pin
    led_pin_(led_pin),        // LED pin
    command_(MMap::Access::RW, MMap::Storage::RAM), // Led command
    test_(MMap::Access::RW, MMap::Storage::EEPROM),
    float_(MMap::Access::RW, MMap::Storage::RAM)
    {
      // Config pins
      pinMode(dataControlPin, OUTPUT);
      pinMode(reset_pin_, INPUT);
      pinMode(led_pin_, OUTPUT);

      // Get mask and port for data control pin
      dataControlPinMask_ = digitalPinToBitMask(dataControlPin);
      dataControlPinReg_ = portOutputRegister(digitalPinToPort(dataControlPin));

      pinMode(A2, OUTPUT);
      testPinMask_ = digitalPinToBitMask(A2);
      testPinReg_ = portOutputRegister(digitalPinToPort(A2));
    }

    void init()
    {
      DEBUG_PRINTLN("INIT");
      /*
       * Register variables
       */
      DeviceDXL::init();
      mmap_.registerVariable(&command_);
      mmap_.registerVariable(&float_);
      mmap_.registerVariable(&test_);
      mmap_.init();
      
      /*
       * Load default values
       */
      DEBUG_PRINTLN("Load default");
      mmap_.load(); // Load values from EEPROM
      DEBUG_PRINT("data: ");DEBUG_PRINTLN(command_.data);
      INFO_PRINT("id: ");INFO_PRINTLN_RAW(id_.data);
      
      /*
       * Read sensor data
       * e.g. Use ADC, check buttons, etc.
       */
    }

    void update()
    {
      *testPinReg_ |= testPinMask_;
      
      mmap_.deserialize();
      if (command_.data == 1) digitalWrite(led_pin_, HIGH);
      else digitalWrite(led_pin_, LOW);
      mmap_.serialize();
      
      *testPinReg_ &= ~testPinMask_;
    }

    inline bool onReset()
    {
      INFO_PRINTLN("ON RESET");
      return digitalRead(reset_pin_) == HIGH ? true : false;
    }

    inline void setTX()
    {
      *dataControlPinReg_ |= dataControlPinMask_;
    }

    inline void setRX()
    {
      *dataControlPinReg_ &= ~dataControlPinMask_;
    }

  private:
    // Communication direction pin
    uint8_t dataControlPinMask_;
    volatile uint8_t *dataControlPinReg_;

    // Test pin
    uint8_t testPinMask_;
    volatile uint8_t *testPinReg_;

    const uint8_t reset_pin_; // Reset pin
    const uint8_t led_pin_; // LED pin
    float float_raw;
    
    // LED variable
    MMap::Variable<UInt8, UInt8::type, 0, 1, 1> command_;
    MMap::Variable<UInt8, UInt8::type, 0, 255, 0> test_;

    MMap::Variable<Int32, Int32::type, -1000000, 1000000, 0> float_;
};


LedDXL led(7, 6, 13);
SerialDXL<LedDXL> serialDxl;

void setup() {
  Serial.begin(115200);
  delay(50);
  
  led.init();
  led.reset();
  led.mmap_.serialize();

  // Init serial communication using Dynamixel format
  serialDxl.init(&Serial3 ,&led);
}

void loop() {
  // Update msg buffer
  while (Serial3.available())
  {
    serialDxl.process(Serial3.read());
  } 
  led.update();
}
