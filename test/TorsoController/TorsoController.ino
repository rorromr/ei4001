#include <SerialDXL.h>
#include <SimpleEncoder.h>
#include <HBridge.h>
#include <Fin_de_Carrera.h>
#include <PID_v1.h>

// Torso basic config
#define TORSO_MODEL 100
#define TORSO_FIRMWARE 100

/**
 * DEVICEDXL_MIN_BUFFER = 6
 * Is the overhead for dynamixel like devices
 * Has MODEL_L, MODEL_H, FIRMWARE, ID, BAUDRATE and RETURN DELAY
 * This variables are inside DeviceDXL class
 */ 

// We add 2 UInt8 (2 bytes)
#define TORSO_MMAP_SIZE DEVICEDXL_MIN_BUFFER+1

// MMap position for command
#define TORSO_COMMAND DEVICEDXL_MIN_BUFFER

/**
 * @brief Torso controller using DXL communication protocol
 * @details Torso controller using Dynamixel communication protocol over RS485.
 * This implementation uses a 1 uint8_t variable for Torso reference in address 6 
 * of memory map (MMap).
 * 
 * @param dir_pin Toggle communication pin.
 * @param reset_pin Pin for reset device
 */
class TorsoDXL: public DeviceDXL
{
  public:
    TorsoDXL(uint8_t dir_pin, uint8_t reset_pin):
    DeviceDXL(TORSO_MODEL, TORSO_FIRMWARE, TORSO_MMAP_SIZE), // Call parent constructor
    dir_pin_(dir_pin),    // Direction pin for RS485
    reset_pin_(reset_pin), // Reset pin
    command_(TORSO_COMMAND, MMap::Access::RW, 0U, 1U, 1U) // Torso command
    {
      // Config pins
      pinMode(dir_pin_, OUTPUT);
      pinMode(reset_pin_, OUTPUT);
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
      /*
      *
      * Run PID
      *
      */
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
    
    // Torso command
    MMap::UInt8 command_;
    
    // Memory map
    uint8_t msgBuf_[TORSO_MMAP_SIZE];
    MMap::VariablePtr varList_[TORSO_MMAP_SIZE];
};


TorsoDXL torso(6, 7);
SerialDXL serialDxl;

void setup() {
  Serial.begin(115200);
  delay(50);
  
  // Init serial communication using Dynamixel format
  serialDxl.init(115200, &Serial3 ,&torso);

  torso.init();
  torso.reset();
  torso.mmap_.serialize();
}

void loop() {
  // Update msg buffer
  while (Serial3.available())
    serialDxl.process(Serial3.read());

  torso.mmap_.deserialize();
  torso.update();
  torso.mmap_.serialize();
}
