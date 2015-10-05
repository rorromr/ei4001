#include <SerialDXL.h>

uint8_t data;      // variable to read incoming serial data into

uint8_t state = 0;
uint8_t len = 0;
uint8_t instruction = 0;


uint8_t parameters[8]; 
uint8_t param_index = 0;

uint8_t finish = 0;

uint8_t id = 30;

class LedDXL: public DeviceDXL
{
  public:
    LedDXL(const uint8_t dir_pin)
    {
      dir_pin_  = dir_pin;
    }

    void initRAM()
    {
      ;
    }

    void initEEPROM()
    {
      ;
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
    uint8_t dir_pin_;
  
};

mmap_entry_t led_mmap_entry[] = {
  { &id, MMAP_EEPROM | MMAP_RW },
  { &id, MMAP_EEPROM | MMAP_RW }
};

MMap<5> led_mmap(led_mmap_entry);

LedDXL led(3);

void setup() {
  // Serial communication:
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  led.setRX();
}

void loop() {
  
  if (Serial.available() > 0) {
    // Read serial
    data = (uint8_t) Serial.read();
    finish = 0;
    switch(state)
    {
      case 0: // 0xFF
        if (data == 0xFF) state = 1;
        break;
        
      case 1: // 0XFF
        if (data == 0xFF) state = 2;
        break;
        
      case 2: // ID
        // Check error
        if (data == 0xFF)
        {
          state = 0;
          break;
        }
        
        if (data == id)
        {
          state = 3;
        }
        break;
        
      case 3: // Length
        len = data;
        state = 4;
        break;

      case 4: // Instruction
        instruction = data;
        state = 5;
        if (len <= 1) state = 6; else state = 5;                      
        break;
      
      case 5: // Parameters
        parameters[param_index++] = data;
        if (param_index >= len-2) state = 6;
        break;

      case 6: // Checksum
        state = 0;
        param_index = 0;
        finish = 1;
        break;
    }
  }

  if (finish)
  {
    if (parameters[0] == 'H') digitalWrite(10, HIGH);
    else if (parameters[0] == 'L') digitalWrite(10, LOW);
  }
}
