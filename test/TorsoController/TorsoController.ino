#include <SerialDXL.h>
#include <SimpleEncoder.h>
#include <HBridge.h>
#include <Fin_de_Carrera.h>
#include <DPID.h>

# define MOTOR_B 8
# define MOTOR_A 7
# define MOTOR_PWM 6
# define ENCODER_A 2
# define ENCODER_B 4
# define FDC1 13
# define FDC2 12
# define FDC3 11
# define FDC4 10

// Torso basic config
#define TORSO_MODEL 100
#define TORSO_FIRMWARE 100
#define TORSO_MMAP_SIZE 10 



/* Pos.actual
   PWM
   Estado emergencia
   Kp, Ki, Kv, Ts, modo_discretizacion, limit
   Referencia*/

   //New PID
double kp = 1.1;
double kv = 0.007;
double ki = 0.15;
double Ts = 0.005;
int discretization_method = 4;
double limit = 255;
double kaw = sqrt(ki*kv);

/**
 * @brief Torso controller using DXL communication protocol
 * @details Torso controller using Dynamixel communication protocol over RS485.
 * This implementation uses a 1 uint8_t variable for Torso reference in address 6 
 * of memory map (MMap).
 * 
 * @param dir_pin Toggle communication pin.
 * @param reset_pin Pin for reset device
 */
class TorsoDXL: public DeviceDXL<TORSO_MODEL, TORSO_FIRMWARE>
{
  public:
    TorsoDXL(uint8_t dir_pin, uint8_t reset_pin, Encoder *encoder, DPID *pid, HBridge *hbridge, Fin_de_Carrera *fdc):
    DeviceDXL(TORSO_MMAP_SIZE), // Call parent constructor
    dir_pin_(dir_pin),    // Direction pin for RS485
    reset_pin_(reset_pin), // Reset pin
    encoder_(encoder), //Puntero a objetos
    pid_(pid),
    hbridge_(hbridge),
    fdc_(fdc),
    setRef_command_(MMap::Access::RW, MMap::Storage::RAM) // Torso command
    getPos_command_(MMap::Access::R, MMap::Storage::EEPROM)
    {
      // Config pins
      pinMode(dir_pin_, OUTPUT);
      pinMode(reset_pin_, OUTPUT);
      hbridge_.setPwmFrequency(64);
      pid_.setDeadZone(30);
      pid_.enableDeadZone(true);
    }

    void init()
    {
      DEBUG_PRINTLN("INIT");
      /*
       * Register variables
       */
      DeviceDXL::init();
      mmap_.registerVariable(&setRef_command_);
      mmap_.registerVariable(&getPos_command_);
      mmap_.init();
      
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

    Encoder *encoder_;
    DPID *pid_;
    HBridge *hbridge_;
    Fin_de_Carrera *fdc_;
    
    // Torso command
    MMap::Variable<UInt16, UInt16::type, 0, 57600, 0> setRef_command_;
    MMap::Variable<UInt16, UInt16::type, 0, 57600, 0> getPos_command_;

    
};


//controlador
DFILTERS::DPID pid(kp,kv,ki,Ts,discretization_method,limit,kaw);

//Clase encoder
Encoder encoder(ENCODER_A, ENCODER_B);

// H bridge class
HBridge hbridge(MOTOR_PWM, MOTOR_A, MOTOR_B);

// Clase Fin de carrera
Fin_de_Carrera fdc(FDC1, FDC2, FDC3, FDC4);

TorsoDXL torso(6, 7, encoder,pid,hbridge,fdc);

SerialDXL serialDxl;

void setup() {
  Serial.begin(115200);
  delay(50);
  
  // Init serial communication using Dynamixel format
  serialDxl.init(&Serial3 ,&torso);

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
