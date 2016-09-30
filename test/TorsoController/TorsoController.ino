  #include <SerialDXL.h>
  #include <SimpleEncoder.h>
  #include <HBridge.h>
  #include <Fin_de_Carrera.h>
  #include <DPID.h>
  #include "DigitalIO.h"
  #include <TimerThree.h>
  
  # define MOTOR_PWM_R 9
  # define MOTOR_PWM_L 8
  # define ENCODER_A 2
  # define ENCODER_B 4
  # define FDC1 28
  # define FDC2 30
  # define FDC3 32
  # define FDC4 34

  # define FDC2_pos 5000 
  # define FDC3_pos 0
  
  // Torso basic config
  #define TORSO_MODEL 100
  #define TORSO_FIRMWARE 100
  #define TORSO_MMAP_SIZE 9

  DigitalPin<13> pin13;
  
  enum statemachine {
    INIT,
    CHECK,
    ERROR,
    MOVE,
    STATE
  };
  
  enum statemachine sm;
  

  DeviceDXL<TORSO_MODEL, TORSO_FIRMWARE>* mainDevice = NULL;
  
  //static inline __attribute__((always_inline))
  void controlLoopUpdate()
  {
    if (mainDevice == NULL) return;
    mainDevice->update();
  }
  
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
      TorsoDXL(uint8_t dir_pin, uint8_t reset_pin, Encoder *encoder, DFILTERS::DPID *pid, HBridge *hbridge, Fin_de_Carrera *fdc):
        DeviceDXL(TORSO_MMAP_SIZE), // Call parent constructor
        dir_pin_(dir_pin),    // Direction pin for RS485
        reset_pin_(reset_pin), // Reset pin
        encoder_(encoder), //Puntero a objetos
        pid_(pid),
        hbridge_(hbridge),
        fdc_(fdc),
        goalPosition_(MMap::Access::RW, MMap::Storage::RAM),  // Torso command
        movingSpeed_(MMap::Access::RW, MMap::Storage::RAM),
        presentPosition_(MMap::Access::R, MMap::Storage::RAM),
        presentSpeed_(MMap::Access::R, MMap::Storage::RAM),
        lastPosition_(MMap::Access::R, MMap::Storage::RAM),
        kp_(MMap::Access::RW, MMap::Storage::RAM),
        ki_(MMap::Access::RW, MMap::Storage::RAM),
        kv_(MMap::Access::RW, MMap::Storage::RAM),
        limits_(MMap::Access::RW, MMap::Storage::RAM),
        emergencyState_(MMap::Access::R, MMap::Storage::RAM)
      {
        // Config pins
        pinMode(dir_pin_, OUTPUT);
        pinMode(reset_pin_, OUTPUT);
        //hbridge_->setPwmFrequency(64);
        pid_->setDeadZone(30);
        pid_->enableDeadZone(true);
      }
  
      void init()
      {
        
        DEBUG_PRINTLN("INIT");
        /*
         * Register variables
         */
        DeviceDXL::init();
        mmap_.registerVariable(&goalPosition_);
        mmap_.registerVariable(&movingSpeed_);
        mmap_.registerVariable(&emergencyState_);
        mmap_.registerVariable(&presentPosition_);
        mmap_.registerVariable(&presentSpeed_);
        mmap_.registerVariable(&lastPosition_);
        mmap_.registerVariable(&kp_);
        mmap_.registerVariable(&ki_);
        mmap_.registerVariable(&kv_);
        mmap_.registerVariable(&limits_);
        
        
  
        mmap_.init();
  
        /*
         * Load default values
         */
        DEBUG_PRINTLN("Load default");
        mmap_.load(); // Load values from EEPROM
        presentPosition_.data = lastPosition_.data;
        //DEBUG_PRINT("data: ");DEBUG_PRINTLN(command_.data);
  
        /*
         * Read sensor data
         * e.g. Use ADC, check buttons, etc.
         */
      }
  
      void update()
      {
        switch (sm) {
          case INIT:
            noInterrupts();
            mmap_.deserialize();
            interrupts();
            sm = CHECK;
            break;
  
          case CHECK:
            presentPosition_.data = encoder_->read();
            pid_->setLimit(limits_.data);
            if (fdc_->getState()) {
              emergencyState_.data = 1;
              sm = ERROR;
            }
            else {
              emergencyState_.data = 0;
              sm = MOVE;
            }
            break;
  
          case ERROR:
            switch (fdc_->getIndex()) {
              case 1:
                hbridge_->activeBrake();
                // Pasar a modo manual
                pid_->restart();
  
                break;
  
              case 2:
                hbridge_->activeBrake(); //Cuanto seguirá subiendo por inercia?
                pid_->restart();
                encoder_->write(FDC2_pos);
                presentPosition_.data = FDC2_pos;
                goalPosition_.data = presentPosition_.data - 720;  // Solo de prueba, que baje 5 cm en realidad
                sm = MOVE;
                break;
  
              case 3:
                hbridge_->activeBrake();
                pid_->restart();
                encoder_->write(FDC3_pos);
                presentPosition_.data = FDC3_pos;
                goalPosition_.data = presentPosition_.data + 720;  // Solo de prueba, que suba 5 cm en realidad
                sm = MOVE;
  
                break;
  
              case 4:
                hbridge_->activeBrake();
                // Pasar a modo manual
                pid_->restart();
  
                break;
            }
            break;
  
          case MOVE:
  
            pid_->update(goalPosition_.data - presentPosition_.data);
            hbridge_->set( (int16_t) pid_->getOutputAntiwindup());
            presentSpeed_.data = (int8_t) pid_->getOutputAntiwindup();
            emergencyState_.data = pid_->isDeadZone() ? 0 : 1;
            sm = emergencyState_.data == 1? MOVE : STATE; 
            break;
  
          case STATE:
            //delay(5);
            noInterrupts();
            mmap_.deserialize();
            interrupts();
            sm = CHECK;
            break;
  
        }
  
      }
  
      inline bool onReset()
      {
        DEBUG_PRINTLN("ON RESET");
        return digitalRead(reset_pin_) == HIGH ? true : false;
      }
  
      inline void setTX()
      {
        digitalWrite(dir_pin_, HIGH);
      }
  
      inline void setRX()
      {
        digitalWrite(dir_pin_, LOW);
      }
  
    private:
      const uint8_t dir_pin_; // Toggle communication direction pin
      const uint8_t reset_pin_; // Reset pin
  
      Encoder *encoder_;
      DFILTERS::DPID *pid_;
      HBridge *hbridge_;
      Fin_de_Carrera *fdc_;
  
      // Torso command
      MMap::Variable<UInt16, UInt16::type, 0, 57600, 0> goalPosition_;
      MMap::Variable<UInt8, UInt8::type, 0, 255, 0> movingSpeed_;
  
      MMap::Variable<UInt16, UInt16::type, 0, 57600, 0> presentPosition_;
      MMap::Variable<UInt8, UInt8::type, 0, 255, 0> presentSpeed_;
      MMap::Variable<UInt16, UInt16::type, 0, 57600, 0> lastPosition_;
      MMap::Variable<Int32, Int32::type, -1000000, 1000000, 0> kp_;
      MMap::Variable<Int32, Int32::type, -1000000, 1000000, 0> ki_;
      MMap::Variable<Int32, Int32::type, -1000000, 1000000, 0> kv_;
      MMap::Variable<UInt8, UInt8::type, 0, 255, 0> limits_;

      MMap::Variable<UInt8, UInt8::type, 0, 1, 0> emergencyState_;

  
  };
  
  
  //New PID
  float kp = 0.9;
  float kv = 0.15;
  float ki = 0.15;
  float Ts = 0.005;
  int discretization_method = 4;
  float limit = 255;
  float kaw = sqrt(ki*kv);
    
  
  //controlador

  DFILTERS::DPID pid(kp, kv, ki, Ts, discretization_method, limit, kaw);
  
  //Clase encoder
  Encoder encoder(ENCODER_A, ENCODER_B);
  
  // H bridge class
  HBridge hbridge(MOTOR_PWM_R, MOTOR_PWM_L);
  
  // Clase Fin de carrera
  Fin_de_Carrera fdc(FDC1, FDC2, FDC3, FDC4);
  
  // Torso Controller
  TorsoDXL torso(7, 6, &encoder, &pid, &hbridge, &fdc);

  SerialDXL<TorsoDXL> serialDxl;  
  
  void setup() {
    mainDevice = &torso;
    Serial.begin(115200);
    pin13.mode(OUTPUT);
  
    // Init serial communication using Dynamixel format
    serialDxl.init(&Serial3 , &torso);
  
    torso.init();
    torso.reset();
    noInterrupts();
    torso.mmap_.serialize();
    interrupts();

    Timer3.initialize(1000); // 1000 us, 1 khz
    Timer3.attachInterrupt(controlLoopUpdate);
    
  }
  
  void loop() {
    // Update msg buffer
    pin13.high();
    while (Serial3.available())
      serialDxl.process(Serial3.read());

    noInterrupts();
    torso.mmap_.serialize();
    interrupts();
    pin13.low();

    
  }
