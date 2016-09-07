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
  #define TORSO_MMAP_SIZE 4
  
  enum statemachine {
    INIT,
    CHECK,
    ERROR,
    MOVE,
    STATE
  };
  
  enum statemachine sm;
  
  
  
  /* Pos.actual
     PWM
     Estado emergencia
     Kp, Ki, Kv, Ts, modo_discretizacion, limit
     Referencia*/
  
  
  
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
        presentSpeed_(MMap::Access::R, MMap::Storage::RAM)
      {
        // Config pins
        pinMode(dir_pin_, OUTPUT);
        pinMode(reset_pin_, OUTPUT);
        hbridge_->setPwmFrequency(64);
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
        mmap_.registerVariable(&presentPosition_);
        mmap_.registerVariable(&presentSpeed_);
  
        mmap_.init();
  
        /*
         * Load default values
         */
        DEBUG_PRINTLN("Load default");
        mmap_.load(); // Load values from EEPROM
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
            mmap_.deserialize();
            sm = CHECK;
            break;
  
          case CHECK:
            presentPosition_.data = encoder_->read();
            if (fdc_->getState()) {
              sm = ERROR;
            }
            else {
              sm = MOVE;
            }
            break;
  
          case ERROR:
            switch (fdc_->getIndex()) {
              case 1:
                mmap_.serialize();
                // Pasar a modo manual
                pid_->restart();
  
                break;
  
              case 2:
                mmap_.serialize();
                hbridge_->activeBrake(); //Cuanto seguirá subiendo por inercia?
                pid_->restart();
                goalPosition_.data = presentPosition_.data - 720;  // Solo de prueba, que baje 5 cm en realidad
                sm = MOVE;
                break;
  
              case 3:
                mmap_.serialize();
                hbridge_->activeBrake();
                pid_->restart();
                goalPosition_.data = presentPosition_.data + 720;  // Solo de prueba, que suba 5 cm en realidad
                sm = MOVE;
  
                break;
  
              case 4:
                mmap_.serialize();
                // Pasar a modo manual
                pid_->restart();
  
                break;
            }
            break;
  
          case MOVE:
  
            pid_->update(goalPosition_.data - presentPosition_.data);
            hbridge_->set( (int16_t) pid_->getOutputAntiwindup());
            presentSpeed_.data = pid_->getOutputAntiwindup();
            mmap_.serialize();
            sm = STATE;
            break;
  
          case STATE:
            delay(5);
            mmap_.deserialize();
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

  
  };
  
  
  //New PID
  double kp = 1.1;
  double kv = 0.007;
  double ki = 0.15;
  double Ts = 0.005;
  int discretization_method = 4;
  double limit = 255;
  double kaw = sqrt(ki*kv);
  
  
  //controlador
  DFILTERS::DPID pid(kp, kv, ki, Ts, discretization_method, limit, kaw);
  
  //Clase encoder
  Encoder encoder(ENCODER_A, ENCODER_B);
  
  // H bridge class
  HBridge hbridge(MOTOR_PWM, MOTOR_A, MOTOR_B);
  
  // Clase Fin de carrera
  Fin_de_Carrera fdc(FDC1, FDC2, FDC3, FDC4);
  
  // Torso Controller
  TorsoDXL torso(6, 7, &encoder, &pid, &hbridge, &fdc);
  
  SerialDXL<TorsoDXL> serialDxl;
  
  void setup() {
    Serial.begin(115200);
    delay(50);
  
    // Init serial communication using Dynamixel format
    serialDxl.init(&Serial3 , &torso);
  
    torso.init();
    torso.reset();
    torso.mmap_.serialize();
  }
  
  void loop() {
    // Update msg buffer
    while (Serial3.available())
      serialDxl.process(Serial3.read());
  
    torso.update();
  }
