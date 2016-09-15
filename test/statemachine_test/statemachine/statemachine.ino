#include <SimpleEncoder.h>
#include <DPID.h>
#include <HBridge.h>
#include <Fin_de_Carrera.h>
#include <EEPROM.h>


# define MOTOR_PWM_R 9
# define MOTOR_PWM_L 8
# define ENCODER_A 2
# define ENCODER_B 4
# define FDC1 28
# define FDC2 30
# define FDC3 32
# define FDC4 34


# define FDC2_pos 4000 // 5 vueltas
# define FDC3_pos 0


enum statemachine {
  INIT,
  CHECK,
  ERROR,
  MOVE,
  STATE
};

enum statemachine sm;

//New PID
double kp = 0.9;
double kv = 0.15;
double ki = 0.15;
double Ts = 0.005;
int discretization_method = 4;
double limit = 255;
double kaw = sqrt(ki*kv);  //backCalculation

double ref;
double input;

long l;

//controlador
DFILTERS::DPID pid(kp,kv,ki,Ts,discretization_method,limit,kaw);

//Clase encoder
Encoder encoder(ENCODER_A, ENCODER_B);

// H bridge class

HBridge hbridge(MOTOR_PWM_R, MOTOR_PWM_L);

// Clase Fin de carrera
Fin_de_Carrera fdc(FDC1, FDC2, FDC3, FDC4);


void setup() {
  Serial.begin(115200);
  hbridge.setPwmFrequency(64); //Pin 6, divido frecuencia base 62,5 Khz por 64

  pid.setDeadZone(30);
  pid.enableDeadZone(true);
  ref = 5000;

  sm = INIT;
  //Serial.println(encoder.read());
  sm = CHECK;

}

void loop() {
  Serial.println(encoder.read());
  switch (sm) {
    case CHECK:
      input = encoder.read();
      if (fdc.getState()) {
        sm = ERROR;
      }
      else {
        sm = MOVE;
      }
      break;

    case ERROR:
      switch (fdc.getIndex()) {
        case 1:

          break;

        case 2:
          //Serial.println(encoder.read());
          hbridge.activeBrake(); //Cuanto seguirá subiendo por inercia?
          pid.restart();
          encoder.write(FDC2_pos);
          ref = encoder.read() - 720; //bajar 2 vueltas
          sm = MOVE;
          break;

        case 3:
          //Serial.println(encoder.read());
          hbridge.activeBrake();
          pid.restart();
          encoder.write(FDC3_pos);
          ref = encoder.read() + 720;
          sm = MOVE;

          break;

        case 4:

          break;
      }
      break;

    case MOVE:

      pid.update(ref - encoder.read());
      hbridge.set( (int16_t) pid.getOutputAntiwindup());
      sm = STATE;
      break;

    case STATE:
      // aca hay que serializar  y deserializar los datos internos
      delay(5);
      sm = CHECK;
      break;

  }

}
