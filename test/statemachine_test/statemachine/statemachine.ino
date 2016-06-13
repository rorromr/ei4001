#include <SimpleEncoder.h>
#include <PID_v1.h>
#include <HBridge.h>
#include <Fin_de_Carrera.h>
#include <EEPROM.h>


# define MOTOR_B 8
# define MOTOR_A 7
# define MOTOR_PWM 6
# define ENCODER_A 2
# define ENCODER_B 4
# define FDC1 13
# define FDC2 12
# define FDC3 11
# define FDC4 10

enum statemachine {
  INIT,
  CHECK,
  ERROR,
  MOVE,
  STATE
};

enum statemachine sm;

// PID
double ref, input, output; //ref recibe referencia de numero de tics
double Kp = 1.1, Ki = 0.11, Kd = 0.007;

long l;

//controladores
PID pid(&input, &output, &ref, Kp, Ki, Kd, DIRECT);

//Clase encoder
Encoder encoder(ENCODER_A, ENCODER_B);

// H bridge class
HBridge hbridge(MOTOR_PWM, MOTOR_A, MOTOR_B);

// Clase Fin de carrera
Fin_de_Carrera fdc(FDC1, FDC2, FDC3, FDC4);


void setup() {
  Serial.begin(115200);
  hbridge.setPwmFrequency(64); //Pin 6, divido frecuencia base 62,5 Khz por 64

  pid.SetSampleTime(5);
  pid.SetOutputLimits(-255, 255);
  pid.setDeadZone(30);
  pid.enableDeadZone(true);
  ref = 10000;

  sm = INIT;
  encoder.write(EEPROM.get(0, l)); //leo la direccion 0, buscando un long
  Serial.println(encoder.read());
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
          Serial.println(encoder.read());
          hbridge.activeBrake();
          pid.Restart();
          ref = encoder.read() - 720; //bajar 2 vueltas
          sm = MOVE;
          break;

        case 3:
          Serial.println(encoder.read());
          hbridge.activeBrake();
          pid.Restart();
          ref = encoder.read() + 720;
          sm = MOVE;

          break;

        case 4:

          break;
      }
      break;

    case MOVE:
      pid.Compute();
      hbridge.set((int16_t) output);
      sm = STATE;
      break;

    case STATE:
      // aca hay que serializar  y deserializar los datos internos
      delay(5);
      sm = CHECK;
      break;

  }

}
