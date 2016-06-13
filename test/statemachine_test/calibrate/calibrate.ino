#include <SimpleEncoder.h>
#include <HBridge.h>
#include <Fin_de_Carrera.h>
#include <EEPROM.h>
#include <PID_v1.h>

# define MOTOR_B 8
# define MOTOR_A 7
# define MOTOR_PWM 6
# define ENCODER_A 2
# define ENCODER_B 4
# define FDC1 13
# define FDC2 12
# define FDC3 11
# define FDC4 10

# define FDC2_pos 1800 // 5 vueltas
# define FDC3_pos 0

// PID
double ref, input, output; //ref recibe referencia de numero de tics
double Kp = 1.1, Ki = 0.15, Kd = 0.007;

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
  pid.SetSampleTime(5);
  pid.SetOutputLimits(-255, 255);
  pid.setDeadZone(30);
  pid.enableDeadZone(true);
  hbridge.setPwmFrequency(64);
  hbridge.backward();
  hbridge.setPwm(200);
}

void loop() {
  Serial.println(encoder.read());
  if (fdc.getState()) {
    switch (fdc.getIndex()) {
      case 2:
        hbridge.activeBrake();
        encoder.write(FDC2_pos);
        ref = FDC2_pos - 720; //volver 2 vueltas
        while (!pid.isDeadZone()) {
          input = encoder.read();
          pid.Compute();
          hbridge.set((int16_t) output);
          Serial.println(encoder.read());
        }
        EEPROM.put(0, encoder.read());
        break;

      case 3:
        hbridge.activeBrake();
        encoder.write(FDC3_pos);
        ref = FDC3_pos + 720; //volver 2 vueltas
        while (!pid.isDeadZone()) {
          input = encoder.read();
          pid.Compute();
          hbridge.set((int16_t) output);
          Serial.println(encoder.read());
        }
        EEPROM.put(0, encoder.read());
        break;

    }

  }

}

