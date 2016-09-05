#include <SimpleEncoder.h>
#include <HBridge.h>
#include <Fin_de_Carrera.h>
#include <EEPROM.h>
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

# define FDC2_pos 1800 // 5 vueltas
# define FDC3_pos 0

//New PID
double kp = 1.1;
double kv = 0.007;
double ki = 0.15;
double Ts = 0.005;
int discretization_method = 4;
double limit = 255;
double kaw = sqrt(ki*kv);  //backCalculation

//controladores
DFILTERS::DPID pid(kp,kv,ki,Ts,discretization_method,limit,kaw);

//Clase encoder
Encoder encoder(ENCODER_A, ENCODER_B);

// H bridge class
HBridge hbridge(MOTOR_PWM, MOTOR_A, MOTOR_B);

// Clase Fin de carrera
Fin_de_Carrera fdc(FDC1, FDC2, FDC3, FDC4);


void setup() {
  Serial.begin(115200);

  pid.setDeadZone(30);
  pid.enableDeadZone(true);
  hbridge.setPwmFrequency(64);


/**
Una vez se haya roseado, debe poder recibir un comando que indique en que direccion 
quiere calibrarse (arriba o abajo) y a que velocidad quiere ir*/
  hbridge.backward();
  hbridge.setPwm(200);
}

void loop() {
  //Serial.println(encoder.read());
  if (fdc.getState()) {
    switch (fdc.getIndex()) {
      case 2:
        hbridge.activeBrake();
        encoder.write(FDC2_pos);
        ref = FDC2_pos - 720; //volver 2 vueltas
        while (!pid.isDeadZone()) {
          pid.update(ref - encoder.read());
          hbridge.set( (int16_t) pid.getOutputAntiwindup());
        }
        EEPROM.put(0, encoder.read());
        break;

      case 3:
        hbridge.activeBrake();
        encoder.write(FDC3_pos);
        ref = FDC3_pos + 720; //volver 2 vueltas
        while (!pid.isDeadZone()) {
          pid.update(ref - encoder.read());
          hbridge.set( (int16_t) pid.getOutputAntiwindup());
        }
        EEPROM.put(0, encoder.read());
        break;

    }

  }

}



