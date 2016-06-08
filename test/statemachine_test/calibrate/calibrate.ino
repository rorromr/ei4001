#include <SimpleEncoder.h>
#include <HBridge.h>
#include <Fin_de_Carrera.h>
#include <EEPROM.h>

# define MOTOR_B 8
# define MOTOR_A 7
# define MOTOR_PWM 6
# define ENCODER_A 2
# define ENCODER_B 4
# define FDC1 10
# define FDC2 11
# define FDC3 12
# define FDC4 13

# define FDC2_pos 1800 // 5 vueltas
# define FDC3_pos 0

//Clase encoder
Encoder encoder(ENCODER_A, ENCODER_B);

// H bridge class
HBridge hbridge(MOTOR_PWM, MOTOR_A, MOTOR_B);

// Clase Fin de carrera
Fin_de_Carrera fdc(FDC1, FDC2, FDC3, FDC4);



//Guardo posicion en EEPROM
void savePosition() {
  long dato = encoder.read();
  byte B;
  for (int i = 0; i < sizeof(dato); i++) {
    B = highByte(dato);
    EEPROM.write(i, B);
    dato = dato << 8;
  }
}


void setup() {
  hbridge.setPwmFrequency(64);
  hbridge.forward();
  hbridge.setPwm(35);
}

void loop() {
  if (fdc.getState()) {
    switch (fdc.getIndex()) {
      case 2:
        hbridge.activeBrake();
        hbridge.backward();
        encoder.write(FDC2_pos);
        while (encoder.read() >= FDC2_pos - 720) { //volver 2 vueltas
          hbridge.setPwm(35);
        }
        hbridge.activeBrake();
        savePosition();
        break;

      case 3:
        hbridge.activeBrake();
        hbridge.forward();
        encoder.write(FDC3_pos);
        while (encoder.read() <= FDC3_pos + 720) { //volver 2 vueltas
          hbridge.setPwm(35);
        }
        hbridge.activeBrake();
        savePosition();
        break;

    }

  }

}

