#include <SimpleEncoder.h>
#include <PID_v1.h>
#include <HBridge.h>

# define MOTOR_B 8
# define MOTOR_A 7
# define MOTOR_PWM 6
# define ENCODER_A 2
# define ENCODER_B 4

// PID
double refP,inputP,outputP; //refP recibe referencia de numero de tics
double KpP = 1.3, KiP=1.0, KdP=0.01;

//controladores
PID pidP(&inputP, &outputP, &refP, KpP, KiP, KdP, DIRECT);

//Clase encoder
Encoder encoder(ENCODER_A, ENCODER_B);

// H bridge class
HBridge hbridge(MOTOR_PWM, MOTOR_A, MOTOR_B);

// Set PWM frequency  
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

  
void setup(){
  Serial.begin(9600);
  
  setPwmFrequency(6,64); //Pin 6, divido frecuencia base 62,5 Khz por 64
  
  pidP.SetSampleTime(5);
  pidP.SetOutputLimits(-255,255);
  refP = 1800;
}

void loop(){
  inputP = encoder.read();
  pidP.Compute();
  hbridge.set((int16_t) outputP);
  Serial.print("| Out ");
  Serial.println(outputP);
  delay(5);
}
