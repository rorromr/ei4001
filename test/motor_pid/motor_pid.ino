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
double KpP = 1.3, KiP=1.0, KdP=0.005;

//controladores
PID pidP(&inputP, &outputP, &refP, KpP, KiP, KdP, DIRECT);

//Clase encoder
Encoder encoder(ENCODER_A, ENCODER_B);

// H bridge class
HBridge hbridge(MOTOR_PWM, MOTOR_A, MOTOR_B);

  
void setup(){
  Serial.begin(9600);
  
  hbridge.setPwmFrequency(64); //Pin 6, divido frecuencia base 62,5 Khz por 64
  
  pidP.SetSampleTime(5);
  pidP.SetOutputLimits(-255,255);
  pidP.setDeadZone(30);
  pidP.enableDeadZone(true);
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
