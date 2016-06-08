#include <SimpleEncoder.h>
#include <PID_v1.h>
#include <HBridge.h>

# define MOTOR_B 8
# define MOTOR_A 7
# define MOTOR_PWM 6
# define ENCODER_A 2
# define ENCODER_B 4
# define FDC1 10
# define FDC2 11
# define FDC3 12
# define FDC4 13

// PID
double refP,inputP,outputP; //refP recibe referencia de numero de tics
double KpP = 0.11, KiP=0.05, KdP=0.007;

//controladores
PID pidP(&inputP, &outputP, &refP, KpP, KiP, KdP, DIRECT);

//Clase encoder
Encoder encoder(ENCODER_A, ENCODER_B);

// H bridge class
HBridge hbridge(MOTOR_PWM, MOTOR_A, MOTOR_B);


// Buttons
uint8_t k1 = 0, k1f = 0;
uint8_t k2 = 0, k2f = 0;

void setup(){
  Serial.begin(9600);
  
  hbridge.setPwmFrequency(64); //Pin 6, divido frecuencia base 62,5 Khz por 64
  
  pidP.SetSampleTime(5);
  pidP.SetOutputLimits(-255,255);
  pidP.setDeadZone(30);
  pidP.enableDeadZone(true);
  refP = 5000;

  // Buttons
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
}

void loop(){
  inputP = encoder.read();
  pidP.Compute();
  hbridge.set((int16_t) outputP);
  //Serial.print("| Out ");
  //Serial.println(outputP);
  //delay(1);

  // Falling edge K1
  k1f = k1;
  k1 = digitalRead(9);
  if (k1f == HIGH && k1 == LOW) refP += 800;

  // Falling edge K2
  k2f = k2;
  k2 = digitalRead(10);
  if (k2f == HIGH && k2 == LOW) refP -= 800;
}
