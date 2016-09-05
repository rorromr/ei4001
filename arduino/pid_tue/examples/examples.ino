#include "DPID.h"
#include "SimpleEncoder.h"
#include "HBridge.h"

# define MOTOR_B 8
# define MOTOR_A 7
# define MOTOR_PWM 6
# define ENCODER_A 2
# define ENCODER_B 4

# define ref 5000

double kp = 1.1;
double kv = 0.007;
double ki = 0.15;
double Ts = 0.005;
int discretization_method = 4;
double limit = 255;
double kaw = sqrt(ki*kv);  //backCalculation

DFILTERS::DPID pid(kp,kv,ki,Ts,discretization_method,limit,kaw);

Encoder encoder(ENCODER_A, ENCODER_B);

HBridge hbridge(MOTOR_PWM, MOTOR_A, MOTOR_B);


void setup () {
Serial.begin(9600);

hbridge.setPwmFrequency(64);


  
}


void loop() {
	pid.update(ref - encoder.read());
	hbridge.set( (int16_t) pid.getOutput());


  
}

