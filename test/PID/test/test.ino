#include <Encoder.h>
#include <PID_v1.h>


# define MOTOR_CTL1 8
# define MOTOR_CTL2 10
# define MOTOR_PWM 9
# define encoder_channelA 2
# define encoder_channelB 3

double ref, Input, Output;
double Kp=2, Ki=5, Kd=1;
PID pid(&Input, &Output, &ref, Kp, Ki, Kd, DIRECT);

Encoder encoder(encoder_channelA,encoder_channelB);

void setup(){
  Serial.begin(9600);
  pinMode ( MOTOR_CTL1 , OUTPUT );
  pinMode ( MOTOR_CTL2 , OUTPUT );
  pinMode ( MOTOR_PWM , OUTPUT );

  pid.SetMode(AUTOMATIC);
  
}

