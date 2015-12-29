#include <SimpleEncoder.h>
#include <PID_v1.h>
#include <HBridge.h>
#include <SimpleTimer.h>
// ROS Libs
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <ros.h>

// Constants
# define MOTOR_B 8
# define MOTOR_A 7
# define MOTOR_PWM 6
# define ENCODER_A 2
# define ENCODER_B 4

// PID
double refP,inputP,outputP; //refP recibe referencia de numero de tics
double KpP = 0.11, KiP=0.05, KdP=0.007;

//controladores
PID pidP(&inputP, &outputP, &refP, KpP, KiP, KdP, DIRECT);

// Encoder class
Encoder encoder(ENCODER_A, ENCODER_B);

// H bridge class
HBridge hbridge(MOTOR_PWM, MOTOR_A, MOTOR_B);

// Buttons
uint8_t k1 = 0, k1f = 0;
uint8_t k2 = 0, k2f = 0;

void cmdCb( const std_msgs::Float64& cmd)
{
  refP = cmd.data;
}

// ROS
ros::NodeHandle nh;
std_msgs::Int32 posMsg;
ros::Publisher pub("torso_state", &posMsg);
ros::Subscriber<std_msgs::Float64>subCmd("torso_cmd",&cmdCb);


// Position publisher timer callback
unsigned long last_call = 1;
void posPublisher()
{
  unsigned long call = millis();
  if (call - last_call > 100)
  {
    posMsg.data = encoder.read();
    pub.publish(&posMsg);
    nh.spinOnce();
    last_call = call;
  }
}


void setup(){
  // Set PWM
  hbridge.setPwmFrequency(64); //Pin 6, divido frecuencia base 62,5 Khz por 64
  
  /*
  * ------------------------------
  * PID Config
  * ------------------------------
  */
  pidP.SetSampleTime(5);
  pidP.SetOutputLimits(-30,30);
  pidP.setDeadZone(30);
  pidP.enableDeadZone(true);

  /*
  * ------------------------------
  * Button Config
  * ------------------------------
  */
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  /*
  * ------------------------------
  * ROS Config
  * ------------------------------
  */
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(subCmd);
}

void loop(){
  /*
  * ------------------------------
  * Control
  * ------------------------------
  */
  inputP = encoder.read();
  pidP.Compute();
  hbridge.set((int16_t) outputP);

  posPublisher();

  /*
  * ------------------------------
  * Button Check
  * ------------------------------
  */
  // Falling edge K1
  k1f = k1;
  k1 = digitalRead(9);
  if (k1f == HIGH && k1 == LOW) refP += 500;

  // Falling edge K2
  k2f = k2;
  k2 = digitalRead(10);
  if (k2f == HIGH && k2 == LOW) refP -= 500;
}
