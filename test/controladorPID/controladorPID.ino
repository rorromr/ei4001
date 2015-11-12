#include <Encoder.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Float64.h> //Cambiar tipo de datos, al final los mensajes son casi puros int
#include <std_msgs/Int32.h>

//ros::NodeHandle nh;
//std_msgs::Int32 pos_msg;

# define MOTOR_CTL1 7
# define MOTOR_CTL2 8
# define MOTOR_PWM 9
# define encoder_channelA 2
# define encoder_channelB 3

//Sample time ms
int STime = 20; 

//Defino variables
double refP = 10*1440,inputP,outputP,refW,inputW,outputW; //refP recibe referencia de numero de tics
double KpP=1.3, KiP=1, KdP=0.12,KbP = 1, KpW=3, KiW=1, KdW=0.1, KbW = 0;
long actualPos, auxPos=0;

//Limites
float Wmax = 255,Vmax; //limitadores para velocidad y voltaje




//controladores
PID pidP(&inputP, &outputP, &refP, KpP, KiP, KdP, KbP, DIRECT);
PID pidW(&inputW, &outputW, &refW, KpW, KiW, KdW, KbW, DIRECT);

Encoder encoder(encoder_channelA,encoder_channelB);

void REF(const std_msgs::Float64& setpoint){
  refP = setpoint.data;
  
}
  

void controlP(){
  inputP = encoder.read();
  
  if (millis()>20000){
    refP = 3*1440;
  }
  
  //if (abs(refP-inputP)<1000){
  //  pidP.SetTunings(4,0.5,1);
 // }
  
  pidP.Compute();
  if (outputP<0){
    digitalWrite(MOTOR_CTL1,HIGH);
    digitalWrite(MOTOR_CTL2,LOW);
  }
  else{
    digitalWrite(MOTOR_CTL1,LOW);
    digitalWrite(MOTOR_CTL2,HIGH);
  }
  analogWrite(MOTOR_PWM,abs(outputP));

  //controlW(abs(outputP));
  Serial.println("ref=");
  Serial.println(refP);
  
  Serial.println("input=");
  Serial.println(inputP);
  
  Serial.println("output");
  Serial.println(outputP);
}


void controlW(double ref){
  actualPos = encoder.read();
  refW = ref;
  inputW = (actualPos-auxPos)/(STime/10); //diferencia entre posicion actual y anterior, en un periodo de tiempo igual al muestreo
  pidW.Compute();
   //voltaje
 
  auxPos = actualPos;
  
}

//ros::Subscriber<std_msgs::Float64> sub("referencia",&REF);
//ros::Publisher  pub("actual_pos",&pos_msg); 
  
void setup(){
  Serial.begin(9600);
  pinMode ( MOTOR_CTL1 , OUTPUT );
  pinMode ( MOTOR_CTL2 , OUTPUT );
  pinMode ( MOTOR_PWM , OUTPUT );
  
  //nh.initNode();
  //nh.subscribe(sub);
  //nh.advertise(pub);
  
  pidP.SetMode(AUTOMATIC); 
  //pidW.SetMode(AUTOMATIC); 
  pidP.SetSampleTime(STime);
  //pidW.SetSampleTime(STime/10);
  pidP.SetOutputLimits(-Wmax,Wmax);
  //pidW.SetOutputLimits(-Vmax,Vmax);
}

void loop(){
  //pos_msg.data = encoder.read();
  //pub.publish(&pos_msg);
  //nh.spinOnce();
  controlP();
  delay(STime);
}
