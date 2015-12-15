#include <SimpleEncoder.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Float64.h> //Cambiar tipo de datos, al final los mensajes son casi puros int
#include <std_msgs/Int32.h>
#include <bender_torso/TorsoParams.h>
#include <EEPROM.h>
#include <SimpleTimer.h>


ros::NodeHandle nh;
std_msgs::Int32 pos_msg;

# define MOTOR_CTL1 7
# define MOTOR_CTL2 8
# define MOTOR_PWM 6
# define encoder_channelA 2
# define encoder_channelB 4
# define fdc_interrupt 3  //Solo por ahora, luego se debe usar el pin 3

# define fdc_1 9 //En orden desde lo mas alto a lo mas bajo
# define fdc_2 10
# define fdc_3 11
# define fdc_4 12



//Sample time ms
int STime = 5; 

//Defino variables
double refP,inputP,outputP; //refP recibe referencia de numero de tics
double KpP, KiP, KdP;
long actualPos, auxPos=0;

//Limites
float Wmax = 255,Vmax; //limitadores para velocidad y voltaje
int pose_max,pose_min = 0; //limites en que puede andar bendercillo, seteada en tics. 1 vuelta son 360 tics

//controladores
PID pidP(&inputP, &outputP, &refP, KpP, KiP, KdP, DIRECT);

//Clase encoder
Encoder encoder(encoder_channelA,encoder_channelB);





//Clase Timer
SimpleTimer timer_savePosition;
SimpleTimer timer_publicador;






//Funciones que reciben de ROS
void REF(const std_msgs::Float64& setpoint){
  pidP.SetMode(AUTOMATIC);
  refP = setpoint.data;
  if (refP > pose_max){
    refP = pose_max;
  }
  else if (refP < pose_min){
    refP = pose_min;
  }
}





void PAR(const bender_torso::TorsoParams& constantes){
  KpP = constantes.kp;
  KiP = constantes.ki;
  KdP = constantes.kd;
}






//Set la frecuencia a la que se entrega el PWM  
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





  
//Controlador PID
void controlP(){
  inputP = encoder.read();
  pidP.Check();
  if (outputP<0){
    digitalWrite(MOTOR_CTL1,HIGH);
    digitalWrite(MOTOR_CTL2,LOW);
  }
  else{
    digitalWrite(MOTOR_CTL1,LOW);
    digitalWrite(MOTOR_CTL2,HIGH);
  }
  analogWrite(MOTOR_PWM,abs(outputP)); //PWM
}





//Calibrar, fines de carrera
void calibrate(){
  if (!digitalRead(fdc_1)){ //Leo el pin que fue apretado, todos estan inicialmente en high pues conecto el NO a arduino
    //Cortar todo, rele superior
  }
  else if (!digitalRead(fdc_2)){
    refP = encoder.read()+ 2*360; //Paso de 4 mm, 2 vueltas, 8 mm mas
    while (outputP !=0){
      controlP();
    }
    encoder.write(pose_max);
  }
  else if (!digitalRead(fdc_3)){
    refP = encoder.read()-2*360;
    while (outputP !=0){
      controlP();
    }
    encoder.write(pose_min);
  }
  else{
    //cortar todo, rele inferior
  }
}
  



//Guardo posicion en EEPROM
void savePosition(){
  long dato = encoder.read();
  byte B;
  for (int i=0; i<sizeof(dato);i++){
    B = highByte(dato);
    EEPROM.write(i,B);
    dato = dato << 8;
  }
}





//Leo posicion desde EEPROM
long readPosition(){
  long Pos = 0;
  long D;
  int length = sizeof(Pos);
  for (int i=0; i<length; i++){
    D = EEPROM.read(i);
    if (i != 3){
      D = D << (8*(length-i-1));
    }
    
    Pos = Pos | D; 
    D = 0;
  }
}





//Publisher y Subscriber
ros::Subscriber<std_msgs::Float64> sub_ref("cmd",&REF);
ros::Subscriber<bender_torso::TorsoParams> sub_parameters("parameters",&PAR);
ros::Publisher  pub("actual_pos",&pos_msg); 

//Publico a ROS posicion actual
void publicador(){
  pos_msg.data = encoder.read();
  pub.publish(&pos_msg);
  nh.spinOnce();
}



  
void setup(){
  Serial.begin(9600);
  pinMode ( MOTOR_CTL1 , OUTPUT );
  pinMode ( MOTOR_CTL2 , OUTPUT );
  pinMode ( MOTOR_PWM , OUTPUT );
  pinMode ( fdc_interrupt, INPUT );
  pinMode (fdc_1,INPUT);
  pinMode (fdc_2,INPUT);
  pinMode (fdc_3,INPUT);
  pinMode (fdc_4,INPUT);
  
  attachInterrupt(0,calibrate,RISING);//Interrupcion cuando se aprieta fin de carrera
  
  setPwmFrequency(6,64); //Pin 6, divido frecuencia base 62,5 Khz por 64
  
  nh.initNode();
  nh.subscribe(sub_ref);
  nh.subscribe(sub_parameters);
  nh.advertise(pub);
  
  pidP.SetSampleTime(STime);
  pidP.SetOutputLimits(-Wmax,Wmax);
  
  //Timers para llamar funciones
  //timer_savePosition.setInterval(1000,savePosition);
  timer_publicador.setInterval(5,publicador);
  
  //Al prender, entrego a encoder la posicion actual
  //encoder.write(readPosition()); aca asigno al encoder la pos que tiene la EEPROM. Sino tendra que recibirlo de ros
}

void loop(){
  //timer_savePosition.run();
  timer_publicador.run();
  nh.spinOnce();
  controlP(); //No sera mejor hacerle un timer o llamarla cuando yo quiera? duda
  
  delay(STime);
}
