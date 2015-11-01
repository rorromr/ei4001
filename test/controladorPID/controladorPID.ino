#include <Encoder.h>
#include <PID_v1.h>

# define MOTOR_CTL1 8
# define MOTOR_CTL2 10
# define MOTOR_PWM 9
# define encoder_channelA 2
# define encoder_channelB 3

//Defino variables
double refP,inputP,outputP,refW,inputW,outputW; //refP recibe referencia de numero de tics
double KpP=2, KiP=5, KdP=1, KpW=2, KiW=5, KdW=4;
long actualPos, auxPos=0;

//Limites
float Wmax,Vmax; //limitadores para velocidad y voltaje

//Sample time ms
int STime = 5; 


//controladores
PID pidP(&inputP, &outputP, &refP, KpP, KiP, KdP, DIRECT);
PID pidW(&inputW, &outputW, &refW, KpW, KiW, KdW, DIRECT);

Encoder encoder(encoder_channelA,encoder_channelB);



void controlP(){
  inputP = encoder.read();
  pidP.Compute();
  if (outputP>Wmax){
    outputP = Wmax;
  }
  else if (outputP<-Wmax){
    outputP = -Wmax;
  }
  controlW(outputP); //Revisar esto de las referencia, si estan bien puestas
}


void controlW(double outputP){
  actualPos = encoder.read();
  refW = outputP;
  inputW = (actualPos-auxPos)/(STime/10); //diferencia entre posicion actual y anterior, en un periodo de tiempo igual al muestreo
  pidW.Compute();
  
  outputW = outputW/Vmax; //voltaje
  if (outputW>0){
    digitalWrite(MOTOR_CTL1,HIGH);
    digitalWrite(MOTOR_CTL2,LOW);
  }
  else{
    digitalWrite(MOTOR_CTL1,LOW);
    digitalWrite(MOTOR_CTL2,HIGH);
  }
  analogWrite(MOTOR_PWM,(int) abs(outputW*255));
  auxPos = actualPos;
  
}
    
  
void setup(){
  Serial.begin(9600);
  pinMode ( MOTOR_CTL1 , OUTPUT );
  pinMode ( MOTOR_CTL2 , OUTPUT );
  pinMode ( MOTOR_PWM , OUTPUT );
  pidP.SetMode(AUTOMATIC); 
  pidW.SetMode(AUTOMATIC); 
  pidP.SetSampleTime(STime);
  pidW.SetSampleTime(STime/10);
}

void loop(){
  controlP();
  delay(30);
}
