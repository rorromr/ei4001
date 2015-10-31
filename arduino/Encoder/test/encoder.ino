//PIN's definition
#define encoder0PinA  2
#define encoder0PinB  3

# define MOTOR_CTL1 8
# define MOTOR_CTL2 10
# define MOTOR_PWM 9


volatile int encoder0Pos = 0;
volatile boolean PastA = 0;
volatile boolean PastB = 0;

void setup() 
{
  Serial.begin(9600);
  pinMode(encoder0PinA, INPUT);
  //turn on pullup resistor
  //digitalWrite(encoder0PinA, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  pinMode(encoder0PinB, INPUT); 
  //turn on pullup resistor
  //digitalWrite(encoder0PinB, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  PastA = (boolean)digitalRead(encoder0PinA); //initial value of channel A;
  PastB = (boolean)digitalRead(encoder0PinB); //and channel B

//To speed up even more, you may define manually the ISRs
// encoder A channel on interrupt 0 (arduino's pin 2)
  attachInterrupt(0, doEncoderA, RISING);
// encoder B channel pin on interrupt 1 (arduino's pin 3)
  attachInterrupt(1, doEncoderB, CHANGE); 

  pinMode ( MOTOR_CTL1 , OUTPUT ) ;
  pinMode ( MOTOR_CTL2 , OUTPUT ) ;

  pinMode ( MOTOR_PWM , OUTPUT ) ;

}


void loop()
{  
 Serial.println(encoder0Pos);
 digitalWrite ( MOTOR_CTL1 , HIGH ) ;
 digitalWrite ( MOTOR_CTL2 , LOW ) ;
// PWM para controlar velocidad
 analogWrite ( MOTOR_PWM , 200) ;
 delay (2000) ;
// Cambiar sentido de giro
 digitalWrite ( MOTOR_CTL1 , LOW ) ;
 digitalWrite ( MOTOR_CTL2 , HIGH ) ;
 delay (1000) ;
 analogWrite ( MOTOR_PWM , 255) ; // Velocidad Max
 delay (100) ;
}

//you may easily modify the code  get quadrature..
//..but be sure this whouldn't let Arduino back! 

void doEncoderA()
{
     PastB ? encoder0Pos--:  encoder0Pos++;
}

void doEncoderB()
{
     PastB = !PastB; 
}


