# define fdc_interrupt 2 //Detecta interrupción

# define fdc_1 5 //En orden desde lo mas alto a lo mas bajo
# define fdc_2 6
# define fdc_3 7
# define fdc_4 8

volatile boolean flag = LOW;

  void callback(){
    flag = !flag;
  }

void setup() {
  Serial.begin(115200);
  pinMode(fdc_interrupt,INPUT_PULLUP);
  pinMode(fdc_1,INPUT);
  pinMode(fdc_2,INPUT);
  pinMode(fdc_3,INPUT);
  pinMode(fdc_4,INPUT);
  attachInterrupt(digitalPinToInterrupt(fdc_interrupt), callback, FALLING);
}

void loop() {
  if (flag){
    flag = !flag;
    if (!digitalRead(fdc_1)){
      Serial.println("Fin de carrera 1");
      }
    else if (!digitalRead(fdc_2)){
      Serial.println("Fin de carrera 2");
      }
    else if (!digitalRead(fdc_3)){
      Serial.println("Fin de carrera 3");
      }
    else if (!digitalRead(fdc_4)){
      Serial.println("Fin de carrera 4");
      }
    }
  delay(10);
}
