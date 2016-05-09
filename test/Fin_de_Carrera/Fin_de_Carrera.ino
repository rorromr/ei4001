#include <Fin_de_Carrera.h>

Fin_de_Carrera fdc(4,5,6,7); //Siempre colocar el comun a tierra

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (fdc.getState()){
    switch (fdc.getIndex()){
    case 1:
      Serial.println("Fin de carrera 1");
      break;
    case 2:
      Serial.println("Fin de carrera 2");
      break;
    case 3:
      Serial.println("Fin de carrera 3");
      break;
    case 4:
      Serial.println("Fin de carrera 4");
      break;
      }  
  }
  delay(100);

}
