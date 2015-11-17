#include <SimpleEncoder.h>

// Attacch encoder A: 2, B: 3
Encoder encoder(2, 3);

void setup() {
  Serial.begin(9600);
}

uint32_t pos;

void loop() {
  pos = encoder.read();
  Serial.println(pos);
  delay(100);
}
