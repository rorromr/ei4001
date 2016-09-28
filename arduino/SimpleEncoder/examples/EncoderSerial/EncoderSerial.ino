#include <SimpleEncoder.h>

// Attacch encoder A: 2, B: 3
Encoder encoder(2, 4);

void setup() {
  Serial.begin(9600);
}

int32_t pos;

void loop() {
  pos = encoder.read();
  Serial.println(pos);
  delay(100);
}
