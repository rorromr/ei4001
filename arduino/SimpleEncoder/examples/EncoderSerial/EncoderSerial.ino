#include <SimpleEncoder.h>

// Attacch encoder A: 2, B: 4
# define ENCODER_A 2
# define ENCODER_B 4
Encoder encoder(ENCODER_A, ENCODER_B);

void setup() {
  Serial.begin(9600);
}

int32_t pos;

void loop() {
  pos = encoder.read();
  Serial.println(pos);
  delay(100);
}