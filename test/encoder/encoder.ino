#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder encoder(2, 3);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
}

void loop() {
  long Pos;
  Pos = encoder.read();
  Serial.println(Pos);
  delay(100);
}
