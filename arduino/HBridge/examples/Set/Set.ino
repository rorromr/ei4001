#include <HBridge.h>

// Attacch h bridge pwm: 6, pinA: 7, pinB: 8
HBridge hbridge(6, 7, 8);

void setup() {
  Serial.begin(9600);
}

void loop() {
  hbridge.set(100);
  delay(300);

  hbridge.set(-100);
  delay(300);

  hbridge.set(-1000);
  delay(300);

  hbridge.set(1000);
  delay(300);

  hbridge.set(20);
  delay(300);

  hbridge.set(-20);
  delay(300);
}
