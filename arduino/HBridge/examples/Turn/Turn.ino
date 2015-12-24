#include <HBridge.h>

// Attacch h bridge pwm: 6, pinA: 7, pinB: 8
HBridge hbridge(6, 7, 8);

void setup() {
	Serial.begin(9600);
}

void loop() {
  hbridge.forward();
  hbridge.setPwm(200);
  delay(800);
  hbridge.backward();
  hbridge.setPwm(100);
  delay(800);
}
