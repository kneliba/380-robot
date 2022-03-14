#include <Wire.h>

void setup() {
  // put your setup code here, to run once:
  Wire.begin(D1, D2);  
  Wire.onRequest(requestEvent); // register event
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}

void requestEvent() {
  Wire.write("Hello Uno");
