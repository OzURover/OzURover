#include <Wire.h>
#include "Adafruit_VEML6070.h"

Adafruit_VEML6070 uv = Adafruit_VEML6070();
int uvv;
void setup() {
  Serial.begin(9600);
  Serial.println("VEML6070 Test");
  uv.begin(VEML6070_1_T);  // pass in the integration time constant
}


void loop() {
  Serial.print("UV light level: ");
  uvv = uv.readUV();
  Serial.println(uvv);
  
  delay(1000);
}
