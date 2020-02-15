#include <OneWire.h>
#include <DallasTemperature.h> 

#define ONE_WIRE_BUS 4

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float celsius = 0;

void setup(){
      Serial.begin(9600);
      sensors.begin();
      delay(3000);
}

void loop(){
    sensors.requestTemperatures();
    celsius = sensors.getTempCByIndex(0);

    
    Serial.println(celsius);

    delay(1000);
  
}
