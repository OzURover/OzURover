#include <SDISerial.h>

#define DATA_PIN 2

SDISerial connection(DATA_PIN);
char whole_message[125];
char* resp;

void setup(){
      Serial.begin(9600);
      connection.begin();
      delay(3000);

}

void loop(){
    resp = connection.sdi_query("?M!",1000);
    delay(1000);
    resp = connection.sdi_query("?D0!",1000);//1 second timeout
    
    sprintf(whole_message, "%s", resp);
    Serial.println(whole_message);

    delay(1000);
  
}
