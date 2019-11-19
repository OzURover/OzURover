/*
Tested on UNO R3.
Sketch was built with Arduino 1.6.12
Dependency: Joran Beasley SDISerial Library
Accessed May 26, 2017 here:
https://github.com/joranbeasley/SDISerial

download and rename folder "SDISerial" and place in  "<ARDUINO_ROOT>/libraries"

Tested with the with the 5TM Soil Moisture and Temp Sensor:
http://www.decagon.com/en/soils/volumetric-water-content-sensors/5tm-vwc-temp/

Hook-up:
the WHITE wire goed to 5V. however you could also connect it to a pin and drive power only when you wanted it
the RED wire is the DATA_PIN. - you must hook it up to a pin that can process interrupts (see link below)  
the remaining "shield" wire must be connected to ground
*/

#include <SDISerial.h>
#include <string.h>
#define DATA_PIN 2
SDISerial connection(DATA_PIN);
char output_buffer[125]; // just for uart prints
char tmp_buffer[4];
char* resp; // Pointer to response char

//initialize variables
void setup(){
      connection.begin();
      Serial.begin(9600);//so we can print to standard uart
      //small delay to let the sensor do its startup stuff
      delay(3000);//3 seconds should be more than enough
      char* sensor_info = connection.sdi_query("?I!",1000); // get sensor info
      //print to uart
      sprintf(output_buffer,"Sensor Info: %s",sensor_info?sensor_info:"No Response");
      Serial.println(output_buffer);
}

//main loop
void loop(){

   //print to uart
    Serial.println("Begin Command: ?M!");   
    //send measurement query (M) to the first device on our bus
    resp = connection.sdi_query("?M!",1000);//1 second timeout
    //this really just returns a message that tells you the maximum wait before the measurement is ready
    
    sprintf(output_buffer,"RECV: %s",resp?resp:"No Response Recieved!!");
    Serial.println(output_buffer);
    delay(1000);//sleep for 1 seconds before the next command
    
    //print to uart
    Serial.println("Begin Command: ?D0!");
    resp = connection.sdi_query("?D0!",1000);//1 second timeout
    
    sprintf(output_buffer,"RECV: %s",resp?resp:"No Response Recieved!!");
    Serial.println(output_buffer);
    delay(3000);//sleep for 10 seconds before the next read
  
}
