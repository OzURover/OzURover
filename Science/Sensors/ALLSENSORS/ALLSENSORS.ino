/*
Temperature Sensor +
Moisture Sensor +
Anemometer -
BMP180 -
UV Sensor +
CO Sensor +
Color Sensor -
ADD COMMENTS
*/

#include <SDISerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <string.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include "Adafruit_VEML6070.h"

#define DECAGON_DATA_PIN 2 //Moisture sensor's data pin
#define ONE_WIRE_BUS 4 // Temperature sensor's data pin

ros::NodeHandle nh;

//ROS publishers
std_msgs::String all_sensors_msg;
ros::Publisher AllSensors("AllSensors", &all_sensors_msg);

//UV sensor variables
Adafruit_VEML6070 uv = Adafruit_VEML6070();
int UVvalue;

//CO sensor variables
const int CO_AOUTpin=A0;
const int CO_DOUTpin=3;  //MAYBE IT CAN BE DELETED
int COvalue;

//Temperature sensor variables
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float celsius = 0;

//Moisture sensor variables
SDISerial connection(DECAGON_DATA_PIN);
char msg[150];
char* resp;


void setup(){
    sensors.begin(); //To start temperature sensor
    connection.begin(); //To start moisture sensor
    uv.begin(VEML6070_1_T); //To start UV sensor
    pinMode(CO_DOUTpin, INPUT); //CO sensor pin (MAYBE IT CAN BE DELETED)
    delay(3000);

    nh.initNode();
    nh.advertise(AllSensors);
}

void loop(){
    //Calculate Moisture
    resp = connection.sdi_query("?M!",1000);
    delay(1000);
    resp = connection.sdi_query("?D0!",1000);//1 second timeout

    //Calculate Temperature
    sensors.requestTemperatures();
    celsius = sensors.getTempCByIndex(0);

    //Calculate CO concantration
    COvalue= analogRead(CO_AOUTpin);

    //Calculate UV level
    UVvalue = uv.readUV();
    
    dtostrf(celsius, 4, 2, msg); //Converts float value of celsius to string value
    sprintf(msg, "%s", msg); 

    //Publish temperature data
    all_sensors_msg.data = msg;
    AllSensors.publish( &all_sensors_msg );

    sprintf(msg, "%s", resp); 

    //Publish moisture data
    all_sensors_msg.data = msg;
    AllSensors.publish( &all_sensors_msg );

    sprintf(msg, "%d", COvalue); 

    //Publish CO data
    all_sensors_msg.data = msg;
    AllSensors.publish( &all_sensors_msg );

    sprintf(msg, "%d", UVvalue); 

    //Publish UV data
    all_sensors_msg.data = msg;
    AllSensors.publish( &all_sensors_msg );
    
    nh.spinOnce();
    delay(10);
  
}
