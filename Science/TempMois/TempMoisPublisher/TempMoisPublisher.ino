#include <SDISerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <string.h>
#include <ros.h>
#include <std_msgs/String.h>

#define DATA_PIN 2 //Decagon's data pin
#define ONE_WIRE_BUS 8 // Temperature sensor's data pin

ros::NodeHandle nh;

//ROS publishers
std_msgs::String temp_mois_msg;
ros::Publisher TempMois("TempMois", &temp_mois_msg);

//Temp. sensor variables
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float celsius = 0;

//Decagon variables
SDISerial connection(DATA_PIN);
char whole_message[125];
char* resp;


void setup(){
      sensors.begin();
      connection.begin();
      delay(3000);

      nh.initNode();
      nh.advertise(TempMois);
}

void loop(){
    //Calculate Decagon datas
    resp = connection.sdi_query("?M!",1000);
    delay(1000);
    resp = connection.sdi_query("?D0!",1000);//1 second timeout

    //Calculate temperature
    sensors.requestTemperatures();
    celsius = sensors.getTempCByIndex(0);

    
    dtostrf(celsius, 4, 2, whole_message); //Converts float value of celsius to string value
    sprintf(whole_message,"%s, %s", whole_message, resp); //Combine temperature and moisture datas in a string
    

    //Publish datas
    temp_mois_msg.data = whole_message;
    TempMois.publish( &temp_mois_msg );

    nh.spinOnce();
    delay(1000);
  
}
