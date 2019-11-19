/* 
  IMPORTANT! 
  This code uses an external library DIO2, it needs to be downloaded from: 
  http://www.codeproject.com/Articles/732646/Fast-digital-I-O-for-Arduino
  If the library is not installed properly, the code will not compile and/or 
  the readings coming from the encoder will not be correct.
*/

#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/Float32.h>


#define encoderReset 4

//motor info
#define reductionRatio 86

//diameter in meters
#define wheelDiameter 0.154

const float Pi = 3.14159;
const float mCoefficient = wheelDiameter*Pi/(reductionRatio*500);

SoftwareSerial PIC_UART(3,2);

ros::NodeHandle nh;

std_msgs::Float32 dist_msg;

ros::Publisher curr_pos1("curr_pos1", &dist_msg);



void setup() {
     delay(200);
     pinMode(encoderReset,OUTPUT);
     digitalWrite(encoderReset,HIGH);
     nh.getHardware()->setBaud(9600);
     nh.initNode();
     nh.advertise(curr_pos1);
     PIC_UART.begin(9600);
  }
String strVal = "";
float floatVal = 0;
void loop() {


        while(PIC_UART.available() > 0) {
         char tmp = PIC_UART.read();
         if(tmp != '\n'){
         strVal += tmp;
         }
         else {   
          floatVal = (strVal.toInt()*mCoefficient);
          strVal = "";
          dist_msg.data = floatVal;
          curr_pos1.publish(&dist_msg);
          nh.spinOnce();
          }
        }
        
  }


void resetEncoder(){
  digitalWrite(encoderReset,LOW);
  delay(10);
  digitalWrite(encoderReset,HIGH);
}

