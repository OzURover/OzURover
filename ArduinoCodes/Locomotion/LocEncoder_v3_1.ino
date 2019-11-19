#include "ros.h";
#include <SoftwareSerial.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>

const int reductionRatio = 86 , sampleTime = 100; // in ms;
const float Pi = 3.14159, wheelDiameter = 0.154; //diameter in meters
const float prodCoefficient = wheelDiameter*Pi/(reductionRatio*500);

int encoderResetPin = 4;
long EncoderTicks ,prevEncoderTicks;
float instDistance = 0,prevDistance = 0,instVelocity = 0,prevVelocity = 0,instAcceleration = 0;

unsigned long timePassed;

ros::NodeHandle nh;

std_msgs::Float32MultiArray encoder_msg;

ros::Publisher encoder_data("EncoderRightMiddle", &encoder_msg); //Change topic name for each wheel

void messageCb(const std_msgs::Bool &toggle_msg){
  if(toggle_msg.data == 1) {
    digitalWrite(encoderResetPin,LOW);
    delay(10);
  }
  digitalWrite(encoderResetPin,HIGH);
}

ros::Subscriber<std_msgs::Bool> resetEncoder("resetEncoder", &messageCb );

SoftwareSerial PIC_UART(2,3);

void setup() {
     
     nh.getHardware()->setBaud(19200);
     nh.initNode();

     nh.advertise(encoder_data);
     
     nh.subscribe(resetEncoder);
     
     pinMode( encoderResetPin,OUTPUT);
     
     digitalWrite(encoderResetPin,HIGH);
     PIC_UART.begin(9600);

}


void loop() {
      PIC_UART.write('s');
      if(PIC_UART.available()){
        
        EncoderTicks = getEncoderTicks();
       
        timePassed = (millis() - timePassed);
         
          if( timePassed > sampleTime){
          
            instDistance = calculateDistance(EncoderTicks);
            instVelocity = calculateVelocity(prevDistance , instDistance,timePassed);
            instAcceleration = calculateAcceleration(prevVelocity,instVelocity,timePassed);
            
          }
        
        prevEncoderTicks = EncoderTicks;        
        prevDistance = instDistance;
        prevVelocity = instVelocity;
      }

      encoder_msg.data[0] = instDistance;
      encoder_msg.data[1] = instVelocity;
      encoder_msg.data[2] = instAcceleration;

      encoder_data.publish(&encoder_msg);
      
      nh.spinOnce();
  }

long getEncoderTicks(){
  String strVal = "";
  char tmp;
  while(PIC_UART.available()){
  tmp = PIC_UART.read();  
  strVal += tmp;   
  }
  return strVal.toInt();
 
}

float calculateDistance (long EncoderTicks ){
  return EncoderTicks * prodCoefficient;
}

float calculateVelocity(float initialPosition, float finalPosition, unsigned long timePassed){
  return 1000*(finalPosition - initialPosition) / timePassed ;
}

float calculateAcceleration( float initialVelocity, float finalVelocity, unsigned long timePassed){
  return 1000*(finalVelocity - initialVelocity)/timePassed ;
}
