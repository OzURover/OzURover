#include "ros.h";

#include <SoftwareSerial.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/header.h>
#include <ros/time.h>

#include <std_msgs/Bool.h>

int encoderResetPin = 4;
const int reductionRatio = 86, sampleTime = 100; // in ms;
const float Pi = 3.14159, wheelDiameter = 0.154; //diameter in meters
const float prodCoefficient = wheelDiameter * Pi / (reductionRatio * 500);
long EncoderTicks, prevEncoderTicks;
float prevDistance = 0, prevVelocity = 0;
float disVelAcc[3];
unsigned long timePassed;

ros::NodeHandle nh;
geometry_msgs::PointStamped rosArray;
ros::Publisher distVelArray("LB_info", &rosArray);
SoftwareSerial PIC_UART(2, 3);

void setup() {
  nh.getHardware()->setBaud(9600);
  nh.initNode();

  nh.advertise(distVelArray);

  pinMode(encoderResetPin, OUTPUT);

  digitalWrite(encoderResetPin, HIGH);
  PIC_UART.begin(9600);
}

void loop() {
  PIC_UART.write('s');
  if (PIC_UART.available()) {
    EncoderTicks = getEncoderTicks();

    timePassed = (millis() - timePassed);

    if (timePassed > sampleTime) {

      disVelAcc[0] = calculateDistance(EncoderTicks);
      disVelAcc[1] = calculateVelocity(prevDistance, disVelAcc[0], timePassed);
      disVelAcc[2] = calculateAcceleration(prevVelocity, disVelAcc[1], timePassed);
    }

    prevEncoderTicks = EncoderTicks;
    prevDistance = disVelAcc[0];
    prevVelocity = disVelAcc[1];
  }

  rosArray.point.x = disVelAcc[0];
  rosArray.point.y = disVelAcc[1];
  rosArray.point.z = disVelAcc[2];

  rosArray.header.stamp = nh.now();

  distVelArray.publish(&rosArray);

  nh.spinOnce();
}

long getEncoderTicks() {
  String strVal = "";
  char tmp;
  while (PIC_UART.available()) {
    tmp = PIC_UART.read();
    strVal += tmp;
  }
  return strVal.toInt();
}

float calculateDistance(long EncoderTicks){
  return EncoderTicks * prodCoefficient;
}

float calculateVelocity(float initialPosition, float finalPosition, unsigned long timePassed){
  return 1000000 * (finalPosition - initialPosition) / timePassed;
}

float calculateAcceleration(float initialVelocity, float finalVelocity, unsigned long timePassed){
  return 1000000 * (finalVelocity - initialVelocity) / timePassed;
}
