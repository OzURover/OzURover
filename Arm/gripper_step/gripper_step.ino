#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

const int step_pin = 2;
const int dir_pin = 3;

const int trigPin = A0;
const int echoPin = A1;

ros::NodeHandle nh;

void messageCb(const std_msgs::Int32& data){
  if(data.data < 0) {
    digitalWrite(dir_pin, LOW);
    for(int i = 0; i < abs(data.data); i++) {
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(1500);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(1500);
    }
  }else if(data.data > 0) {
    digitalWrite(dir_pin, HIGH);
    for(int i = 0; i < abs(data.data); i++) {
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(1500);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(1500);
    }
  }
}

ros::Subscriber<std_msgs::Int32>sub("step_claw", &messageCb);

std_msgs::Float32 distanceMsg;
ros::Publisher distance("arm_distance", &distanceMsg);

void setup() {
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(distance);
}

void loop() {
  long duration, cm;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  
  distanceMsg.data = duration/29.0,2.0;
  distance.publish(&distanceMsg);
  
  nh.spinOnce();
  delay(100);
}
