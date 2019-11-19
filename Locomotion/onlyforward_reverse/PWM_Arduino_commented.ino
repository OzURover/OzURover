#you need to upload this file to arduino mega and run on ssh with code "rosrun rosserial_python serial_node.py /dev/ttyACM0"

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh; //if you have any work with ros in arduino ide, you need to write this

void messageCb(const std_msgs::Float32 & joyinputy,const std_msgs::Float32 & joyinputx ){   // determines if the rover goes forward or backward
    if(abs(joyinputy.data) < 0.15){
        move(0);
    } else {
        if(joyinputy.data > 0){
            enableforward();
        }
        if(joyinputy.data < 0){
            enablereverse();
        }
        move(255*abs(joyinputy.data));
    }
}

ros::Subscriber<std_msgs::Float32> sub("joyinputy", &messageCb);

void setup(){
  for (int i = 22; i<=53;i++) pinMode(i, OUTPUT);
  for (int i = 2; i<=7;i++) pinMode(i, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);  //creating subscribers
}

void loop(){
   if (false) { 
    nh.spinOnce(); //ROS only processes your callbacks when you tell it to with ros::spinOnce()
   } else {
    enableforward();
    move(255);
   }
   delay(1);
}

void move(int pwm){
  for (int i = 2; i<=7; i++) analogWrite(i, pwm);
}

void enableforward(){
   digitalWrite(22,LOW);
   digitalWrite(25,LOW);
   digitalWrite(27,LOW);
   digitalWrite(52,LOW);
   digitalWrite(51,LOW);
   digitalWrite(49,LOW);
   
   digitalWrite(23,HIGH);
   digitalWrite(24,HIGH);
   digitalWrite(26,HIGH);
   digitalWrite(53,HIGH);
   digitalWrite(50,HIGH);
   digitalWrite(48,HIGH);
}

void enablereverse(){
   digitalWrite(22,HIGH);
   digitalWrite(25,HIGH);
   digitalWrite(27,HIGH);
   digitalWrite(52,HIGH);
   digitalWrite(51,HIGH);
   digitalWrite(49,HIGH);
   
   digitalWrite(23,LOW);
   digitalWrite(24,LOW);
   digitalWrite(26,LOW);
   digitalWrite(53,LOW);
   digitalWrite(50,LOW);
   digitalWrite(48,LOW);
}
