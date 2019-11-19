#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <avr/wdt.h>

ros::NodeHandle nh;   //if you have any work with ros in arduino ide, you need to write this

void messageCb(const std_msgs::Float32 & joyinputj2, std_msgs::Float32){   // determines if the rover goes forward or backward
    if(abs(joyinputj2.data) < 0.005){
        halt();
    }
    if(joyinputj2.data != 0) {
        if(joyinputj2.data > 0.005){
            enableCCW();
        }
        if(joyinputj2.data < 0){
            enableCW();
        }
        move(127*abs(joyinputj2.data));
    }
}

ros::Subscriber<std_msgs::Float32> joyinputj2("joyinputj2", &messageCb);

void setup(){ 
  	pinMode(6, OUTPUT); 
  	pinMode(7, OUTPUT);
  	pinMode(5, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
   
    halt();  //it is for stopping rover
    nh.initNode();
    nh.subscribe(joyinputj2);
	
}

void loop(){
    nh.spinOnce();   //ROS only processes your callbacks when you tell it to with ros::spinOnce()
    delay(1);
}

void move(int pwm){
	  analogWrite(5,pwm);
}

void halt(){
  digitalWrite(9, HIGH);
  digitalWrite(10, LOW);
	digitalWrite(6, LOW);
	digitalWrite(7, LOW);	
	analogWrite(5, 0);
}

void enableCCW() {
    digitalWrite(10, HIGH);
    digitalWrite(9, LOW);
	  digitalWrite(6, HIGH);
	  digitalWrite(7, LOW);
}


void enableCW() {
    digitalWrite(10, HIGH);
    digitalWrite(9, LOW);
	  digitalWrite(6, LOW);
	  digitalWrite(7, HIGH);
}
