#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;   //if you have any work with ros in arduino ide, you need to write this

void messageCb(const std_msgs::Float32 & joyinputj0, std_msgs::Float32){   // determines if the rover goes forward or backward
    if(abs(joyinputj0.data) < 0.005){
        halt();
    }
    if(joyinputj0.data != 0) {
        if(joyinputj0.data > 0.005){
            enableCCW();
        }
        if(joyinputj0.data < 0){
            enableCW();
        }
        move(255*abs(joyinputj0.data));
    }
}


ros::Subscriber<std_msgs::Float32> joyinputj0("joyinputj0", &messageCb);


void setup(){ 
  	pinMode(6, OUTPUT); 
  	pinMode(7, OUTPUT);
  	pinMode(5, OUTPUT);

    halt();  //it is for stopping rover
    nh.initNode();
    nh.subscribe(joyinputj0);
    nh.getHardware()->setBaud(76800);
}

void loop(){
    nh.spinOnce();   //ROS only processes your callbacks when you tell it to with ros::spinOnce()
    delay(1);
}

void move(int pwm){
	analogWrite(5,pwm);
}

void halt(){
	digitalWrite(6, LOW);
	digitalWrite(7, LOW);	
	analogWrite(5, LOW);
}

void enableCCW() {
	digitalWrite(6, HIGH);
	digitalWrite(7, LOW);
}


void enableCW() {
	digitalWrite(6, LOW);
	digitalWrite(7, HIGH);
}
