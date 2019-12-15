#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;   
void messageCb(const std_msgs::Float32 & joyinputy, std_msgs::Float32){ 
    if(abs(joyinputy.data) == 0){
        stop();
    }
    if(joyinputy.data != 0) {
        if(joyinputy.data > 0){
            forward();
        }
        if(joyinputy.data < 0){
            back();
        }
    }
}
std_msgs::Float32 test;
ros::Subscriber<std_msgs::Float32> sub("joyinputy", &messageCb);

void setup(){ 
    for(int i=22; i<=27; i++){     //pinMode(x, çıkış);
        pinMode(i, OUTPUT);        // x no'lu pin çıkış olarak ayarlandı
    }
    for(int i=48; i<=53; i++){     
        pinMode(i, OUTPUT);
    }

    stop();  
    nh.initNode();
    nh.subscribe(sub);  
    nh.subscribe(sub1);
    Serial.begin(9600);
}

void stop(){
    for(int i=48; i<=53; i++){
        digitalWrite(i, LOW);
    }

    for(int j=22; j<=27; j++){
        digitalWrite(j, LOW);
    }
}

void back(){                     //High: açık 
   digitalWrite(23,LOW);         //Low: kapalı
   digitalWrite(25,LOW);
   digitalWrite(27,LOW);
   digitalWrite(53,LOW);
   digitalWrite(51,LOW);
   digitalWrite(49,LOW);
   
   digitalWrite(22,HIGH);
   digitalWrite(24,HIGH);
   digitalWrite(26,HIGH);
   digitalWrite(52,HIGH);
   digitalWrite(50,HIGH);
   digitalWrite(48,HIGH);
}

void forward(){
   digitalWrite(23,HIGH);
   digitalWrite(25,HIGH);
   digitalWrite(27,HIGH);
   digitalWrite(53,HIGH);
   digitalWrite(51,HIGH);
   digitalWrite(49,HIGH);
   
   digitalWrite(22,LOW);
   digitalWrite(24,LOW);
   digitalWrite(26,LOW);
   digitalWrite(52,LOW);
   digitalWrite(50,LOW);
   digitalWrite(48,LOW);
}




/*
 * 22-23 left-mid
 * 24-25 left-front
 * 26-27 left-back
 * 52-53 right-front
 * 50-51 right-mid
 * 48-49 right-back
 */