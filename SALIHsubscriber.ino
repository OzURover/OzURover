#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;   

void messageCb(const std_msgs::Float32 & joyinputx, std_msgs::Float32){   
if (joyinputx.data==1){
    
    enableforward() ;
}
else if(joyinputx.data==-1){
    
    enablereverse();
}    

}

ros::Subscriber<std_msgs::Float32> sub("joyinputx", &messageCb);


void setup(){ 
    
    pinMode(22, OUTPUT);
    

    halt();  //it is for stopping motor
    nh.initNode();
    nh.subscribe(sub);  //creating subscribers
    nh.subscribe(sub1);
    Serial.begin(9600);
}
void loop(){
    nh.spinOnce();   
    delay(1);
}

void enablereverse(){
   digitalWrite(22,LOW);
   
   
}

void enableforward(){
   digitalWrite(22,HIGH);
   
}


