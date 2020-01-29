/*
D -> Drill System
C -> Chemicals
S -> Sensors
L -> Locomotion

Button 1 -> D_ActuatorUp
Button 2 -> D_ActuatorDown
Button 3 -> C_Step
Button 4 -> C_Water
Button 5 -> S_ActuatorUp
Button 6 -> S_ActuatorDown

Switch 1 -> D_OnOff
Switch 2 -> C_Shake
Switch 3 -> L_DriveMode
*/
// ADD LOCOMOTION OPTIONS

#include <ros.h>
#include <std_msgs/String.h>
#include <LiquidCrystal.h>

#define D_OnOff 53
#define D_ActuatorUp 2
#define D_ActuatorDown 3

#define C_Shake 51
#define C_Step 4
#define C_Water 5

#define S_ActuatorUp 6
#define S_ActuatorDown 7

#define L_DriveMode 49

ros::NodeHandle nh;

std_msgs::String science_msg;
std_msgs::String locomotion_msg;

ros::Publisher ScienceController("ScienceController", &science_msg);
ros::Publisher LocomotionController("LocomotionController", &locomotion_msg);

ros::Subscriber<std_msgs::String> sub("CS_LCD", &messageCb );

char whole_message[125];
char whole_message2[125];

LiquidCrystal lcd(33, 35, 37, 39, 41, 43);


void messageCb( const std_msgs::String& msg){
  lcd.setCursor(0, 0);
  lcd.print(msg);
}

void setup()
{
  lcd.begin(16, 2);

  nh.initNode();
  nh.advertise(ScienceController);
  nh.advertise(LocomotionController);
  nh.subscribe(sub);
}

void loop()
{
  String D_OnOffValue = String(digitalRead(D_OnOff));
  String D_ActuatorUpValue = String(digitalRead(D_ActuatorUp));
  String D_ActuatorDownValue = String(digitalRead(D_ActuatorDown));

  String C_ShakeValue = String(digitalRead(C_Shake));
  String C_StepValue = String(digitalRead(C_Step));
  String C_WaterValue = String(digitalRead(C_Water));

  String S_ActuatorUpValue = String(digitalRead(S_ActuatorUp));
  String S_ActuatorDownValue = String(digitalRead(S_ActuatorDown));

  sprintf(whole_message, "%s,%s,%s,%s,%s,%s,%s,%s",  S_ActuatorUpValue, S_ActuatorDownValue, C_ShakeValue, C_StepValue, C_WaterValue, D_OnOffValue, D_ActuatorUpValue, D_ActuatorDownValue);

  science_msg.data = whole_message;
  ScienceController.publish( &science_msg );

  locomotion_msg.data = whole_message2;
  LocomotionController.publish( &locomotion_msg );

  nh.spinOnce();
  delay(1);
}