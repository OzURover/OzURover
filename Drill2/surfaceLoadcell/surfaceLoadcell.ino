#include <ros.h>
#include <std_msgs/Float64.h>
#include "HX711.h"

ros::NodeHandle nh;

const int loadcell_dout = 3;
const int loadcell_sck = 2;
const int calibration_factor = 380;

HX711 scale;

std_msgs::Float64 surfaceLoadcell_msg;
ros::Publisher surfaceLoadcell("surfaceLoadcell", &surfaceLoadcell_msg);

void setup() {
  scale.begin(loadcell_dout, loadcell_sck);
  scale.set_scale(calibration_factor);
  scale.tare();

  nh.initNode();
  nh.advertise(surfaceLoadcell);
}

void loop() {
  surfaceLoadcell_msg.data = scale.get_units();
  surfaceLoadcell.publish(&surfaceLoadcell_msg);
  nh.spinOnce();
  delay(1);
}
