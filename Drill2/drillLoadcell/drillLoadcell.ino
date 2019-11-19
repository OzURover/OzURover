#include <ros.h>
#include <std_msgs/Float64.h>
#include "HX711.h"

ros::NodeHandle nh;

const int loadcell_dout = 3;
const int loadcell_sck = 2;
const int calibration_factor = 380;

HX711 scale;

std_msgs::Float64 drillLoadcell_msg;
ros::Publisher drillLoadcell("drillLoadcell", &drillLoadcell_msg);

void setup() {
  scale.begin(loadcell_dout, loadcell_sck);
  scale.set_scale(calibration_factor);
  scale.tare();

  nh.initNode();
  nh.advertise(drillLoadcell);
}

void loop() {
  drillLoadcell_msg.data = scale.get_units();
  drillLoadcell.publish(&drillLoadcell_msg);
  nh.spinOnce();
  delay(1);
}
