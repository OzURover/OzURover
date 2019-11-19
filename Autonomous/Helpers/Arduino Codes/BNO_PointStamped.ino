#include <Adafruit_BNO055.h>

#include <ros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/header.h>
#include <ros/time.h>

#include <SoftwareSerial.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (1)

ros::NodeHandle nh;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

ros::Time rosTime;
geometry_msgs::PointStamped gyroArray;
ros::Publisher pub("bno_gyro", &gyroArray);

void setup(){
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(pub);
  
  bno.begin();
  bno.setExtCrystalUse(true);
}

void loop(){
  sensors_event_t event; 
  bno.getEvent(&event);
  
  float Xaxis = event.orientation.x;
  float Yaxis = event.orientation.y;
 
  gyroArray.point.x = Xaxis;
  gyroArray.point.y = Yaxis;

  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  gyroArray.point.z = system_status;

  rosTime = nh.now();
  gyroArray.header.stamp = rosTime;
  
  pub.publish(&gyroArray);
  nh.spinOnce();
}
