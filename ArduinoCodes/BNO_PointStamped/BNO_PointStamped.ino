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

imu::Vector<3> v;
float sensor_published_data[6];

geometry_msgs::PointStamped gyroArray;
geometry_msgs::PointStamped accArray;

ros::Publisher pub("bno_gyro", &gyroArray);
ros::Publisher pub2("bno_acceleration", &accArray);

void setup(){
  
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(pub);
  nh.advertise(pub2);
  delay(10);
  
  bno.begin();

  bno.setExtCrystalUse(true);
}

void loop(){
  sensors_event_t event; 
  bno.getEvent(&event);
  
  float Xaxis = event.orientation.x;
  float Yaxis = event.orientation.y;
  float Zaxis = event.orientation.z;
 
  gyroArray.point.x = Xaxis;
  gyroArray.point.y = Yaxis;
  gyroArray.point.z = Zaxis;
  
  v = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  accArray.point.x = (float)(v.x());
  accArray.point.y = (float)(v.y());
  accArray.point.z = (float)(v.z());
  
  rosTime = nh.now();
  gyroArray.header.stamp = rosTime;
  accArray.header.stamp = rosTime;
  
  pub.publish(&gyroArray);
  pub2.publish(&accArray);
  nh.spinOnce();

}
