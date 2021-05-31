#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/Float32.h>

static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

ros::NodeHandle nh;

std_msgs::Float32 msg;
ros::Publisher gpsPub("gpsPub", &msg);

void setup()
{
  ss.begin(GPSBaud);
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(gpsPub);

}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();
      delay(500);

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    while(true);
  }
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    msg.data = gps.location.lat();
    gpsPub.publish( &msg );
    
    msg.data = gps.location.lng();
    gpsPub.publish( &msg );
  }

  nh.spinOnce();
  delay(1000);
  
}
