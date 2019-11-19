#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh; //if you have any work with ros in arduino ide, you need to write this

void messageCb(const std_msgs::Float32 &joyinputy, std_msgs::Float32)
{ // determines if the rover goes forward or backward
    if (abs(joyinputy.data) < 0.15)
    {
        halt();
    }
    if (joyinputy.data != 0)
    {
        if (joyinputy.data > 0)
        {
            enableLeft();
        }
        if (joyinputy.data < 0)
        {
            enableRight();
        }
        move(255 * abs(joyinputy.data));
    }
}

void messageCb1(const std_msgs::Float32 &joyinputx, const std_msgs::Float32)
{ // determines if the rover turns right or left
    if (abs(joyinputx.data) < 0.15)
    {
        halt();
    }
    if (joyinputx.data != 0)
    {
        if (joyinputx.data > 0)
        {
            enableforward();
        }
        if (joyinputx.data < 0)
        {
            enablereverse();
        }
        move(255 * abs(joyinputx.data));
    }
}

ros::Subscriber<std_msgs::Float32> sub("joyinputy", &messageCb);
ros::Subscriber<std_msgs::Float32> sub1("joyinputx", &messageCb1);

void setup()
{
    for (int i = 8; i <= 13; i++)
    {
        pinMode(i, OUTPUT);
    }
    for (int i = 48; i <= 53; i++)
    {
        pinMode(i, OUTPUT);
    }
    for (int i = 2; i <= 7; i++)
    {
        pinMode(i, OUTPUT);
    }

    halt(); //it is for stopping rover
    nh.initNode();
    nh.subscribe(sub); //creating subscribers
    nh.subscribe(sub1);
    Serial.begin(9600);
}
void loop()
{
    nh.spinOnce(); //ROS only processes your callbacks when you tell it to with ros::spinOnce()
    delay(1);
}

void move(int pwm)
{
    for (int i = 2; i <= 7; i++)
    {
        analogWrite(i, pwm);
    }
}

void halt()
{
    for (int k = 2; k <= 7; k++)
    {
        analogWrite(k, 0);
    }
    for (int i = 48; i <= 53; i++)
    {
        digitalWrite(i, LOW);
    }

    for (int j = 22; j <= 27; j++)
    {
        digitalWrite(j, LOW);
    }
}

void enableRight()
{
    digitalWrite(13, LOW);
    digitalWrite(10, LOW);
    digitalWrite(9,  LOW);
    digitalWrite(53, HIGH);
    digitalWrite(51, HIGH);
    digitalWrite(49, HIGH);

    digitalWrite(12, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(8,  HIGH);
    digitalWrite(52, LOW);
    digitalWrite(50, LOW);
    digitalWrite(48, LOW);
}

void enableLeft()
{
    digitalWrite(13, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(9,  HIGH);
    digitalWrite(53, LOW);
    digitalWrite(51, LOW);
    digitalWrite(49, LOW);

    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8,  LOW);
    digitalWrite(52, HIGH);
    digitalWrite(50, HIGH);
    digitalWrite(48, HIGH);
}

void enableforward()
{
    digitalWrite(13, LOW);
    digitalWrite(10, LOW);
    digitalWrite(9,  LOW);
    digitalWrite(53, LOW);
    digitalWrite(51, LOW);
    digitalWrite(49, LOW);

    digitalWrite(12, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(8,  HIGH);
    digitalWrite(52, HIGH);
    digitalWrite(50, HIGH);
    digitalWrite(48, HIGH);
}

void enablereverse()
{
    digitalWrite(13, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(9,  HIGH);
    digitalWrite(53, HIGH);
    digitalWrite(51, HIGH);
    digitalWrite(49, HIGH);

    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(8,  LOW);
    digitalWrite(52, LOW);
    digitalWrite(50, LOW);
    digitalWrite(48, LOW);
}
