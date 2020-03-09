#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/Float64.h"
#include <string>

using namespace std;

int baudrate;
string portname;

double steering, speed;

void steeringCallback(const std_msgs::Float64::ConstPtr &msg)
{
    steering = msg->data;
}

void speedCallback(const std_msgs::Float64::ConstPtr &msg)
{
    speed = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CJC");
    ros::NodeHandle n;

    n.param<string>("port", portname, "/dev/ttyUSB1");
    n.param<int>("baud", baudrate, 9600);

    steering = 0.;
    speed = 0.;

    ros::Subscriber consigne_sub = n.subscribe("steeringSetpoint", 10, steeringCallback);
    ros::Subscriber consigne_sub = n.subscribe("speedSetpoint", 10, speedCallback);

    ros::Rate loop_rate(25);
    while (ros::ok())
    {
        ros::spinOnce();
        
        //TODO : send command

        loop_rate.sleep();
    }
    return 0;
}