#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#define LINEAR_VEL = 0.22;
#define ANGULAR_VEL = 1.0;

static geometry_msgs::Twist vel;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    float regions, angle;
    for(angle=0; angle<)
}

int main(int argc, char **argv)
{
    using namespace ros;
    init(argc, argv, "motor_on");
    NodeHandle nh;
    Subscriber laser_sub = nh.Subscriber("/scan", 1, scanCallback);
    Publisher vel_pub = nh.advertise<geometry::Twist>("/cmd_vel", 1);
    Rate loop_rate(10);

    while(ok())
    {
        spinOnce();
        loop_rate.sleep();
    }

    return 0;
}