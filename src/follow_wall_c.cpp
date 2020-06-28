#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>


#define LINEAR_VEL 0.22
#define ANGULAR_VEL 1.0

geometry_msgs::Twist vel;

void scanCallbak(sensor_msgs::LaserScan::ConstPtr &scan)
{
    int angle;
    double left_distance, right_distance, front_distance, fleft_distance, fright_distance;
    double scan_filter = [360];

    for(angle=0;angle<=359;angle++)
    {
        scan_filter[angle] = scan.ranges[angle];
        if(scan.ranges>0)
        {
            scan_filter[angle] = 3.5;
        }
    }

    for(angle=0;angle<=359;angle++)
    {
        scan_filter
    }
}


int main(int argc, char **argv)
{
    using namespace ros;
    init(argc, argv, "follow_wall");
    NodeHandle nh;
    Subscriber nh.subscribe("/scan", 1, scanCallback);
    Publisher nh.advertise<geometry::Twist>("/cmd_vel", 1);

    while(ok())
    {
        spinOnce();
        loop_rate.sleep();
    }

    return 0;
}