#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

geometry_msgs::Twist vel;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    int i = 0;
    int j = 0;
    printf("Start");
    for(i=80;i<180;i++)
    {
        if(scan->ranges[i] < 0.28 && scan->ranges[i] > 0)
        {
            vel.angular.z = 0.3;
        }
    }
    for(j=180;j<280;j++)
    {
        if(scan->ranges[j] < 0.28 && scan->ranges[j] > 0)
        {
            vel.angular.z = -0.3;
        }
    }
    ROS_INFO("%f, %f", vel.linear.x, vel.angular.z);
}

int main(int argc, char **argv)
{
    using namespace ros;
    init(argc, argv, "go_stop");
    NodeHandle nh;
    Publisher go_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    Subscriber laser_sub = nh.subscribe("/scan", 1, scanCallback);

    Rate loop_rate(10);

    vel.linear.x = 0.1;
    while(ok())
    {
        go_pub.publish(vel);
        spinOnce();
        loop_rate.sleep();
    }

    return 0;
}