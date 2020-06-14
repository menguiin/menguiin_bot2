#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

int main(int argc, char **argv)
{
    using namespace ros;
    init(argc, argv, "motor_on");
    NodeHandle nh;
    Publisher on_pub = nh.advertise<std_msgs::Bool>("/motor_power", 1);
    Publisher reset_pub = nh.advertise<std_msgs::Empty>("/reset", 1);
    Rate loop_rate(10);
    std_msgs::Bool power;
    std_msgs::Empty reset_odom;

    while(ok())
    {
        int is_on;
        printf("On(1)/Off(0): "); 
        scanf("%d", &is_on);
        if(is_on == 1)
        {
            power.data = true;
        }
        else
        {
            power.data = false;
            reset_pub.publish(reset_odom);
        }

        on_pub.publish(power);
        loop_rate.sleep();
    }

    return 0;
}