#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <sstream>
#include <math.h>

geometry_msgs::Twist twist_last;
bool twist_enable;

void twist_stamped_callback(const geometry_msgs::Twist& twist_msg){
    twist_last = twist_msg;
    twist_enable = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    //publisher
    ros::Publisher velocty_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    //subscriber
    ros::Subscriber test_sub = nh.subscribe("cmd_vel", 10, twist_stamped_callback);

    geometry_msgs::Twist twist_last;
    twist_last.linear.x = 3.0;
    twist_last.linear.y = 0.0;
    twist_last.linear.z = 0.0;
    twist_last.angular.x = 0.0;
    twist_last.angular.y = 0.0;
    twist_last.angular.z = 0.5;
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
        velocty_pub.publish(twist_last);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}