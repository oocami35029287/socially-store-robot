#include <iostream>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

using namespace std;

static const string COLOR_GREEN = "\e[0;32m";
static const string COLOR_NC = "\e[0m";

int first_t = 0;
double k = 0;
double last_max_speed;

class NavigationController
{
private:
    double max_speed;
    int NumOfPeople;
public:
    NavigationController(ros::NodeHandle nh);
    void nav_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twist);
    void max_vel_callback(const std_msgs::Float64::ConstPtr& msg);
    void det_callback(const std_msgs::Int8::ConstPtr& msg);

    ros::Publisher cmd_vel_pub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber max_speed_sub;
    ros::Subscriber det_sub;
};

NavigationController::NavigationController(ros::NodeHandle nh)
{
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mob_plat/cmd_vel", 1);
    cmd_vel_sub = nh.subscribe("/mobile/cmd_vel", 1, &NavigationController::nav_cmd_vel_callback, this);
    det_sub = nh.subscribe("/people/num", 1, &NavigationController::det_callback, this);
    max_speed_sub = nh.subscribe("/navigation_controller/max_speed", 1, &NavigationController::max_vel_callback, this);
    cout << COLOR_GREEN << ros::this_node::getName() << " is ready." << COLOR_NC << endl;
}

void NavigationController::nav_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twist)
{
    double mb_cmd_vel = sqrt(pow(twist->linear.x, 2) + pow(twist->linear.y, 2));
    geometry_msgs::Twist new_cmd_vel;

    if(NumOfPeople == 0)
    {
        if(first_t == 0)
        {
            last_max_speed = max_speed;
            first_t = 1;
        }
        if(last_max_speed < max_speed )
        {
            last_max_speed += (max_speed - last_max_speed)/50 + k;
            k += 0.01;
            // last_max_speed = max_speed;
        }
    }
    else
        last_max_speed = max_speed;

    if(mb_cmd_vel > last_max_speed)
    {
        new_cmd_vel.linear.x = twist->linear.x * (last_max_speed / mb_cmd_vel);
        new_cmd_vel.linear.y = twist->linear.y * (last_max_speed / mb_cmd_vel);
    }
    else
    {
        new_cmd_vel.linear.x = twist->linear.x;
        new_cmd_vel.linear.y = twist->linear.y;
    }
    new_cmd_vel.linear.z = twist->linear.z;
    new_cmd_vel.angular.x = twist->angular.x;
    new_cmd_vel.angular.y = twist->angular.y;

    if(twist->angular.z < 0.08 && twist->angular.z > -0.08)
        new_cmd_vel.angular.z = 0;
    else
        new_cmd_vel.angular.z = twist->angular.z;

    cmd_vel_pub.publish(new_cmd_vel);
    // ROS_INFO("Sending new cmd vel: %lf\n", new_cmd_vel.linear.x);
    // ROS_INFO("Sending new cmd vel: %lf\n", sqrt(pow(new_cmd_vel.linear.x, 2) + pow(new_cmd_vel.linear.y, 2)));
}

void NavigationController::det_callback(const std_msgs::Int8::ConstPtr& msg)
{
    NumOfPeople = msg->data;
    // ROS_INFO("There are %d people\n", NumOfPeople);
}

void NavigationController::max_vel_callback(const std_msgs::Float64::ConstPtr& msg)
{
    // ROS_INFO("Setting Max Speed to %lf\n", msg->data);
    if(msg->data > 0.3)
        max_speed = 0.4;
    else
        max_speed = msg->data;
}

NavigationController *na;

void sigHandler(int signum) 
{
    delete na;

    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_speed_controller");
    ros::NodeHandle nh;
    na = new NavigationController(nh);
    ros::spin();
    return 0;
}
