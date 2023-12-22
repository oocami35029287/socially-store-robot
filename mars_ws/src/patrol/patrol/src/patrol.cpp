#include <iostream>
#include <math.h>
#include <signal.h>

// ROS
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <std_msgs/Int8.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PatrolNode
{
private:
    // double cur_roll, cur_pitch, cur_yaw, start_roll, start_pitch, start_yaw, theta;
    int reach = 0;
public:
    // ros::Publisher pub_mb_goal;
    PatrolNode(ros::NodeHandle nh);
    void Target_one();
    void Target_two();
    void Target_three();
    void Target_four();
    void Target_five();
    void Target_six();
    void Target_home();
    void Regulate();
    void Setting_patrol_path();
    void replace_finish_callback(std_msgs::Int8 msg);
    // void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    ros::Publisher vel_pub;
    ros::Publisher loc_pub;
    // ros::Subscriber odom_sub;
    ros::Subscriber replace_finish_sub;
    geometry_msgs::Twist cmd_twist;
    // nav_msgs::Odometry start_odom, cur_odom;
    std_msgs::Int8 location;
};

PatrolNode::PatrolNode(ros::NodeHandle nh)
{
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    vel_pub = nh.advertise<geometry_msgs::Twist>("/mob_plat/cmd_vel", 1);
    loc_pub = nh.advertise<std_msgs::Int8>("/mob_plat/location", 1);
    replace_finish_sub = nh.subscribe("/replacement_finished", 1, &PatrolNode::replace_finish_callback, this);
    // odom_sub = nh.subscribe("/odom_combined", 1, &PatrolNode::odom_callback, this);
    Setting_patrol_path();
}
/*
void PatrolNode::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{   
    cout<<"Position"<<endl;
    cout<<"X: "<<msg->pose.pose.position.x<<endl;
    cout<<"Y: "<<msg->pose.pose.position.y<<endl;
    cout<<"Z: "<<msg->pose.pose.position.z<<endl;
    cout<<"Orientation"<<endl;
    cout<<"rx: "<<msg->pose.pose.orientation.x<<endl;
    cout<<"ry: "<<msg->pose.pose.orientation.y<<endl;
    cout<<"rz: "<<msg->pose.pose.orientation.z<<endl;

    if(reach)
    {
        start_odom.pose.pose.position.x = msg->pose.pose.position.x;
        start_odom.pose.pose.position.y = msg->pose.pose.position.y;
        start_odom.pose.pose.position.z = msg->pose.pose.position.z;
        start_odom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
        start_odom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
        start_odom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
        start_odom.pose.pose.orientation.w = msg->pose.pose.orientation.w;
        start_odom.twist.twist.linear.x = msg->twist.twist.linear.x;
        start_odom.twist.twist.linear.y = msg->twist.twist.linear.y;
        start_odom.twist.twist.linear.z = msg->twist.twist.linear.z;
        start_odom.twist.twist.angular.x = msg->twist.twist.angular.x;
        start_odom.twist.twist.angular.y = msg->twist.twist.angular.y;
        start_odom.twist.twist.angular.z = msg->twist.twist.angular.z;
        tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(start_roll, start_pitch, start_yaw);
        reach = false;
    }
    else
    {
        cur_odom.pose.pose.position.x = msg->pose.pose.position.x;
        cur_odom.pose.pose.position.y = msg->pose.pose.position.y;
        cur_odom.pose.pose.position.z = msg->pose.pose.position.z;
        cur_odom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
        cur_odom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
        cur_odom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
        cur_odom.pose.pose.orientation.w = msg->pose.pose.orientation.w;
        cur_odom.twist.twist.linear.x = msg->twist.twist.linear.x;
        cur_odom.twist.twist.linear.y = msg->twist.twist.linear.y;
        cur_odom.twist.twist.linear.z = msg->twist.twist.linear.z;
        cur_odom.twist.twist.angular.x = msg->twist.twist.angular.x;
        cur_odom.twist.twist.angular.y = msg->twist.twist.angular.y;
        cur_odom.twist.twist.angular.z = msg->twist.twist.angular.z;

        tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(cur_roll, cur_pitch, cur_yaw);
    }
}*/

void PatrolNode::replace_finish_callback(std_msgs::Int8 msg)
{
    reach = msg.data;
}  

void PatrolNode::Target_one()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // goal.target_pose.pose.position.x = 7.288;
    // goal.target_pose.pose.position.y = -1.459;
    // goal.target_pose.pose.position.z = 0.000;
    // goal.target_pose.pose.orientation.x = 0.000;
    // goal.target_pose.pose.orientation.y = 0.000;
    // goal.target_pose.pose.orientation.z = 0.850;
    // goal.target_pose.pose.orientation.w = 0.526;

    goal.target_pose.pose.position.x = 6.645;
    goal.target_pose.pose.position.y = 3.263;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = 1.000;
    goal.target_pose.pose.orientation.w = -0.022;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");

    location.data = 1;
    loc_pub.publish(location);
}

void PatrolNode::Target_two()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // goal.target_pose.pose.position.x = 3.942;
    // goal.target_pose.pose.position.y = 1.675;
    // goal.target_pose.pose.position.z = 0.000;
    // goal.target_pose.pose.orientation.x = 0.000;
    // goal.target_pose.pose.orientation.y = 0.000;
    // goal.target_pose.pose.orientation.z = 0.984;
    // goal.target_pose.pose.orientation.w = -0.179;

    goal.target_pose.pose.position.x = 0.575;
    goal.target_pose.pose.position.y = 2.443;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = 0.900;
    goal.target_pose.pose.orientation.w = -0.436;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(23.0));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");

    location.data = 2;
    loc_pub.publish(location);
}

void PatrolNode::Target_three()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // goal.target_pose.pose.position.x = 1.740;
    // goal.target_pose.pose.position.y = 0.251;
    // goal.target_pose.pose.position.z = 0.000;
    // goal.target_pose.pose.orientation.x = 0.000;
    // goal.target_pose.pose.orientation.y = 0.000;
    // goal.target_pose.pose.orientation.z = -0.530;
    // goal.target_pose.pose.orientation.w = 0.848;

    goal.target_pose.pose.position.x = 0.613;
    goal.target_pose.pose.position.y = -0.041;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = 0.005;
    goal.target_pose.pose.orientation.w = 1.000;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(7.0));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");

    location.data = 3;
    loc_pub.publish(location);
}

void PatrolNode::Target_four()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // goal.target_pose.pose.position.x = 3.281;
    // goal.target_pose.pose.position.y = -3.910;
    // goal.target_pose.pose.position.z = 0.000;
    // goal.target_pose.pose.orientation.x = 0.000;
    // goal.target_pose.pose.orientation.y = 0.000;
    // goal.target_pose.pose.orientation.z = -0.546;
    // goal.target_pose.pose.orientation.w = 0.838;

    goal.target_pose.pose.position.x = 5.700;
    goal.target_pose.pose.position.y = -0.366;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = 0.030;
    goal.target_pose.pose.orientation.w = 1.000;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(30.0));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");

    location.data = 4;
    loc_pub.publish(location);
}

void PatrolNode::Target_five()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // goal.target_pose.pose.position.x = 6.157;
    // goal.target_pose.pose.position.y = -6.063;
    // goal.target_pose.pose.position.z = 0.000;
    // goal.target_pose.pose.orientation.x = 0.000;
    // goal.target_pose.pose.orientation.y = 0.000;
    // goal.target_pose.pose.orientation.z = 0.218;
    // goal.target_pose.pose.orientation.w = 0.976;

    goal.target_pose.pose.position.x = 8.889;
    goal.target_pose.pose.position.y = 1.525;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = 0.718;
    goal.target_pose.pose.orientation.w = 0.696;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(10.0));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");

    location.data = 5;
    loc_pub.publish(location);
}

void PatrolNode::Target_six()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // goal.target_pose.pose.position.x = 8.215;
    // goal.target_pose.pose.position.y = -4.161;
    // goal.target_pose.pose.position.z = 0.000;
    // goal.target_pose.pose.orientation.x = 0.000;
    // goal.target_pose.pose.orientation.y = 0.000;
    // goal.target_pose.pose.orientation.z = 0.746;
    // goal.target_pose.pose.orientation.w = 0.666;

    goal.target_pose.pose.position.x = 7.972;
    goal.target_pose.pose.position.y = 2.771;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = 0.995;
    goal.target_pose.pose.orientation.w = 0.095;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(6.0));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");

    location.data = 6;
    loc_pub.publish(location);
}

void PatrolNode::Target_home()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.353;
    goal.target_pose.pose.position.y = 1.130;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.226;
    goal.target_pose.pose.orientation.w = 0.974;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("The base failed to move to the goal");
}

void PatrolNode::Regulate()
{
    tf::TransformListener listener;
    tf::StampedTransform tf_l;
    // string tf_l_name = "/tag_369";

    static tf::TransformBroadcaster br;
    tf::Transform tf_b;    
    // string tf_b_name = "/target_four";
    

    // listener.waitForTransform("/base_link", tf_l_name, ros::Time(0), ros::Duration(2.0));
    // listener.lookupTransform("/base_link", tf_l_name, ros::Time(0), tf_l);

    // tf_b.setOrigin(tf::Vector3(-0.630, -(tf_l.getOrigin().getZ()), -0.320));
    // tf_b.setRotation(tf::Quaternion(-0.500, 0.500, 0.500, 0.500));
    // br.sendTransform(tf::StampedTransform(tf_b, ros::Time::now(), "/tag_369", tf_b_name));

    // listener.waitForTransform("/base_link", tf_b_name, ros::Time(0), ros::Duration(2.0));
    // listener.lookupTransform("/base_link", tf_b_name, ros::Time(0), tf_l);

    // cout<<"Relative Pose"<<endl;
    // cout<<"X: "<<tf_l.getOrigin().getX()<<endl;
    // cout<<"Y: "<<tf_l.getOrigin().getY()<<endl;
    // cout<<"Z: "<<tf_l.getOrigin().getZ()<<endl;
    /*
    cout<<"start_x: "<<start_odom.pose.pose.position.x<<endl;
    cout<<"start_y: "<<start_odom.pose.pose.position.y<<endl;
    cout<<"start row pitch yaw: "<<start_roll<<" "<<start_pitch<<" "<<start_yaw<<endl;
    double omega;
    bool ft = true;
    
    if(tf_l.getOrigin().getY() > 0)
    {
        omega = 0.05;
        cmd_twist.linear.x = 0.0;
        cmd_twist.linear.y = 0.0;
        cmd_twist.linear.z = 0.0;
        cmd_twist.angular.x = 0.0;
        cmd_twist.angular.y = 0.0;
        cmd_twist.angular.z = omega;
        theta = acos(tf_l.getOrigin().getX()/sqrt(pow(tf_l.getOrigin().getY(), 2) + pow(tf_l.getOrigin().getX(), 2)));
    }
    else
    {
        omega = -0.05;
        cmd_twist.linear.x = 0.0;
        cmd_twist.linear.y = 0.0;
        cmd_twist.linear.z = 0.0;
        cmd_twist.angular.x = 0.0;
        cmd_twist.angular.y = 0.0;
        cmd_twist.angular.z = omega;
        theta = -acos(tf_l.getOrigin().getX()/sqrt(pow(tf_l.getOrigin().getY(), 2) + pow(tf_l.getOrigin().getX(), 2)));
    }
    cout<<theta<<endl;

    while(1)
    {
        vel_pub.publish(cmd_twist);

        if(cur_yaw - start_yaw < theta && ft)
        {
            continue;
        }
        else if(cur_odom.pose.pose.position.x - start_odom.pose.pose.position.x < tf_l.getOrigin().getX())
        {
            ft = false;
            cmd_twist.linear.x = tf_l.getOrigin().getX()/5.0;
            cmd_twist.linear.y = 0.0;
            cmd_twist.linear.z = 0.0;
            cmd_twist.angular.x = 0.0;
            cmd_twist.angular.y = 0.0;
            cmd_twist.angular.z = 0.0;
        }
        else if(cur_yaw > 0.0003)
        {
            cmd_twist.linear.x = 0.0;
            cmd_twist.linear.y = 0.0;
            cmd_twist.linear.z = 0.0;
            cmd_twist.angular.x = 0.0;
            cmd_twist.angular.y = 0.0;
            cmd_twist.angular.z = -omega;
        }
        else
        {
            cmd_twist.linear.x = 0.0;
            cmd_twist.linear.y = 0.0;
            cmd_twist.linear.z = 0.0;
            cmd_twist.angular.x = 0.0;
            cmd_twist.angular.y = 0.0;
            cmd_twist.angular.z = 0.0;
            vel_pub.publish(cmd_twist);
            cout<<"end_x: "<<cur_odom.pose.pose.position.x<<endl;
            cout<<"end_y: "<<cur_odom.pose.pose.position.y<<endl;
            cout<<"end row pitch yaw: "<<cur_roll<<" "<<cur_pitch<<" "<<cur_yaw<<endl;
            break;
        }
    }*/

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 5.700 + tf_l.getOrigin().getX();
    goal.target_pose.pose.position.y = -0.366 + tf_l.getOrigin().getY();
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = 0.004;
    goal.target_pose.pose.orientation.w = 1.000;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(30.0));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");

    location.data = 41;
    loc_pub.publish(location);
}

void PatrolNode::Setting_patrol_path()
{
    while(1)
    {
        char c = getchar();

        if(c == '1')
            Target_one();

        if(c == '2')
            Target_two();

        if(c == '3')
            Target_three();

        if(c == '4')
            Target_four();

        if(c == '5')
            Target_five();

        if(c == '6')
            Target_six();

        if(c == 's')
        {
            while(1)
            {
                if(reach == 1)
                {
                    Target_two();
                    Target_three();
                    Target_four();
                    Regulate();
                    break;
                }
            }
            
            while(1)
            {
                if(reach == 2)
                {
                    Target_five();
                    Target_six();
                    Target_one();
                    break;
                }
            }
        }
        
        if(c == 't')
        {
            Target_four();
            Regulate();
            // while(1)
            // {
            //     if(reach)
            //     {
            //         Target_five();
            //         Target_six();
            //         Target_one();
            //         break;
            //     }
            // }
        }

        if(c == 'l')
        {
            location.data = 41;
            loc_pub.publish(location);
        }

        if(c == 'q')
        {
            cout<<"End of patrol"<<endl;
            break;
        }

        if(c == 'h')
        {
            Target_home();
            cout<<"Mars go back to home"<<endl;
            break;
        }
    }    
}

PatrolNode *pa;

void sigHandler(int signum) 
{
    delete pa;

    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "patrol_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigHandler);
    pa = new PatrolNode(nh);
    ros::spin();
    
    return 0;
}