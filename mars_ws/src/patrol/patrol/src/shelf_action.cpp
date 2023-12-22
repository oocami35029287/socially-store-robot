#include <iostream>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

//ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Custom msg & srv
#include <detection_msgs/Detection2DTrig.h>
#include <detection_msgs/Det3D.h>
#include <detection_msgs/Det3DArray.h>

using namespace std;

class shelf_action
{
private:
    char target_number;
    const string PLANNING_GROUP = "tm_arm";
    const vector<double> home_p = {-M_PI_2, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.0};
    const vector<double> joint_sf_scan1 = {-1.564, -0.531, 2.223, -1.681, 1.621, 0.000};
    const vector<double> joint_sf_scan2 = {-0.870, 0.014, 2.161, -2.185, 0.888, 0.000};
    
    float *x_tmp, *y_tmp, *z_tmp;
    int count = 0;
    detection_msgs::Det3DArray target_bias;
    detection_msgs::Det3D bias;
    bool reach = false;
public:
    shelf_action(ros::NodeHandle nh);
    void det_callback(detection_msgs::Det3DArray msg);
    void Position_Manager();

    ros::Publisher mis_pub;
    ros::Subscriber det_sub;
    geometry_msgs::Pose current_pose;
};

shelf_action::shelf_action(ros::NodeHandle nh)
{  
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    x_tmp = new float[10] ();
    y_tmp = new float[10] ();
    z_tmp = new float[10] ();

    mis_pub = nh.advertise<detection_msgs::Det3DArray>("/missing_bottle", 1);
    det_sub = nh.subscribe("/scan_clustering_node/det3d_result", 1, &shelf_action::det_callback, this);
    Position_Manager();
}

void shelf_action::det_callback(detection_msgs::Det3DArray msg)
{
    if(reach)
    {
        target_bias.dets_list.clear();

        while(count < 10)
        {
            for(int i=0; i<msg.dets_list.size(); i++)
            {
                x_tmp[i] += msg.dets_list[i].x;
                y_tmp[i] += msg.dets_list[i].y;
                z_tmp[i] += msg.dets_list[i].z;
            }
            count++;
        }

        count = 0;

        for(int i=0; i<msg.dets_list.size(); i++)
        {
            bias.class_name = msg.dets_list[i].class_name;
            bias.class_id = msg.dets_list[i].class_id;
            bias.x = x_tmp[i]/10;
            bias.y = y_tmp[i]/10;
            bias.z = z_tmp[i]/10;
            x_tmp[i] = 0;
            y_tmp[i] = 0;
            z_tmp[i] = 0;
            target_bias.dets_list.push_back(bias);
        }

        cout<<"There are "<<target_bias.dets_list.size()<<" bottles"<<endl;
        for(int i=0; i<target_bias.dets_list.size(); i++)
        {
            printf("No.%d: ", target_bias.dets_list[i].class_id);
            cout<<target_bias.dets_list[i].class_name<<endl;
        }
        
        reach = false;
    }

    mis_pub.publish(target_bias);
}

void shelf_action::Position_Manager()
{
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group.setStartState(*move_group.getCurrentState());

    while(1)
    {
        target_number = getchar();

        if(target_number == 'h')
        {
            ROS_INFO("GO HOME");

            joint_group_positions = home_p;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("DONE");
        }
        if(target_number == 's')
        {
            ROS_INFO("GO SCANNING POINT 1");
            
            joint_group_positions = joint_sf_scan1;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("DONE");

            sleep(1);
            reach = true;
        }
        if(target_number == 'q')
        {
            ROS_INFO("QUIT");
            break;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shelf_action");
    ros::NodeHandle nh;
    shelf_action node(nh);
    ros::spin();
    return 0;
}