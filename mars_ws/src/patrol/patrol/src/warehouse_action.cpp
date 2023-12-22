#include <iostream>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

//ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h> 
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Custom msg & srv
#include <detection_msgs/Detection2DTrig.h>
#include <detection_msgs/Det3D.h>
#include <detection_msgs/Det3DArray.h>
#include <detection_msgs/StringArray.h>

using namespace std;

class warehouse_action
{
private:
    std_msgs::Bool grip;
    const string PLANNING_GROUP = "tm_arm";
    char command;
    const vector<double> home_p = {-M_PI_2, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.000};
    const vector<double> home_p_2 = {-M_PI_2, -M_PI_4, M_PI*2/3, -1.309, M_PI_2, 0.000};
    const vector<double> home_r = {M_PI, M_PI/3, -M_PI_4*3, 1.309, -M_PI_2, 0.0};
    const vector<double> middle1_a = {-M_PI, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.000};
    const vector<double> middle1_b = {-2.552, -0.527, 2.218, -1.679, 1.009, 0.000};
    const vector<double> middle2_a = {0, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.000};
    const vector<double> middle2_b = {3.344, -1.023, 2.259, -1.218, 1.285, 0.000};
    const vector<double> middler_p = {M_PI_2, -M_PI/3, M_PI_4*3, -1.309, -M_PI_2, 0.0};
    const vector<double> middlep_r = {0.000, 0.000, 0.000, 0.000, -M_PI_2, 0.0};
    const vector<double> position_s = {0, M_PI/3, -M_PI_4*3, 1.309, 0.000, 0.000}; //M_PI_4*3

    const vector<double> joint_sh_scan = {-M_PI_2, -M_PI_4, M_PI*2/3, -1.309, 2.800, 0.000};

    // const vector<double> joint_wh_scan3 = {-0.305, 0.485, 1.635, -1.953, 0.293, 0.160};
    // const vector<double> joint_wh_scan2 = {-0.772, -0.048, 2.320, -2.200, 0.758, 0.056};
    // const vector<double> joint_wh_scan1 = {-1.564, 0.014, 2.261, -2.227, 1.549, 0.003};

    const vector<double> joint_wh_scan2 = {-0.362, 0.002, 2.369, -2.270, 0.350, 0.133};
    const vector<double> joint_wh_scan1 = {-1.564, -0.518, 2.621, -2.082, M_PI_2, 0.000};

    const vector<double> joint_place1 = {-3.133, 0.180, 2.279, -2.450, 1.545, 0.000};
    const vector<double> joint_place1_mid = {-3.133, -0.282, 2.255, -2.234, 1.547, 0.002};
    const vector<double> joint_place2 = {-2.583, 0.183, 2.462, -2.644, 1.019, 0.000};
    const vector<double> joint_place2_mid = {-2.822, -0.355, 2.231, -2.045, 1.473, 0.016};
    const vector<double> joint_place3 = {-0.020, 0.269, -1.979, -1.416, 1.551, 3.130};
    const vector<double> joint_place3_mid = {-0.028, 0.410, -1.616, -1.813, 1.544, 3.041};
    const vector<double> joint_place4 = {3.181, -0.120,2.709, -2.586, 1.454, 0.034};
    const vector<double> joint_place4_mid = {3.344, -1.023, 2.259, -1.218, 1.285, 0.028};
    const vector<double> joint_place5 = {0.012, 0.647, -2.194, -1.579, 1.552, 3.040};
    const vector<double> joint_place5_mid = {0.002, 0.755, -1.787, -2.089, 1.533, 3.035};

    const vector<double> joint_place_tag = {-1.558, -0.568, 2.070, -1.501, 1.555, 0.000};

    float *x_tmp, *y_tmp, *z_tmp;
    int count = 0, target_amount = 0, target_number = 0;
    bool find = false, grab = false, collect = false, reach = false;
    std_msgs::Int8 replacement_finished;
    detection_msgs::Det3DArray target_bias;
    detection_msgs::Det3D bias;
public:
    warehouse_action(ros::NodeHandle nh);
    void det_callback(detection_msgs::Det3DArray msg);
    void mis_callback(detection_msgs::StringArray msg);
    void loc_callback(std_msgs::Int8 msg);
    void Position_Manager();

    ros::Publisher gripper_pub;
    ros::Publisher mis_pub;
    ros::Publisher prod_pub;
    ros::Publisher replace_finish_pub;
    ros::Subscriber det_sub;
    ros::Subscriber mis_sub;
    ros::Subscriber loc_sub;
    geometry_msgs::Pose current_pose, tag_pose;
    detection_msgs::StringArray mis_list, sa, prod_list;
};

warehouse_action::warehouse_action(ros::NodeHandle nh)
{  
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    x_tmp = new float[10] ();
    y_tmp = new float[10] ();
    z_tmp = new float[10] ();

    gripper_pub = nh.advertise<std_msgs::Bool>("/gripper/cmd_gripper", 1);
    prod_pub = nh.advertise<detection_msgs::StringArray>("/product/information", 1);
    replace_finish_pub = nh.advertise<std_msgs::Int8>("/replacement_finished", 1);
    mis_sub = nh.subscribe("/missing_bottle", 1, &warehouse_action::mis_callback, this);
    mis_pub = nh.advertise<detection_msgs::StringArray>("/missing_bottle", 1);
    string s;
    s = "Soda";
    sa.strings.push_back(s);
    s = "MineralWater";
    sa.strings.push_back(s);
    s = "PinkSoda";
    sa.strings.push_back(s);
    mis_pub.publish(sa);
    replacement_finished.data = 0;
    replace_finish_pub.publish(replacement_finished);
    loc_sub = nh.subscribe("/mob_plat/location", 1, &warehouse_action::loc_callback, this);
    det_sub = nh.subscribe("/scan_clustering_node/det3d_result", 1, &warehouse_action::det_callback, this);
    Position_Manager();
}

void warehouse_action::det_callback(detection_msgs::Det3DArray msg)
{
    if(msg.dets_list.size() != 0 && reach)
        find = true;
    else
        find = false;

    if(find && !collect)
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
            for(int j=0; j<mis_list.strings.size(); j++)
            {
                if(mis_list.strings[j] == msg.dets_list[i].class_name)
                {
                    bias.class_name = msg.dets_list[i].class_name;
                    cout<<bias.class_name<<endl;
                    bias.class_id = msg.dets_list[i].class_id;
                    bias.x = x_tmp[i]/10;
                    bias.y = y_tmp[i]/10;
                    bias.z = z_tmp[i]/10;
                    x_tmp[i] = 0;
                    y_tmp[i] = 0;
                    z_tmp[i] = 0;
                    target_bias.dets_list.push_back(bias);
                }
            }
        }
        collect = true;
    }
    
    if(collect)
    {   
        target_amount = target_bias.dets_list.size();

        for(int i=0; i<target_bias.dets_list.size(); i++)
        {
            static tf::TransformBroadcaster br;
            tf::Transform tf1;
            string tf_name1 = "tm_target_pose";
            tf1.setOrigin(tf::Vector3(target_bias.dets_list[i].x, target_bias.dets_list[i].y, target_bias.dets_list[i].z));
            tf1.setRotation(tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w));
            br.sendTransform(tf::StampedTransform(tf1, ros::Time::now(), "camera1_link", tf_name1 + to_string(i)));

            string b;
            b = target_bias.dets_list[i].class_name;
            prod_list.strings.push_back(b);
        }
    }
}

void warehouse_action::mis_callback(detection_msgs::StringArray msg)
{
    mis_list.strings = msg.strings;

    for(int i=0; i<msg.strings.size(); i++)
    {
        cout<<msg.strings[i]<<endl;
    }
}

void warehouse_action::loc_callback(std_msgs::Int8 msg)
{
    printf("%d\n", msg.data);
    if(msg.data == 1)
        cout<<"At Position One"<<endl;
}

void warehouse_action::Position_Manager()
{
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group.setStartState(*move_group.getCurrentState());
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;

    while(1)
    {
        command = getchar();

        if(command == 'h')
        {
            ROS_INFO("GO HOME");

            grip.data = false;
            gripper_pub.publish(grip);

            joint_group_positions = home_p;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("DONE");
        }
        if(command == 'w')
        {
            detection_msgs::StringArray prod_list;
            ROS_INFO("GO SCANNING POINT 1");
        
            joint_group_positions = joint_wh_scan1;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            current_pose = move_group.getCurrentPose().pose;
            sleep(1);
            reach = true;
            sleep(1);            

            ROS_INFO("DONE");
            cout<<target_amount<<" target(s)"<<endl;
            
            if(target_amount > 0)
            {
                for(int i=0; i<target_amount; i++)
                {
                    current_pose = move_group.getCurrentPose().pose;
                    tf::TransformListener listener;
                    tf::StampedTransform tf2;
                    string tf_name2 = "/tm_target_pose";
                    listener.waitForTransform("/tm_end_eff", tf_name2 + to_string(i), ros::Time(0), ros::Duration(4.0));
                    listener.lookupTransform("/tm_end_eff", tf_name2 + to_string(i), ros::Time(0), tf2);

                    current_pose.position.x += tf2.getOrigin().getX();
                    current_pose.position.y -= tf2.getOrigin().getZ()-0.095;
                    current_pose.position.z += tf2.getOrigin().getY()-0.09;

                    move_group.setPoseTarget(current_pose);
                    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    move_group.move();

                    current_pose.position.y -= 0.1;

                    move_group.setPoseTarget(current_pose);
                    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    move_group.move();
                    
                    grip.data = true;
                    gripper_pub.publish(grip);
                    sleep(0.5);

                    current_pose.position.y -= -0.1;

                    move_group.setPoseTarget(current_pose);
                    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    move_group.move();
                    
                    target_number++;

                    if(target_number == 1)
                    {
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place1_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 1");
                        joint_group_positions = joint_place1;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place1_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place1_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place1_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 1");
                        joint_group_positions = joint_wh_scan1;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                    if(target_number == 2)
                    {
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place2_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 2");
                        joint_group_positions = joint_place2;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place2_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place2_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place2_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 1");
                        joint_group_positions = joint_wh_scan1;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                    if(target_number == 3)
                    {  
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place3_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 3");
                        joint_group_positions = joint_place3;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place3_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place3_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place3_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 1");
                        joint_group_positions = joint_wh_scan1;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                    if(target_number == 4)
                    { 
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place4_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 1");
                        joint_group_positions = joint_place4;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place4_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place4_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place4_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 1");
                        joint_group_positions = joint_wh_scan1;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                    if(target_number == 5)
                    {
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place5_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 1");
                        joint_group_positions = joint_place5;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place5_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place5_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place5_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 1");
                        joint_group_positions = joint_wh_scan1;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                }
                target_amount = 0;
            }
            
            collect = false;
            reach = false;

            ROS_INFO("GO SCANNING POINT 2");

            joint_group_positions = joint_wh_scan2;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            
            current_pose = move_group.getCurrentPose().pose;
            sleep(1);
            reach = true;
            sleep(1); 

            ROS_INFO("DONE");
            cout<<target_amount<<" target(s)"<<endl;

            if(target_amount > 0)
            {
                for(int i=0; i<target_amount; i++)
                {
                    current_pose = move_group.getCurrentPose().pose;
                    tf::TransformListener listener;
                    tf::StampedTransform tf2;
                    string tf_name2 = "/tm_target_pose";
                    listener.waitForTransform("/tm_end_eff", tf_name2 + to_string(i), ros::Time(0), ros::Duration(4.0));
                    listener.lookupTransform("/tm_end_eff", tf_name2 + to_string(i), ros::Time(0), tf2);

                    current_pose.position.x += tf2.getOrigin().getX();
                    current_pose.position.y -= tf2.getOrigin().getZ()-0.095;
                    current_pose.position.z += tf2.getOrigin().getY()-0.1;

                    move_group.setPoseTarget(current_pose);
                    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    move_group.move();

                    current_pose.position.y -= 0.1;

                    move_group.setPoseTarget(current_pose);
                    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    move_group.move();

                    grip.data = true;
                    gripper_pub.publish(grip);
                    sleep(0.5);

                    current_pose.position.y -= -0.1;

                    move_group.setPoseTarget(current_pose);
                    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    move_group.move();

                    target_number++;

                    if(target_number == 1)
                    {
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place1_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 1");
                        joint_group_positions = joint_place1;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place1_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place1_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place1_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 2");
                        joint_group_positions = joint_wh_scan2;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                    if(target_number == 2)
                    {
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place2_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 2");
                        joint_group_positions = joint_place2;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place2_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place2_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place2_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 2");
                        joint_group_positions = joint_wh_scan2;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                    if(target_number == 3)
                    {
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place3_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 3");
                        joint_group_positions = joint_place3;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place3_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place3_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place3_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 2");
                        joint_group_positions = joint_wh_scan2;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                    if(target_number == 4)
                    { 
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place4_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 1");
                        joint_group_positions = joint_place4;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place4_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place4_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place4_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 2");
                        joint_group_positions = joint_wh_scan2;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                    if(target_number == 5)
                    {
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place5_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 1");
                        joint_group_positions = joint_place5;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place5_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place5_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place5_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 2");
                        joint_group_positions = joint_wh_scan2;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                }
                target_amount = 0;
            }
            
            collect = false;
            reach = false;
            /*
            ROS_INFO("GO SCANNING POINT 3");

            joint_group_positions = joint_wh_scan3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            
            current_pose = move_group.getCurrentPose().pose;
            sleep(1);
            reach = true;
            sleep(1); 

            ROS_INFO("DONE");
            cout<<target_amount<<" target(s)"<<endl;
            if(target_amount > 0)
            {
                for(int i=0; i<target_amount; i++)
                {
                    current_pose = move_group.getCurrentPose().pose;
                    tf::TransformListener listener;
                    tf::StampedTransform tf2;
                    string tf_name2 = "/tm_target_pose";
                    listener.waitForTransform("/tm_end_eff", tf_name2 + to_string(i), ros::Time(0), ros::Duration(4.0));
                    listener.lookupTransform("/tm_end_eff", tf_name2 + to_string(i), ros::Time(0), tf2);

                    current_pose.position.x += tf2.getOrigin().getX();
                    current_pose.position.y -= tf2.getOrigin().getZ()-0.095;
                    current_pose.position.z += tf2.getOrigin().getY()-0.1;

                    move_group.setPoseTarget(current_pose);
                    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    move_group.move();

                    current_pose.position.y -= 0.1;

                    move_group.setPoseTarget(current_pose);
                    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    move_group.move();

                    grip.data = true;
                    gripper_pub.publish(grip);
                    sleep(0.5);

                    current_pose.position.y -= -0.1;

                    move_group.setPoseTarget(current_pose);
                    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    move_group.move();

                    target_number++;

                    if(target_number == 1)
                    {
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place1_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 1");
                        joint_group_positions = joint_place1;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place1_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place1_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place1_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 3");
                        joint_group_positions = joint_wh_scan3;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                    if(target_number == 2)
                    {
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place2_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 2");
                        joint_group_positions = joint_place2;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place2_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place2_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place2_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 3");
                        joint_group_positions = joint_wh_scan3;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                    if(target_number == 3)
                    {  
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place3_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 3");
                        joint_group_positions = joint_place3;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place3_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place3_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place3_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 3");
                        joint_group_positions = joint_wh_scan3;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                    if(target_number == 4)
                    {  
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place4_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 1");
                        joint_group_positions = joint_place4;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place4_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place4_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place4_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 3");
                        joint_group_positions = joint_wh_scan3;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                    if(target_number == 5)
                    {
                        ROS_INFO("GO MIDDLE POINT");

                        joint_group_positions = joint_place5_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO PLACE 1");
                        joint_group_positions = joint_place5;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        
                        grip.data = false;
                        gripper_pub.publish(grip);

                        ROS_INFO("GO MIDDLE POINT");
                        joint_group_positions = joint_place5_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place5_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();
                        joint_group_positions = joint_place5_mid;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("GO SCAN 3");
                        joint_group_positions = joint_wh_scan3;
                        move_group.setJointValueTarget(joint_group_positions);
                        move_group.move();

                        ROS_INFO("DONE");
                    }
                }
                target_amount = 0;
            }

            collect = false;
            reach = false;
            */
            ROS_INFO("GO HOME");

            joint_group_positions = home_p;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
   
            ROS_INFO("DONE");
            target_number = 0;

            prod_pub.publish(prod_list);
            
            replacement_finished.data = 1;
            replace_finish_pub.publish(replacement_finished);

            ROS_INFO("GO SHELF SCAN POINT");

            joint_group_positions = joint_sh_scan;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("DONE");
        }
        if(command == 's')
        {
            ROS_INFO("GO SCANNING POINT 1");
        
            joint_group_positions = joint_wh_scan1;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO SCANNING POINT 2");
        
            joint_group_positions = joint_wh_scan2;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
        }
        if(command == 'r')
        {
            ROS_INFO("GO JOINT PLACE TAG");

            joint_group_positions = joint_place_tag;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            sleep(1);

            tf::TransformListener listener;
            tf::StampedTransform tf_l, tf_bl2coke, tf_bl2soda, tf_bl2lemonade, tf_bl2pinksoda, tf_bl2mineralwater;
            static tf::TransformBroadcaster br;
            tf::Transform tf_b;
            std::string tf_l_name = "/tag_308";
            std::string tf_coke_name = "/target_Coke";
            std::string tf_soda_name = "/target_Soda";
            std::string tf_lemonade_name = "/target_Lemonade";
            std::string tf_pinksoda_name = "/target_PinkSoda";
            std::string tf_mineralwater_name = "/target_MineralWater";

            listener.waitForTransform("/base_link", tf_l_name, ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", tf_l_name, ros::Time(0), tf_l);

            tf_b.setOrigin(tf::Vector3(0.05, -0.11, 0.40));
            tf_b.setRotation(tf::Quaternion(-0.500, 0.500, 0.500, 0.500));
            br.sendTransform(tf::StampedTransform(tf_b, ros::Time::now(), "/tag_308", tf_coke_name));
            listener.waitForTransform("/base_link", tf_coke_name, ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", tf_coke_name, ros::Time(0), tf_bl2coke);

            tf_b.setOrigin(tf::Vector3(-0.08, -0.11, 0.40));
            tf_b.setRotation(tf::Quaternion(-0.500, 0.500, 0.500, 0.500));
            br.sendTransform(tf::StampedTransform(tf_b, ros::Time::now(), "/tag_308", tf_mineralwater_name));
            listener.waitForTransform("/base_link", tf_mineralwater_name, ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", tf_mineralwater_name, ros::Time(0), tf_bl2mineralwater);

            tf_b.setOrigin(tf::Vector3(0.18, -0.11, 0.40));
            tf_b.setRotation(tf::Quaternion(-0.500, 0.500, 0.500, 0.500));
            br.sendTransform(tf::StampedTransform(tf_b, ros::Time::now(), "/tag_308", tf_pinksoda_name));
            listener.waitForTransform("/base_link", tf_pinksoda_name, ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", tf_pinksoda_name, ros::Time(0), tf_bl2pinksoda);

            tf_b.setOrigin(tf::Vector3(-0.21, -0.11, 0.40));
            tf_b.setRotation(tf::Quaternion(-0.500, 0.500, 0.500, 0.500));
            br.sendTransform(tf::StampedTransform(tf_b, ros::Time::now(), "/tag_308", tf_lemonade_name));
            listener.waitForTransform("/base_link", tf_lemonade_name, ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", tf_lemonade_name, ros::Time(0), tf_bl2lemonade);

            tf_b.setOrigin(tf::Vector3(-0.34, -0.11, 0.40));
            tf_b.setRotation(tf::Quaternion(-0.500, 0.500, 0.500, 0.500));
            br.sendTransform(tf::StampedTransform(tf_b, ros::Time::now(), "/tag_308", tf_soda_name));
            listener.waitForTransform("/base_link", tf_soda_name, ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", tf_soda_name, ros::Time(0), tf_bl2soda);

            tf::StampedTransform product_at_p1 = tf_bl2pinksoda;
            tf::StampedTransform product_at_p2 = tf_bl2mineralwater;
            tf::StampedTransform product_at_p3 = tf_bl2soda;

            //START REPLACEMENT
            ROS_INFO("GO JOINT PLACE 3");
        
            joint_group_positions = joint_place3_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            joint_group_positions = joint_place3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            grip.data = true;
            gripper_pub.publish(grip);

            joint_group_positions = joint_place3_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO JOINT PLACE TARGET");

            joint_group_positions = joint_place_tag;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            tag_pose = move_group.getCurrentPose().pose;
            tag_pose.position.x = product_at_p3.getOrigin().getX();
            tag_pose.position.y = product_at_p3.getOrigin().getY();
            tag_pose.position.z = product_at_p3.getOrigin().getZ();
            tag_pose.position.y += 0.10;
            tag_pose.position.z += 0.10;
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            tag_pose.position.y -= 0.10;
            tag_pose.position.z -= 0.10;
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();

            grip.data = false;
            gripper_pub.publish(grip);

            tag_pose.position.y += 0.15;
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();

            joint_group_positions = joint_place_tag;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO JOINT PLACE 2");
        
            joint_group_positions = joint_place2_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            joint_group_positions = joint_place2;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            grip.data = true;
            gripper_pub.publish(grip);

            joint_group_positions = joint_place2_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO JOINT PLACE TARGET");

            joint_group_positions = joint_place_tag;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            tag_pose = move_group.getCurrentPose().pose;
            tag_pose.position.x = product_at_p2.getOrigin().getX();
            tag_pose.position.y = product_at_p2.getOrigin().getY();
            tag_pose.position.z = product_at_p2.getOrigin().getZ();           
            tag_pose.position.y += 0.10;
            tag_pose.position.z += 0.10;
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            tag_pose.position.y -= 0.10;
            tag_pose.position.z -= 0.10;
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();

            grip.data = false;
            gripper_pub.publish(grip);

            tag_pose.position.y += 0.15;
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();

            joint_group_positions = joint_place_tag;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO JOINT PLACE 1");
        
            joint_group_positions = joint_place1_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            joint_group_positions = joint_place1;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            grip.data = true;
            gripper_pub.publish(grip);

            joint_group_positions = joint_place1_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO JOINT PLACE TARGET");

            joint_group_positions = joint_place_tag;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            tag_pose = move_group.getCurrentPose().pose;
            tag_pose.position.x = product_at_p1.getOrigin().getX();
            tag_pose.position.y = product_at_p1.getOrigin().getY();
            tag_pose.position.z = product_at_p1.getOrigin().getZ();           
            tag_pose.position.y += 0.10;
            tag_pose.position.z += 0.10;
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            tag_pose.position.y -= 0.10;
            tag_pose.position.z -= 0.10;
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();

            grip.data = false;
            gripper_pub.publish(grip);

            tag_pose.position.y += 0.15;
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();
            move_group.setPoseTarget(tag_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();

            joint_group_positions = joint_place_tag;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("DONE");

            replacement_finished.data = 2;
            replace_finish_pub.publish(replacement_finished);
        }
        if(command == 'm')
        {
            ROS_INFO("GO JOINT PLACE 1 MID");
        
            joint_group_positions = joint_place1_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

        }
        if(command == 'p')
        {
            ROS_INFO("GO JOINT PLACE 1");
        
            joint_group_positions = joint_place1;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

        }
        if(command == 'q')
        {
            joint_group_positions = home_p_2;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("QUIT");
            break;
        }
    }
}

warehouse_action *wa;

void sigHandler(int signum) 
{
    delete wa;

    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "warehouse_action");
    ros::NodeHandle nh;
    signal(SIGINT, sigHandler);
    wa = new warehouse_action(nh);
    ros::spin();

    return 0;
}