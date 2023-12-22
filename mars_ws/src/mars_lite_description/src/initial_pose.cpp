#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char** argv) {
  // 初始化ROS節點
  ros::init(argc, argv, "initial_pose_publisher");
  ros::NodeHandle nh;

  // 讀取參數
  double robot_init_x, robot_init_y, robot_init_rot;
  ros::param::param<double>("~robot_init_x", robot_init_x, 0.0);
  ros::param::param<double>("~robot_init_y", robot_init_y, 0.0);
  ros::param::param<double>("~robot_init_rot", robot_init_rot, 0.0);

  // 創建一個ROS消息，設定初始位姿
  geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
  initial_pose_msg.header.seq = 0;
  initial_pose_msg.header.stamp = ros::Time(0);
  initial_pose_msg.header.frame_id = "/map";
  initial_pose_msg.pose.pose.position.x = robot_init_x;
  initial_pose_msg.pose.pose.position.y = robot_init_y;
  initial_pose_msg.pose.pose.position.z = 0.0;
  initial_pose_msg.pose.pose.orientation.x = 0.0;
  initial_pose_msg.pose.pose.orientation.y = 0.0;
  initial_pose_msg.pose.pose.orientation.z = sin(robot_init_rot / 2.0);
  initial_pose_msg.pose.pose.orientation.w = cos(robot_init_rot / 2.0);

  // 發布一次初始位姿消息
  ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
  initial_pose_pub.publish(initial_pose_msg);

  ROS_INFO("Published initial pose: x=%.2f, y=%.2f, theta=%.2f radians", robot_init_x, robot_init_y, robot_init_rot);

  // 休眠一段時間，確保消息發佈完成
  ros::Duration(1.0).sleep();

  return 0;
}