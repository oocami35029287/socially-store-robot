source docker_run cuda10
source docker_run same
#////////gym/////////////
cmd1	roslaunch pedsim_simulator gym_crowd_environment.launch scene_file:=crossing_corridor.xml
cmd2	roslaunch pedsim_gazebo_plugin crossing_corridor.launch
cmd3	roslaunch mars_lite_description spawn_mars.launch
cmd4	rosservice call /pedsim_simulator/unpause_simulation "{}"
cmd5	roslaunch scan yolodetect.launch
cmd6	roslaunch path_finding astar_path_finding_with_social_proxemics.launch
cmd7	roslaunch path_tracking path_tracking_autonomous.launch
cmd8	roslaunch turtlebot3_navigation globalmap.launch map_file:=crossing_corridor.yaml

#/////////////testing//////////////////////////////////

roslaunch mars_lite_moveit_config mars_lite_moveit_planning_execution_gz.launch
#/////////////optional//////////////////////////////////
cmd3 	roslaunch mars_lite_description spawn_mars.launch realsense_enabled:=false
cmd5	roslaunch scan mot2d_sim.launch


#///////shopping mall/////
roslaunch pedsim_simulator shopping_mall.launch
roslaunch pedsim_gazebo_plugin shopping_mall.launch	
roslaunch mars_lite_description spawn_mars.launch
rosservice call /pedsim_simulator/unpause_simulation "{}"
roslaunch turtlebot3_navigation globalmap.launch map_file:=map.yaml
#////////support/////////
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
rosrun r_reconfigure rqt_reconfigure
rqt_plot

meld ./mars_ws/src/mars_lite_description ./mars_lite_simulation_ws/src/mars_lite_description


////////slam///////////////
gmapping	roslaunch gmapping gmapping_rviz.launch
save		rosrun map_server map_saver -f ~/socially-store-robot/mars_ws/src/navigation/turtlebot3_navigation/maps/crossing_corrider

