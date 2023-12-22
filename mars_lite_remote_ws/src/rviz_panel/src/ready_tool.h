#ifndef READY_TOOL_H
#define READY_TOOL_H

#ifndef Q_MOC_RUN 
    #include <QObject>
    #include <ros/ros.h>
    #include <rviz/tool.h>
    #include <std_msgs/Empty.h>
    #include <move_base_msgs/MoveBaseAction.h>
    #include <actionlib/client/simple_action_client.h>
#endif


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace rviz
{
  class BoolProperty;
}

namespace rviz_panel
{

class ReadyTool: public rviz::Tool
{
Q_OBJECT
public:
  ReadyTool();
  virtual ~ReadyTool() {}

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate() {}
//  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
protected:
  void publish_ready();

private Q_SLOTS:
 // void updateTopic();
  void updateAutoDeactivate();
  

private:
  ros::NodeHandle nh_;
 // ros::Publisher pub_;

  MoveBaseClient *mob_plat_client;

 // rviz::StringProperty* topic_property_;
  rviz::BoolProperty* auto_deactivate_property_;

};
}

#endif