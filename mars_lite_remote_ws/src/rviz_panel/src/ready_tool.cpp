#include <OgreVector3.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/selection/selection_manager.h>

#include "ready_tool.h"


namespace rviz_panel
{

ReadyTool::ReadyTool() :Tool()
{
  shortcut_key_ = '4';

 /*  topic_property_ = new rviz::StringProperty( "Topic", "/path_ready",
                                        "The topic on which to publish multiple pose to follow.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this ); */
  auto_deactivate_property_ = new rviz::BoolProperty( "Single click", true,
                                                "Switch away from this tool after one click.",
                                        getPropertyContainer(), SLOT( updateAutoDeactivate() ), this );  
  
  mob_plat_client = new MoveBaseClient("/move_base", true);

    //wait for action server to come up
    while (!mob_plat_client->isServerConnected())
    {
      ROS_INFO("Waiting for move base action lib server...");
      usleep(3000000); //Wait 3s
    }
    ROS_INFO("Connected to move base action server...");
                                      
}

void ReadyTool::onInitialize()
{
  rviz::Tool::onInitialize();
  setName( "Cancel Goal" );
 // updateTopic();
}

void ReadyTool::activate()
{
    publish_ready();
}


/* void ReadyTool::updateTopic()
{
  pub_ = nh_.advertise<std_msgs::Empty>( topic_property_->getStdString(), 1 );
}
 */
void ReadyTool::publish_ready()
{
   // std::cout << "Navigation enabled. Cancelling current goals." << std::endl;
    setStatus( "Cancelling current goals." );
    mob_plat_client->cancelAllGoals();
   // auto_deactivate_property_->setBool;
}

void ReadyTool::updateAutoDeactivate()
{
    
}


}// end namespace rviz_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_panel::ReadyTool,rviz::Tool )