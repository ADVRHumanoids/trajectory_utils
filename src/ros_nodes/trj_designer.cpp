#include <ros/ros.h>
#include <trajectory_utils/ros_nodes/trj_designer.h>

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_designer");

  ros::NodeHandle nh;

  //Getting parameters from param server
  XmlRpc::XmlRpcValue base_link_list;
  nh.getParam("base_link_list", base_link_list);
  XmlRpc::XmlRpcValue distal_link_list;
  nh.getParam("distal_link_list", distal_link_list);

  ROS_ASSERT(base_link_list.size() == distal_link_list.size());
  ROS_ASSERT(base_link_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(distal_link_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  std::vector<std::string> base_links;
  std::vector<std::string> distal_links;
  for(unsigned int i = 0; i < base_link_list.size(); ++i)
  {
      ROS_ASSERT(base_link_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      std::string base_link = static_cast<std::string>(base_link_list[i]);
      base_links.push_back(base_link);

      ROS_ASSERT(distal_link_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      std::string distal_link = static_cast<std::string>(distal_link_list[i]);
      distal_links.push_back(distal_link);

      ROS_INFO("Controlling %s wrt %s", distal_link.c_str(), base_link.c_str());
  }

  ROS_INFO("Creating Markers...");
  std::vector<trj_designer::Marker6DoFs> markers;
  for(unsigned int i = 0; i < distal_links.size(); ++i)
  {
      trj_designer::Marker6DoFs marker(distal_links[i], false,
                                       visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
                                       true);
      markers.push_back(marker);
  }

  ROS_INFO("Adding Markers to MarkerServer...");
  interactive_markers::InteractiveMarkerServer server("trajectory_marker_server");
  interactive_markers::MenuHandler menu_handler;
  for(unsigned int i = 0; i < markers.size(); ++i){
      server.insert(markers[i].int_marker, &processFeedback);
      if (markers[i].interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
          menu_handler.apply( server, markers[i].int_marker.name );
  }

  server.applyChanges();

  ROS_INFO("Running Trajectory Designer...");
  ros::spin();
}
