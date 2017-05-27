#include <ros/ros.h>
#include <trajectory_utils/ros_nodes/trj_designer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_designer");

  ros::NodeHandle nh("~");

  //Getting parameters from param server
  std::string base_link;
  nh.getParam("base_link", base_link);
  std::string distal_link;
  nh.getParam("distal_link", distal_link);
  ROS_INFO("Controlling %s wrt %s", distal_link.c_str(), base_link.c_str());


  ROS_INFO("Creating Marker...");
  ROS_INFO("Adding Marker to MarkerServer...");
  interactive_markers::InteractiveMarkerServer server(distal_link+"_trajectory_marker_server");
  trj_designer::Marker6DoFs marker(base_link, distal_link, server);

  server.applyChanges();

  ROS_INFO("Running Trajectory Designer...");
  ros::spin();
}
