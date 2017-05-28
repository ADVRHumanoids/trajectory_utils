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
  double dT;
  nh.getParam("dT", dT);
  ROS_INFO("Trajectory: %s wrt %s at %f [sec]", distal_link.c_str(), base_link.c_str(), dT);



  interactive_markers::InteractiveMarkerServer server(distal_link+"_trajectory_marker_server");
  trj_designer::Marker6DoFs marker(base_link, distal_link, server, dT);

  server.applyChanges();

  ROS_INFO("Running Trajectory Designer...");
  while(ros::ok()){
      marker.publishTrjs();
      ros::spinOnce();}
}
