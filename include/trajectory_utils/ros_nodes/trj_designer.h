#ifndef _TRJ_DESIGNER_H_
#define _TRJ_DESIGNER_H_

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_listener.h>

namespace trj_designer{


class Marker6DoFs{
public:
    Marker6DoFs(const std::string& base_link, const std::string& distal_link,
                interactive_markers::InteractiveMarkerServer& server):
        _server(server),
        initial_pose(KDL::Frame::Identity()),
        _base_link(base_link), _distal_link(distal_link)
    {
        tf::TransformListener listener;
        tf::StampedTransform transform;
        for(unsigned int i = 0; i < 10; ++i){
        try{
            ros::Time now = ros::Time::now();
            listener.waitForTransform(base_link, distal_link,now,ros::Duration(1.0));

            listener.lookupTransform(base_link, distal_link,
                ros::Time(), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }}
        KDL::Frame transform_KDL; transform_KDL = transform_KDL.Identity();
        transform_KDL.p.x(transform.getOrigin().x());
        transform_KDL.p.y(transform.getOrigin().y());
        transform_KDL.p.z(transform.getOrigin().z());

        transform_KDL.M = transform_KDL.M.Quaternion(
                    transform.getRotation().getX(), transform.getRotation().getY(),
                    transform.getRotation().getZ(), transform.getRotation().getW()
                    );

        initial_pose = transform_KDL*initial_pose;


        MakeMarker(distal_link, false,
                   visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
                   true);

        ROS_WARN("%s initial pose wrt %s is: ", int_marker.name.c_str(),
                 base_link.c_str());
        double qx, qy, qz, qw;
        initial_pose.M.GetQuaternion(qx, qy, qz, qw);
        ROS_WARN("      pose:   [%f, %f, %f]", initial_pose.p.x(), initial_pose.p.y(), initial_pose.p.z());
        ROS_WARN("      quat:   [%f, %f, %f, %f]", qx, qy, qz, qw);

    }



    void MakeMarker( const std::string& distal_link,
                bool fixed,
                unsigned int interaction_mode,
                bool show_6dof)
    {
      int_marker.header.frame_id = distal_link;
      int_marker.scale = 0.2;

      int_marker.name = distal_link;
      int_marker.description = "";

      // insert a box
      makeBoxControl(int_marker);
      int_marker.controls[0].interaction_mode = interaction_mode;

      if ( fixed )
      {
        //int_marker.name += "_fixed";
        int_marker.description += "";
        control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
      }

      if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
      {
          std::string mode_text;
          if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
          if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
          if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
          //int_marker.name += "_" + mode_text;
          int_marker.description = "";
      }

      if(show_6dof)
      {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
      }

      _server.insert(int_marker,
                    boost::bind(boost::mem_fn(&Marker6DoFs::MarkerFeedback),
                                this, _1));
    }

    visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg )
    {
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = msg.scale * 0.45;
      marker.scale.y = msg.scale * 0.45;
      marker.scale.z = msg.scale * 0.45;
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      marker.color.a = 1.0;

      return marker;
    }

    visualization_msgs::InteractiveMarkerControl& makeBoxControl(
            visualization_msgs::InteractiveMarker &msg )
    {
      control2.always_visible = true;
      control2.markers.push_back( makeBox(msg) );
      msg.controls.push_back( control2 );

      return msg.controls.back();
    }

    void MarkerFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        tf::poseMsgToKDL(feedback->pose, actual_pose);
        actual_pose = initial_pose*actual_pose;

        ROS_WARN("%s actual_pose pose wrt %s is: ", int_marker.name.c_str(),
                 _base_link.c_str());
        double qx, qy, qz, qw;
        actual_pose.M.GetQuaternion(qx, qy, qz, qw);
        ROS_WARN("      pose:   [%f, %f, %f]", actual_pose.p.x(), actual_pose.p.y(), actual_pose.p.z());
        ROS_WARN("      quat:   [%f, %f, %f, %f]", qx, qy, qz, qw);
    }

    KDL::Frame getActualPose(){return actual_pose;}
    KDL::Frame getInitialPose(){return initial_pose;}
    std::string getBaseLink(){return _base_link;}
    std::string getDistalLink(){return _distal_link;}

    visualization_msgs::InteractiveMarker int_marker;
private:
    visualization_msgs::InteractiveMarkerControl control;
    visualization_msgs::InteractiveMarkerControl control2;
    visualization_msgs::Marker marker;
    interactive_markers::InteractiveMarkerServer& _server;
    KDL::Frame initial_pose;
    KDL::Frame actual_pose;
    std::string _base_link;
    std::string _distal_link;





};

}

#endif
