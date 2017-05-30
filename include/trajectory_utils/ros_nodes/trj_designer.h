#ifndef _TRJ_DESIGNER_H_
#define _TRJ_DESIGNER_H_

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_listener.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/utils/ros_trj_publisher.h>
#include <urdf/model.h>
#include <geometry_msgs/Pose.h>

#define SECS 5

namespace trj_designer{

struct segment_trj{
    std::string type;
    double T;
    KDL::Frame start;
    KDL::Frame end;
};

class Marker6DoFs{
public:
    Marker6DoFs(const std::string& base_link, const std::string& distal_link,
                interactive_markers::InteractiveMarkerServer& server, const double dT,
                const urdf::Model& robot_urdf,
                ros::NodeHandle& n):
        _server(server),
        _base_link(base_link), _distal_link(distal_link),
        _dT(dT), _urdf(robot_urdf)
    {

        start_pose = getRobotActualPose();
        initial_pose = start_pose;

        MakeMarker(distal_link, base_link, false,
                   visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
                   true);


        MakeMenu();

        trj_pub.reset(new trajectory_utils::trajectory_publisher(_distal_link+"_trj_viz"));

        _goal_sub = n.subscribe(_distal_link+"_goal", 1000, &Marker6DoFs::GoalCallBack, this);
    }



    void GoalCallBack(const geometry_msgs::Pose::ConstPtr& msg)
    {
        ROS_WARN("Received goal pose for %s", _distal_link.c_str());

        KDL::Frame goal; goal.Identity();
        goal.p.x(msg->position.x);
        goal.p.y(msg->position.y);
        goal.p.z(msg->position.z);
        goal.M = goal.M.Quaternion(msg->orientation.x, msg->orientation.y,
                                   msg->orientation.z, msg->orientation.w);

        _goal_pose.reset(new KDL::Frame(goal.M, goal.p));

        goal_int_marker.header.frame_id = _base_link;
        goal_int_marker.scale = 0.2;

        goal_int_marker.pose.position.x = msg->position.x;
        goal_int_marker.pose.position.y = msg->position.y;
        goal_int_marker.pose.position.z = msg->position.z;
        goal_int_marker.pose.orientation.x = msg->orientation.x;
        goal_int_marker.pose.orientation.y = msg->orientation.y;
        goal_int_marker.pose.orientation.z = msg->orientation.z;
        goal_int_marker.pose.orientation.w = msg->orientation.w;

        goal_int_marker.name = _distal_link+"_goal";
        goal_int_marker.description = "";

        makeSTLControl(goal_int_marker);

        goal_int_marker.controls[0].interaction_mode =
                visualization_msgs::InteractiveMarkerControl::NONE;

        _server.erase(_distal_link+"_goal");

        _server.insert(goal_int_marker);

        _server.applyChanges();
    }

    void MakeMenu()
    {
        //#1 Min Jerk
        min_jerk_entry = menu_handler.insert("MinJerk");
        T_entry = menu_handler.insert(min_jerk_entry, "T [sec]");
        for ( int i=0; i<SECS; i++ )
        {
            std::ostringstream s;
            s <<i+1;
            T_last = menu_handler.insert( T_entry, s.str(),
                boost::bind(boost::mem_fn(&Marker6DoFs::MinJerkMenuCallBack),
                            this, _1));
            menu_handler.setCheckState( T_last, interactive_markers::MenuHandler::UNCHECKED );
        }

        //#2ResetMarker
        reset_marker_entry = menu_handler.insert("Reset Marker Pose",
            boost::bind(boost::mem_fn(&Marker6DoFs::ResetMarkerCb), this, _1));
        menu_handler.setCheckState(reset_marker_entry, interactive_markers::MenuHandler::UNCHECKED);


        //#3 ResetTrj
        reset_trj_entry = menu_handler.insert("Reset Trajectory");
        reset_last_entry = menu_handler.insert(reset_trj_entry, "Last",
            boost::bind(boost::mem_fn(&Marker6DoFs::ResetLastTrjCb), this, _1));
        menu_handler.setCheckState(reset_last_entry, interactive_markers::MenuHandler::UNCHECKED);
        reset_all_entry = menu_handler.insert(reset_trj_entry, "All",
            boost::bind(boost::mem_fn(&Marker6DoFs::ResetTrjCb), this, _1));
        menu_handler.setCheckState(reset_all_entry, interactive_markers::MenuHandler::UNCHECKED);

        //#4 Goal
        goal_entry = menu_handler.insert("Goal");
        remove_goal_entry = menu_handler.insert(goal_entry, "Remove Goal",
            boost::bind(boost::mem_fn(&Marker6DoFs::RemoveGoalCb), this, _1));
        menu_handler.setCheckState(remove_goal_entry, interactive_markers::MenuHandler::UNCHECKED);
        move_to_goal_entry = menu_handler.insert(goal_entry, "Move to Goal in T [sec]");
        for ( int i=0; i<SECS; i++ )
        {
            std::ostringstream s;
            s <<i+1;
            goal_T_last = menu_handler.insert( move_to_goal_entry, s.str(),
                boost::bind(boost::mem_fn(&Marker6DoFs::MinJerkToGoalMenuCallBack),
                            this, _1));
            menu_handler.setCheckState( goal_T_last, interactive_markers::MenuHandler::UNCHECKED );
        }


        menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        menu_control.always_visible = true;

        int_marker.controls.push_back(menu_control);

        menu_handler.apply(_server, int_marker.name);
    }



    void MakeMarker( const std::string& distal_link,
                const std::string& base_link,
                bool fixed,
                unsigned int interaction_mode,
                bool show_6dof)
    {
      int_marker.header.frame_id = base_link;
      int_marker.scale = 0.2;

      int_marker.name = distal_link;
      int_marker.description = "";

      // insert STL
      makeSTLControl(int_marker);

      // insert a box
      //makeBoxControl(int_marker);

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

      int_marker.pose.position.x = start_pose.p.x();
      int_marker.pose.position.y = start_pose.p.y();
      int_marker.pose.position.z = start_pose.p.z();
      double qx,qy,qz,qw; start_pose.M.GetQuaternion(qx,qy,qz,qw);
      int_marker.pose.orientation.x = qx;
      int_marker.pose.orientation.y = qy;
      int_marker.pose.orientation.z = qz;
      int_marker.pose.orientation.w = qw;

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

    visualization_msgs::Marker makeSTL( visualization_msgs::InteractiveMarker &msg )
    {


        boost::shared_ptr<const urdf::Link> link = _urdf.getLink(_distal_link);

        if(link->visual->geometry->type == urdf::Geometry::MESH)
        {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;

            boost::shared_ptr<urdf::Mesh> mesh =
                    boost::static_pointer_cast<urdf::Mesh>(link->visual->geometry);

            marker.mesh_resource = mesh->filename;

            marker.pose.position.x = link->visual->origin.position.x;
            marker.pose.position.y = link->visual->origin.position.y;
            marker.pose.position.z = link->visual->origin.position.z;
            marker.pose.orientation.x = link->visual->origin.rotation.x;
            marker.pose.orientation.y = link->visual->origin.rotation.y;
            marker.pose.orientation.z = link->visual->origin.rotation.z;
            marker.pose.orientation.w = link->visual->origin.rotation.w;

            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;

            marker.scale.x = mesh->scale.x;
            marker.scale.y = mesh->scale.y;
            marker.scale.z = mesh->scale.z;
        }
        else
            makeBox(msg);

        marker.color.a = .9;
        return marker;
    }

    visualization_msgs::InteractiveMarkerControl& makeSTLControl(
            visualization_msgs::InteractiveMarker &msg )
    {
      control2.always_visible = true;
      control2.markers.push_back( makeSTL(msg) );
      msg.controls.push_back( control2 );

      return msg.controls.back();
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
        //In base_link
        tf::poseMsgToKDL(feedback->pose, actual_pose);
    }

    void MinJerkToGoalMenuCallBack(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        if(_goal_pose)
        {
            std::string trj_type = "MIN_JERK";
            double T = feedback->menu_entry_id-2;

//            ROS_WARN("REQUEST:");
//            ROS_WARN("  Trj Type: %s", trj_type.c_str());
//            ROS_WARN("  base_link: %s", _base_link.c_str());
//            ROS_WARN("  distal_link: %s", _distal_link.c_str());
//            ROS_WARN("  Time [sec]: %f", T);
//            double qx, qy, qz, qw;
//            initial_pose.M.GetQuaternion(qx, qy, qz, qw);
//            ROS_WARN("  START: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
//                     initial_pose.p.x(), initial_pose.p.y(), initial_pose.p.z(),
//                     qx, qy, qz, qw);
//            actual_pose.M.GetQuaternion(qx, qy, qz, qw);
//            ROS_WARN("  END: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
//                     actual_pose.p.x(), actual_pose.p.y(), actual_pose.p.z(),
//                     qx, qy, qz, qw);

            segment_trj seg;
            seg.type = trj_type;
            seg.start = start_pose;
            seg.end = *(_goal_pose.get());
            seg.T = T;

            segments_trj.push_back(seg);

            start_pose = *(_goal_pose.get());

            ResetMarkerPose();
        }
    }

    void MinJerkMenuCallBack(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        std::string trj_type = "MIN_JERK";
        double T = feedback->menu_entry_id-2;

//        ROS_WARN("REQUEST:");
//        ROS_WARN("  Trj Type: %s", trj_type.c_str());
//        ROS_WARN("  base_link: %s", _base_link.c_str());
//        ROS_WARN("  distal_link: %s", _distal_link.c_str());
//        ROS_WARN("  Time [sec]: %f", T);
//        double qx, qy, qz, qw;
//        initial_pose.M.GetQuaternion(qx, qy, qz, qw);
//        ROS_WARN("  START: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
//                 initial_pose.p.x(), initial_pose.p.y(), initial_pose.p.z(),
//                 qx, qy, qz, qw);
//        actual_pose.M.GetQuaternion(qx, qy, qz, qw);
//        ROS_WARN("  END: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
//                 actual_pose.p.x(), actual_pose.p.y(), actual_pose.p.z(),
//                 qx, qy, qz, qw);

        segment_trj seg;
        seg.type = trj_type;
        seg.start = start_pose;


        seg.end = actual_pose;


        //seg.end = actual_pose;
        seg.T = T;

        segments_trj.push_back(seg);

        //start_pose = actual_pose;
        start_pose = actual_pose;

    }

    KDL::Frame getRobotActualPose()
    {
        for(unsigned int i = 0; i < 10; ++i){
        try{
            ros::Time now = ros::Time::now();
            _listener.waitForTransform(_base_link, _distal_link,now,ros::Duration(1.0));

            _listener.lookupTransform(_base_link, _distal_link,
                ros::Time(), _transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }}
        KDL::Frame transform_KDL; transform_KDL = transform_KDL.Identity();
        transform_KDL.p.x(_transform.getOrigin().x());
        transform_KDL.p.y(_transform.getOrigin().y());
        transform_KDL.p.z(_transform.getOrigin().z());

        transform_KDL.M = transform_KDL.M.Quaternion(
                    _transform.getRotation().getX(), _transform.getRotation().getY(),
                    _transform.getRotation().getZ(), _transform.getRotation().getW()
                    );

        return transform_KDL;
    }



    void ResetMarkerPose()
    {
        geometry_msgs::Pose last_pose;
        KDL::Frame last_pose_KDL;


        if(segments_trj.size() > 0)
        {
            segment_trj trj = segments_trj[segments_trj.size()-1];

            last_pose_KDL = trj.end;
            //last_pose_KDL = initial_pose.Inverse()*trj.end;
        }
        else
            last_pose_KDL = initial_pose;

        tf::poseKDLToMsg(last_pose_KDL, last_pose);

        _server.setPose(int_marker.name, last_pose);
        _server.applyChanges();
    }

    void RemoveGoalCb(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        _goal_pose.reset();

        _server.erase(_distal_link+"_goal");

        _server.applyChanges();
    }

    void ResetMarkerCb(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        ResetMarkerPose();
    }

    void ResetLastTrjCb(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        if(segments_trj.size() > 0){
            segments_trj.pop_back();
            start_pose = segments_trj[segments_trj.size()-1].end;

            trj_pub->deleteAllMarkersAndTrj();
        }
        ResetMarkerPose();
    }

    void ResetTrjCb(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        if(segments_trj.size()){
            segments_trj.clear();

            start_pose = initial_pose;

            trj_pub->deleteAllMarkersAndTrj();
        }

        ResetMarkerPose();
    }

    void publishTrjs()
    {
        if(segments_trj.size() > 0)
        {
            trj_gen.reset(new trajectory_utils::trajectory_generator(_dT, _base_link, _distal_link));
            for(unsigned int i = 0; i < segments_trj.size(); ++i)
            {
                segment_trj trj = segments_trj[i];
                if(trj.type == "MIN_JERK"){
                    trj_gen->addMinJerkTrj(trj.start, trj.end, trj.T);
                }
            }


            //trj_pub->deleteAllMarkersAndTrj();
            trj_pub->setTrj(trj_gen->getTrajectory(), _base_link, _distal_link);

            trj_pub->publish();
        }
    }


    std::string getBaseLink(){return _base_link;}
    std::string getDistalLink(){return _distal_link;}

    visualization_msgs::InteractiveMarker int_marker;
    interactive_markers::MenuHandler menu_handler;
private:
    visualization_msgs::InteractiveMarkerControl control;
    visualization_msgs::InteractiveMarkerControl control2;
    visualization_msgs::Marker marker;
    interactive_markers::InteractiveMarkerServer& _server;
    KDL::Frame initial_pose;
    KDL::Frame actual_pose;
    KDL::Frame start_pose;
    std::string _base_link;
    std::string _distal_link;

    interactive_markers::MenuHandler::EntryHandle min_jerk_entry;
    interactive_markers::MenuHandler::EntryHandle T_entry;
    interactive_markers::MenuHandler::EntryHandle T_last;
    visualization_msgs::InteractiveMarkerControl  menu_control;

    std::vector<segment_trj> segments_trj;
    boost::shared_ptr<trajectory_utils::trajectory_generator> trj_gen;
    double _dT;
    boost::shared_ptr<trajectory_utils::trajectory_publisher> trj_pub;

    interactive_markers::MenuHandler::EntryHandle reset_marker_entry;

    interactive_markers::MenuHandler::EntryHandle reset_trj_entry;
    interactive_markers::MenuHandler::EntryHandle reset_last_entry;
    interactive_markers::MenuHandler::EntryHandle reset_all_entry;

    urdf::Model _urdf;

    ros::Subscriber _goal_sub;
    boost::shared_ptr<KDL::Frame> _goal_pose;
    visualization_msgs::InteractiveMarker goal_int_marker;
    visualization_msgs::Marker goal_marker;
    interactive_markers::MenuHandler::EntryHandle goal_entry;
    interactive_markers::MenuHandler::EntryHandle remove_goal_entry;
    interactive_markers::MenuHandler::EntryHandle move_to_goal_entry;
    interactive_markers::MenuHandler::EntryHandle goal_T_last;

    tf::TransformListener _listener;
    tf::StampedTransform _transform;

};

}

#endif
