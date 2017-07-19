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
#include <std_srvs/Empty.h>
#include <trajectory_utils/CartesianTrj.h>
#include <trajectory_utils/segmentTrj.h>

#define SECS 5

namespace trj_designer{

enum trj_type: int {
    MIN_JERK = 0,
    SEMI_CIRCLE = 1
};

struct segment_trj{
    //Minimal set for MinJerk
    int type;
    double T;
    KDL::Frame start;
    KDL::Frame end;

    //These extra are for SemiCircular
    KDL::Rotation end_rot;
    double angle_rot;
    KDL::Vector circle_center;
    KDL::Vector plane_normal;
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
        offset_menu_entry_goal_min_jerk = 0;
        offset_menu_entry_min_jerk = 0;
        offset_menu_entry_semicircular_XY = 0;
        offset_menu_entry_semicircular_XZ = 0;
        offset_menu_entry_semicircular_YZ = 0;

        start_pose = getRobotActualPose();
        initial_pose = start_pose;
        actual_pose = start_pose;

        printPose("initial_pose_"+_distal_link, initial_pose);

        MakeMarker(distal_link, base_link, false,
                   visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
                   true);


        MakeMenu();

        trj_pub.reset(new trajectory_utils::trajectory_publisher(_distal_link+"_trj_viz"));

        _goal_sub = n.subscribe("/"+_distal_link+"_goal", 1000, &Marker6DoFs::GoalCallBack, this);
    }

    static void printPose(const std::string& frame_name, const KDL::Frame& pose)
    {
        double qx,qy,qz,qw;
        pose.M.GetQuaternion(qx, qy, qz, qw);
        ROS_INFO("%s pose:", frame_name.c_str());
        ROS_INFO(" position [%f, %f, %f]", pose.p.x(), pose.p.y(), pose.p.z());
        ROS_INFO(" orientation [%f, %f, %f, %f]", qx, qy, qz, qw);
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
        int insert_counter = 0;

        //#1 Min Jerk
        min_jerk_entry = menu_handler.insert("MinJerk");
        insert_counter++;
        T_entry = menu_handler.insert(min_jerk_entry, "T [sec]");
        insert_counter++;
        offset_menu_entry_min_jerk = insert_counter;
        for ( int i=0; i<SECS; i++ )
        {
            std::ostringstream s;
            s <<i+1;
            T_last = menu_handler.insert( T_entry, s.str(),
                boost::bind(boost::mem_fn(&Marker6DoFs::MinJerkMenuCallBack),
                            this, _1));
            insert_counter++;
            menu_handler.setCheckState( T_last, interactive_markers::MenuHandler::UNCHECKED );
        }

        //#2 Arc
        circual_entry = menu_handler.insert("SemiCircular");
        insert_counter++;
        reverse_entry = menu_handler.insert(circual_entry, "Reverse",
            boost::bind(boost::mem_fn(&Marker6DoFs::ReverseMenuCallBack),this, _1));
        insert_counter++;
        reverse = 1;
        menu_handler.setCheckState(reverse_entry, interactive_markers::MenuHandler::UNCHECKED);
        circularXY_entry = menu_handler.insert(circual_entry, "XY in T [sec]");
        insert_counter++;
        offset_menu_entry_semicircular_XY = insert_counter++;
        for ( int i=0; i<SECS; i++ )
        {
            std::ostringstream s;
            s <<i+1;
            T_XY_entry = menu_handler.insert( circularXY_entry, s.str(),
                boost::bind(boost::mem_fn(&Marker6DoFs::CircularXYMenuCallBack),
                            this, _1));
            insert_counter++;
            menu_handler.setCheckState( T_XY_entry, interactive_markers::MenuHandler::UNCHECKED );
        }
        circularYZ_entry = menu_handler.insert(circual_entry, "YZ in T [sec]");
        insert_counter++;
        offset_menu_entry_semicircular_YZ = insert_counter;
        for ( int i=0; i<SECS; i++ )
        {
            std::ostringstream s;
            s <<i+1;
            T_YZ_entry = menu_handler.insert( circularYZ_entry, s.str(),
                boost::bind(boost::mem_fn(&Marker6DoFs::CircularYZMenuCallBack),
                            this, _1));
            insert_counter++;
            menu_handler.setCheckState( T_YZ_entry, interactive_markers::MenuHandler::UNCHECKED );
        }
        circularXZ_entry = menu_handler.insert(circual_entry, "XZ in T [sec]");
        insert_counter++;
        offset_menu_entry_semicircular_XZ = insert_counter;
        for ( int i=0; i<SECS; i++ )
        {
            std::ostringstream s;
            s <<i+1;
            T_XZ_entry = menu_handler.insert( circularXZ_entry, s.str(),
                boost::bind(boost::mem_fn(&Marker6DoFs::CircularXZMenuCallBack),
                            this, _1));
            insert_counter++;
            menu_handler.setCheckState( T_XZ_entry, interactive_markers::MenuHandler::UNCHECKED );
        }

        //#3 ResetMarker
        reset_marker_entry = menu_handler.insert("Reset Marker Pose",
            boost::bind(boost::mem_fn(&Marker6DoFs::ResetMarkerCb), this, _1));
        insert_counter++;
        menu_handler.setCheckState(reset_marker_entry, interactive_markers::MenuHandler::UNCHECKED);


        //#4 ResetTrj
        reset_trj_entry = menu_handler.insert("Reset Trajectory");
        insert_counter++;
        reset_last_entry = menu_handler.insert(reset_trj_entry, "Last",
            boost::bind(boost::mem_fn(&Marker6DoFs::ResetLastTrjCb), this, _1));
        insert_counter++;
        menu_handler.setCheckState(reset_last_entry, interactive_markers::MenuHandler::UNCHECKED);
        reset_all_entry = menu_handler.insert(reset_trj_entry, "All",
            boost::bind(boost::mem_fn(&Marker6DoFs::ResetTrjCb), this, _1));
        insert_counter++;
        menu_handler.setCheckState(reset_all_entry, interactive_markers::MenuHandler::UNCHECKED);

        //#5 Goal
        goal_entry = menu_handler.insert("Goal");
        insert_counter++;
        remove_goal_entry = menu_handler.insert(goal_entry, "Remove Goal",
            boost::bind(boost::mem_fn(&Marker6DoFs::RemoveGoalCb), this, _1));
        insert_counter++;
        menu_handler.setCheckState(remove_goal_entry, interactive_markers::MenuHandler::UNCHECKED);
        move_to_goal_entry = menu_handler.insert(goal_entry, "Move to Goal in T [sec]");
        insert_counter++;
        offset_menu_entry_goal_min_jerk = insert_counter;
        for ( int i=0; i<SECS; i++ )
        {
            std::ostringstream s;
            s <<i+1;
            goal_T_last = menu_handler.insert( move_to_goal_entry, s.str(),
                boost::bind(boost::mem_fn(&Marker6DoFs::MinJerkToGoalMenuCallBack),
                            this, _1));
            insert_counter++;
            menu_handler.setCheckState( goal_T_last, interactive_markers::MenuHandler::UNCHECKED );
        }
        marker_pose_to_goal_entry = menu_handler.insert(goal_entry, "Set Marker Pose",
            boost::bind(boost::mem_fn(&Marker6DoFs::SetMarkerPoseGoalCb), this, _1));
        insert_counter++;

        //#6 RestartMarker
        restart_marker_entry = menu_handler.insert("Restart Marker",
            boost::bind(boost::mem_fn(&Marker6DoFs::RestartCb), this, _1));
        insert_counter++;
        menu_handler.setCheckState(restart_marker_entry, interactive_markers::MenuHandler::UNCHECKED);


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

    visualization_msgs::Marker makeSphere( visualization_msgs::InteractiveMarker &msg )
    {
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = msg.scale * 1.;//0.45
      marker.scale.y = msg.scale * 1.;
      marker.scale.z = msg.scale * 1.;
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      marker.color.a = 1.0;

      return marker;
    }

    visualization_msgs::Marker makeSTL( visualization_msgs::InteractiveMarker &msg )
    {
        boost::shared_ptr<const urdf::Link> link = _urdf.getLink(_distal_link);
        boost::shared_ptr<const urdf::Link> controlled_link = link;

        KDL::Frame T; T.Identity();
        while(!link->visual)
                link = _urdf.getLink(link->parent_joint->parent_link_name);
        T = getPose(controlled_link->name, link->name);



        if(link->visual->geometry->type == urdf::Geometry::MESH)
        {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;

            boost::shared_ptr<urdf::Mesh> mesh =
                    boost::static_pointer_cast<urdf::Mesh>(link->visual->geometry);

            marker.mesh_resource = mesh->filename;


            KDL::Frame T_marker;
            T_marker.p.x(link->visual->origin.position.x);
            T_marker.p.y(link->visual->origin.position.y);
            T_marker.p.z(link->visual->origin.position.z);
            T_marker.M = T_marker.M.Quaternion(link->visual->origin.rotation.x,
                                  link->visual->origin.rotation.y,
                                  link->visual->origin.rotation.z,
                                  link->visual->origin.rotation.w);

            T = T*T_marker;
            marker.pose.position.x = T.p.x();
            marker.pose.position.y = T.p.y();
            marker.pose.position.z = T.p.z();
            double qx,qy,qz,qw; T.M.GetQuaternion(qx,qy,qz,qw);
            marker.pose.orientation.x = qx;
            marker.pose.orientation.y = qy;
            marker.pose.orientation.z = qz;
            marker.pose.orientation.w = qw;

            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;

            marker.scale.x = mesh->scale.x;
            marker.scale.y = mesh->scale.y;
            marker.scale.z = mesh->scale.z;
        }

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

    void MarkerFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        //In base_link
        tf::poseMsgToKDL(feedback->pose, actual_pose);
    }

    void ReverseMenuCallBack(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        reverse *= -1;
        if(reverse == 1)
            menu_handler.setCheckState(reverse_entry, interactive_markers::MenuHandler::UNCHECKED);
        else if(reverse == -1)
            menu_handler.setCheckState(reverse_entry, interactive_markers::MenuHandler::CHECKED);


        menu_handler.reApply(_server);
        _server.applyChanges();
    }

    segment_trj semiCircularTrj(const KDL::Vector& plane_normal)
    {
        KDL::Frame dist = start_pose.Inverse()*actual_pose;

        segment_trj seg;
        seg.start = start_pose;
        KDL::Vector n = start_pose.M*plane_normal;
        n=reverse*n;
        seg.plane_normal = n;
        seg.circle_center = start_pose.p + start_pose.M*(dist.p)/2.;
        seg.angle_rot = M_PI;
        seg.end_rot = actual_pose.M;
        seg.end = actual_pose;

        seg.type = trj_designer::trj_type::SEMI_CIRCLE;

        return seg;
    }

    void CircularXYMenuCallBack(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        KDL::Frame dist = start_pose.Inverse()*actual_pose;
        if(std::fabs(dist.p.z()) > 1e-4)
            ROS_ERROR("Target frame is not in XY plane!");
        else
        {
            KDL::Vector n(0,0,1);
            segment_trj seg = semiCircularTrj(n);
            seg.T = double(feedback->menu_entry_id-offset_menu_entry_semicircular_XY);
            segments_trj.push_back(seg);
            start_pose = actual_pose;
        }
    }

    void CircularXZMenuCallBack(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        KDL::Frame dist = start_pose.Inverse()*actual_pose;
        if(std::fabs(dist.p.y()) > 1e-4)
            ROS_ERROR("Target frame is not in XZ plane!");
        else
        {
            KDL::Vector n(0,1,0);
            segment_trj seg = semiCircularTrj(n);
            seg.T = double(feedback->menu_entry_id-offset_menu_entry_semicircular_XZ);
            segments_trj.push_back(seg);
            start_pose = actual_pose;
        }
    }

    void CircularYZMenuCallBack(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        KDL::Frame dist = start_pose.Inverse()*actual_pose;
        if(std::fabs(dist.p.x()) > 1e-4)
            ROS_ERROR("Target frame is not in YZ plane!");
        else
        {
            KDL::Vector n(1,0,0);
            segment_trj seg = semiCircularTrj(n);
            seg.T = double(feedback->menu_entry_id-offset_menu_entry_semicircular_YZ);
            segments_trj.push_back(seg);
            start_pose = actual_pose;
        }
    }

    void MinJerkToGoalMenuCallBack(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        if(_goal_pose)
        {
            int trj_type = trj_designer::trj_type::MIN_JERK;
            double T = double(feedback->menu_entry_id-offset_menu_entry_goal_min_jerk);

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
        int trj_type = trj_designer::trj_type::MIN_JERK;
        double T = double(feedback->menu_entry_id-offset_menu_entry_min_jerk);

        ROS_INFO("REQUEST:");
        ROS_INFO("  Trj Type: %i", trj_type);
        ROS_INFO("  base_link: %s", _base_link.c_str());
        ROS_INFO("  distal_link: %s", _distal_link.c_str());
        ROS_INFO("  Time [sec]: %f", T);
        printPose("  START", initial_pose);
        printPose("  END", actual_pose);

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

    KDL::Frame getPose(const std::string& base_link, const std::string& distal_link)
    {

        for(unsigned int i = 0; i < 10; ++i){
        try{
            ros::Time now = ros::Time::now();
            _listener.waitForTransform(base_link, distal_link,now,ros::Duration(1.0));

            _listener.lookupTransform(base_link, distal_link,
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

    void SetMarkerPoseGoalCb(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        if(_goal_pose)
        {
            geometry_msgs::Pose goal_pose;
            tf::poseKDLToMsg(*(_goal_pose.get()), goal_pose);

            _server.setPose(int_marker.name, goal_pose);
            _server.applyChanges();
        }
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

    void RestartCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        ROS_WARN("Restart");
        KDL::Frame robot_pose_KDL = getRobotActualPose();
        initial_pose = robot_pose_KDL;
        start_pose = robot_pose_KDL;
        actual_pose = robot_pose_KDL;
        printPose("initial_pose_"+_distal_link, robot_pose_KDL);

        segments_trj.clear();
        trj_pub->deleteAllMarkersAndTrj();

        geometry_msgs::Pose robot_pose;

        tf::poseKDLToMsg(robot_pose_KDL, robot_pose);

        _server.setPose(int_marker.name, robot_pose);
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
                if(trj.type == trj_designer::trj_type::MIN_JERK){
                    trj_gen->addMinJerkTrj(trj.start, trj.end, trj.T);
                }
                else if(trj.type == trj_designer::trj_type::SEMI_CIRCLE){
                    trj_gen->addArcTrj(trj.start, trj.end_rot, trj.angle_rot,
                                       trj.circle_center, trj.plane_normal, trj.T);
                }
            }


            //trj_pub->deleteAllMarkersAndTrj();
            trj_pub->setTrj(trj_gen->getTrajectory(), _base_link, _distal_link);

            trj_pub->publish();
        }
    }

    std::vector<segment_trj> getTrajSegments(){return segments_trj;}
    std::string getBaseLink(){return _base_link;}
    std::string getDistalLink(){return _distal_link;}
    visualization_msgs::InteractiveMarker getInteractiveMarker(){return int_marker;}
    double getTime(){return _dT;}

private:
    visualization_msgs::InteractiveMarker int_marker;
    interactive_markers::MenuHandler menu_handler;

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
    interactive_markers::MenuHandler::EntryHandle marker_pose_to_goal_entry;

    tf::TransformListener _listener;
    tf::StampedTransform _transform;

    interactive_markers::MenuHandler::EntryHandle circual_entry;
    interactive_markers::MenuHandler::EntryHandle reverse_entry;
    interactive_markers::MenuHandler::EntryHandle circularXY_entry;
    interactive_markers::MenuHandler::EntryHandle circularXZ_entry;
    interactive_markers::MenuHandler::EntryHandle circularYZ_entry;
    interactive_markers::MenuHandler::EntryHandle T_XY_entry;
    interactive_markers::MenuHandler::EntryHandle T_XZ_entry;
    interactive_markers::MenuHandler::EntryHandle T_YZ_entry;
    int reverse;

    interactive_markers::MenuHandler::EntryHandle restart_marker_entry;

    int offset_menu_entry_min_jerk;
    int offset_menu_entry_goal_min_jerk;
    int offset_menu_entry_semicircular_XY;
    int offset_menu_entry_semicircular_XZ;
    int offset_menu_entry_semicircular_YZ;

};

class trjBroadcaster
{
public:
    trjBroadcaster(Marker6DoFs& trj, ros::NodeHandle& n):
        _trj(trj)
    {
        _service = n.advertiseService("/"+_trj.getDistalLink()+"_getTrj", &trjBroadcaster::service_cb, this);
        _pub = n.advertise<trajectory_utils::CartesianTrj>("/"+_trj.getDistalLink()+"_trj", 1000);
        _pub2 = n.advertise<trajectory_utils::segmentTrj>("/"+_trj.getDistalLink()+"_segments", 1000);
    }

    bool service_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        std::vector<segment_trj> segments =  _trj.getTrajSegments();

        setSegments(segments);

        _trj_gen.reset(new trajectory_utils::trajectory_generator(
                           _trj.getTime(),
                           _trj.getBaseLink(), _trj.getDistalLink()));
        for(unsigned int i = 0; i < segments.size(); ++i)
        {
            segment_trj trj = segments[i];

            if(trj.type == trj_designer::trj_type::MIN_JERK){
                _trj_gen->addMinJerkTrj(trj.start, trj.end, trj.T);
            }
            else if(trj.type == trj_designer::trj_type::SEMI_CIRCLE){
                _trj_gen->addArcTrj(trj.start, trj.end_rot, trj.angle_rot,
                                   trj.circle_center, trj.plane_normal, trj.T);
            }
        }

        setTrj();

        _pub.publish(_msg);
        _pub2.publish(_msg2);

        return true;
    }

    void setSegments(std::vector<segment_trj>& segments)
    {
        _msg2.segments.clear();

        double qx,qy,qz,qw;
        for(unsigned int i = 0; i < segments.size(); ++i)
        {
            segment_trj seg = segments[i];

            trajectory_utils::segment seg_msg;
            //Type
            seg_msg.type.data = seg.type;
            //T
            seg_msg.T.data = seg.T;
            //Start
            seg_msg.start.frame.pose.position.x = seg.start.p.x();
            seg_msg.start.frame.pose.position.y = seg.start.p.y();
            seg_msg.start.frame.pose.position.z = seg.start.p.z();
            seg.start.M.GetQuaternion(qx,qy,qz,qw);
            seg_msg.start.frame.pose.orientation.x = qx;
            seg_msg.start.frame.pose.orientation.y = qy;
            seg_msg.start.frame.pose.orientation.z = qz;
            seg_msg.start.frame.pose.orientation.w = qw;
            seg_msg.start.frame.header.frame_id = _trj.getBaseLink();
            seg_msg.start.distal_frame = _trj.getDistalLink();
            //End
            seg_msg.end.frame.pose.position.x = seg.end.p.x();
            seg_msg.end.frame.pose.position.y = seg.end.p.y();
            seg_msg.end.frame.pose.position.z = seg.end.p.z();
            seg.end.M.GetQuaternion(qx,qy,qz,qw);
            seg_msg.end.frame.pose.orientation.x = qx;
            seg_msg.end.frame.pose.orientation.y = qy;
            seg_msg.end.frame.pose.orientation.z = qz;
            seg_msg.end.frame.pose.orientation.w = qw;
            seg_msg.end.frame.header.frame_id = _trj.getBaseLink();
            seg_msg.end.distal_frame = _trj.getDistalLink();

            if(seg.type == trj_designer::trj_type::SEMI_CIRCLE)
            {
                //end_rot
                seg_msg.end_rot = seg_msg.end.frame.pose.orientation;
                //angle_rot;
                seg_msg.angle_rot.data = seg.angle_rot;
                //circle_center;
                seg_msg.circle_center.x = seg.circle_center.x();
                seg_msg.circle_center.y = seg.circle_center.y();
                seg_msg.circle_center.z = seg.circle_center.z();
                //plane_normal;
                seg_msg.plane_normal.x = seg.plane_normal.x();
                seg_msg.plane_normal.y = seg.plane_normal.y();
                seg_msg.plane_normal.z = seg.plane_normal.z();
            }

            _msg2.segments.push_back(seg_msg);
        }
        _msg2.header.stamp = ros::Time::now();
    }

    void setTrj()
    {
        _msg.frames.clear();
        _msg.accelerations.clear();
        _msg.twists.clear();

        trajectory_utils::Cartesian T;
        KDL::Frame F;
        geometry_msgs::Twist twist;
        KDL::Twist v;
        geometry_msgs::Accel acc;
        KDL::Twist a;
        double qx,qy,qz,qw;

        while (!_trj_gen->isFinished()) {
            F = _trj_gen->Pos();
            v = _trj_gen->Vel();
            a = _trj_gen->Acc();

            T.frame.pose.position.x = F.p.x();
            T.frame.pose.position.y = F.p.y();
            T.frame.pose.position.z = F.p.z();
            F.M.GetQuaternion(qx,qy,qz,qw);
            T.frame.pose.orientation.x = qx;
            T.frame.pose.orientation.y = qy;
            T.frame.pose.orientation.z = qz;
            T.frame.pose.orientation.w = qw;

            T.frame.header.frame_id = _trj.getBaseLink();

            T.distal_frame = _trj.getDistalLink();

            _msg.frames.push_back(T);


            twist.linear.x = v.vel.x();
            twist.linear.y = v.vel.y();
            twist.linear.z = v.vel.z();
            twist.angular.x = v.rot.x();
            twist.angular.y = v.rot.y();
            twist.angular.z = v.rot.z();

            _msg.twists.push_back(twist);

            acc.linear.x = a.vel.x();
            acc.linear.y = a.vel.y();
            acc.linear.z = a.vel.z();
            acc.angular.x = a.rot.x();
            acc.angular.y = a.rot.y();
            acc.angular.z = a.rot.z();

            _msg.accelerations.push_back(acc);

            _trj_gen->updateTrj();
        }

        _msg.header.stamp = ros::Time::now();
        _msg.dT = _trj_gen->getSampleTime();
    }

private:
    Marker6DoFs& _trj;
    trajectory_utils::CartesianTrj _msg;
    ros::Publisher _pub;
    ros::Publisher _pub2;
    ros::ServiceServer _service;
    boost::shared_ptr<trajectory_utils::trajectory_generator> _trj_gen;
    trajectory_utils::segmentTrj _msg2;
};

}

#endif
