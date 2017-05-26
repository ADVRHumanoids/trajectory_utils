#ifndef _TRJ_DESIGNER_H_
#define _TRJ_DESIGNER_H_

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

namespace trj_designer{


class Marker6DoFs{
public:
    Marker6DoFs(const std::string& distal_link,
                bool fixed,
                unsigned int _interaction_mode,
                bool show_6dof ):
        interaction_mode(_interaction_mode)
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

    visualization_msgs::InteractiveMarker int_marker;
    unsigned int interaction_mode;
private:
    visualization_msgs::InteractiveMarkerControl control;
    visualization_msgs::InteractiveMarkerControl control2;
    visualization_msgs::Marker marker;



};

}

#endif
