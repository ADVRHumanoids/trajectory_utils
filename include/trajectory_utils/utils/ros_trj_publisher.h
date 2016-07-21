/*
 * Copyright (C) 2016 Walkman
 * Author: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef __ROS_TRJ_PUBLISHER_H__
#define __ROS_TRJ_PUBLISHER_H__

#include <ros/ros.h>
#include <ros/publisher.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include "trajectory_utils/trajectory_utils.h"
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace trajectory_utils{

class trajectory_publisher
{
public:
    trajectory_publisher(const std::string& topic_name);

    void setTrj(const boost::shared_ptr<KDL::Trajectory_Composite> trj, const std::string& frame);
    void setDecimation(const double decimation);
    void publish();

private:
    ros::NodeHandle _n;

    ros::Publisher _trj_publisher;
    nav_msgs::Path _trj_msg;

    rviz_visual_tools::RvizVisualToolsPtr _visual_tools;


    double _decimation;
    int _decimation2;

};

}


#endif
