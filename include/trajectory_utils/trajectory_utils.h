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

#ifndef __TRAJECTORY_UTILS_H__
#define __TRAJECTORY_UTILS_H__

#include <kdl/path_circle.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_point.hpp>
#include <kdl/path_composite.hpp>

#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_spline.hpp>

#include <kdl/rotational_interpolation_sa.hpp>

#include <boost/shared_ptr.hpp>

namespace trajectory_utils{

enum velocity_profile{
    BANG_COAST_BANG
};

class trajectory_generator{
public:
    /**
     * @brief trajectory_generator constructor
     * @param dt loop time
     */
    trajectory_generator(const double dt);

    /**
     * @brief getTrajectory
     * @return the sharep pointer to the KDL trajectory object
     */
    const boost::shared_ptr<KDL::Trajectory_Composite>& getTrajectory();

    /**
     * @brief resetTrajectory clear the trajectory object (all the stored trajectories are deleted)
     */
    void resetTrajectory();

    bool addArcTrj(const KDL::Frame &start_pose, const KDL::Rotation &final_rotation,
                   const double angle_of_rotation,
                   const KDL::Vector &circle_center, const KDL::Vector &plane_normal,
                   const double T);

    /**
     * @brief addArcTrj add an arc trajectory
     * @param vel_profile desired velocity profile
     * @param start_pose on the arc trajectory
     * @param final_rotation final rotation at the end of the arc trajectory
     * @param angle_of_rotation amount of rotation (arc trajectory)
     * @param circle_center center of the arc trajectory
     * @param plane_normal normal of the plane where the arc trajecotry is
     * @param max_vel max velocity of the trajectory
     * @param max_acc max acceleration of the trajectory
     * @return true if the trajectories has been added
     */
    bool addArcTrj(const velocity_profile vel_profile,
                   const KDL::Frame& start_pose, const KDL::Rotation& final_rotation,
                   const double angle_of_rotation,
                   const KDL::Vector& circle_center, const KDL::Vector& plane_normal,
                   const double max_vel, const double max_acc);

    /**
     * @brief addLineTrj add a set of linear trajectories, specified by way-points and each
     * costituted by a BANG_COAST_BANG velocity profile. Here we assume that the BANG phases least as
     * the COAST phase
     * @param way_points NOTE that the first way-point is the start!
     * @param T all the segments least the same time
     * @return true if the trajectories has been added
     */
    bool addLineTrj(const std::vector<KDL::Frame>& way_points, double T);

    /**
     * @brief addLineTrj add a set of linear trajectories, specified by way-points and each
     * costituted by a BANG_COAST_BANG velocity profile. Here we assume that the BANG phases least as
     * the COAST phase
     * @param way_points NOTE that the first way-point is the start!
     * @param T duration of each segment of trajectory
     * @return true if the trajectories has been added
     */
    bool addLineTrj(const std::vector<KDL::Frame>& way_points, const std::vector<double> T);

    /**
     * @brief addLineTrj add a set of linear trajectories, specified by way-points,
     * each constituted by a linear path and a specified velocity profile.
     * @param vel_profile desired velocity profile
     * @param way_points NOTE that the first way-point is the start!
     * @param max_vels max velocity for all the trajectories
     * @param max_accs max acceleration for all the trajectories
     * @return true if the trajectories has been added
     */
    bool addLineTrj(const velocity_profile vel_profile, const std::vector<KDL::Frame>& way_points,
                     const double max_vel, const double max_acc);

    /**
     * @brief addLineTrj add a set of linear trajectories, specified by way-points,
     * each constituted by a linear path and a specified velocity profile.
     * @param vel_profile desired velocity profile
     * @param way_points NOTE that the first way-point is the start!
     * @param max_vels max velocity for each trajectory
     * @param max_accs max acceleration for each trajectory
     * @return true if the trajectories has been added
     */
    bool addLineTrj(const velocity_profile vel_profile, const std::vector<KDL::Frame>& way_points,
                     const std::vector<double> max_vels, const std::vector<double> max_accs);
    /**
     * @brief addLineTrj add a trajectory constituted by a linear path and a specified velocity profile
     * @param vel_profile desired velocity profile
     * @param start Frame
     * @param end Frame
     * @param max_vel max velocity of the trajectory
     * @param max_acc max acceleration of the trajectory
     * @return true if the trajectory has been added
     */
    bool addLineTrj(const velocity_profile vel_profile, const KDL::Frame& start, const KDL::Frame& end,
                    const double max_vel, const double max_acc);

    /**
     * @brief addLineTrj add a trajectory of duration T constituted by a linear path and a BANG_COAST_BANG
     * velocity profile. Here we assume that the BANG phases least as the COAST phase
     * @param start Frame
     * @param end Frame
     * @param T total time of the trajectory
     * @return true if the trajectory has been added
     */
    bool addLineTrj(const KDL::Frame& start, const KDL::Frame& end, const double T);

    /**
     * @brief Pos return a frame from the trajectory at time t
     * @param t time
     * @return a frame
     */
    KDL::Frame Pos(double t);

    /**
     * @brief Vel a twist from the trajectory at time t
     * @param t time
     * @return a twist
     */
    KDL::Twist Vel(double t);

    /**
     * @brief Acc a twist (representing accelerations) from the trajectory at time t
     * @param t time
     * @return a twist
     */
    KDL::Twist Acc(double t);

    /**
     * @brief Pos a frame from the trajectory at actual (internal) time
     * @return a frame
     */
    KDL::Frame Pos();

    /**
     * @brief Vel a twist from the trajectory at actual (internal) time
     * @return a twist
     */
    KDL::Twist Vel();

    /**
     * @brief Acc a twist (representing accelerations) from the trajectory at actual (internal) time
     * @return a twist
     */
    KDL::Twist Acc();

    /**
     * @brief updateTrj increment internal time
     */
    void updateTrj();

    /**
     * @brief resetInternalTime set internal time to 0
     */
    void resetInternalTime();

    /**
     * @brief getTime return internal time
     * @return time
     */
    double getTime();

    /**
     * @brief Duration fot he whole trajectory
     * @return time of the trajectory
     */
    double Duration();

    /**
     * @brief isFinished
     * @return true if internal time >= duration
     */
    bool isFinished();

    /**
     * @brief isStarted
     * @return true if time > 0.0
     */
    bool isStarted();

    /**
     * @brief isRunning
     * @return true if started and not finished
     */
    bool isRunning();

    /**
     * @brief isInited
     * @return true if at least one trajectory has been added
     */
    bool isInited();


protected:
    double _dt;
    double _time;
    double _eq_radius;
    bool   _is_inited;

    boost::shared_ptr<KDL::Trajectory_Composite> _trj;

    boost::shared_ptr<KDL::Path> createLinePath(const KDL::Frame& start, const KDL::Frame& end);

    boost::shared_ptr<KDL::Path> createArcPath(const KDL::Frame& start_pose,
                                               const KDL::Rotation& final_rotation,
                                               const double angle_of_rotation,
                                               const KDL::Vector& circle_center,
                                               const KDL::Vector& plane_normal);

    boost::shared_ptr<KDL::VelocityProfile> createTrapezoidalVelProfile(const double max_vel, const double max_acc,
                                                                        const double L);

   bool checkIfCoastPhaseExists(const double max_vel, const double max_acc, const double L);

   void normalizeQuaternion(KDL::Frame& T);

   void computeMaxVelAndMaxAccForBCB(const double T, const double L, double& max_vel, double& max_acc);
};

}

#endif
