#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <trajectory_utils/CartesianTrj.h>
#include <sensor_msgs/JointState.h>
#include <kdl_conversions/kdl_msg.h>
#include <std_srvs/Empty.h>
#include <trajectory_utils/ros_nodes/trj_designer.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <XBotInterface/ModelInterface.h>

trajectory_utils::CartesianTrj left_arm_trj;
trajectory_utils::CartesianTrj right_arm_trj;

OpenSoT::AutoStack::Ptr auto_stack;
Eigen::VectorXd q;
std::vector<Eigen::VectorXd> qd;
Eigen::VectorXd q0;
Eigen::VectorXd dq;
std::vector<Eigen::VectorXd> dqd;
OpenSoT::solvers::QPOases_sot::Ptr solver;
XBot::ModelInterface::Ptr model_ptr;
OpenSoT::tasks::velocity::Cartesian::Ptr left_arm;
OpenSoT::tasks::velocity::Cartesian::Ptr right_arm;
OpenSoT::tasks::velocity::Postural::Ptr postural;
OpenSoT::constraints::velocity::JointLimits::Ptr joint_lims;
OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_lims;
std::string left_arm_distal_link, left_arm_base_link;
std::string right_arm_distal_link, right_arm_base_link;
ros::Publisher joint_trajectory_desired_pub;
std::string tf_prefix;




int left_counter = -1;
int right_counter = -1;

double dT;

bool solve = false;

bool service_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    q = q0;
    dq.setZero(q.size());

    model_ptr->setJointPosition(q);
    model_ptr->update();


    left_arm.reset(new OpenSoT::tasks::velocity::Cartesian("left_arm", q, *(model_ptr.get()),
                                                           left_arm_distal_link,
                                                           left_arm_base_link));

    left_arm->setOrientationErrorGain(1.0);

    right_arm.reset(new OpenSoT::tasks::velocity::Cartesian("right_arm", q, *(model_ptr.get()),
                                                            right_arm_distal_link,
                                                            right_arm_base_link));
    right_arm->setOrientationErrorGain(1.0);

    KDL::Frame F;
    right_arm->getActualPose(F);
    trj_designer::Marker6DoFs::printPose("right arm initial pose", F);

    right_arm->setOrientationErrorGain(1.0);
    postural.reset(new OpenSoT::tasks::velocity::Postural(q));



    Eigen::VectorXd qmin, qmax;
    model_ptr->getJointLimits(qmin, qmax);
    joint_lims.reset(new OpenSoT::constraints::velocity::JointLimits(q, qmax, qmin));

    vel_lims.reset(new OpenSoT::constraints::velocity::VelocityLimits(M_PI, dT, q.size()));

    auto_stack = (left_arm + right_arm)/
                 (postural)<<joint_lims<<vel_lims;
    auto_stack->update(q);

    solver.reset(new OpenSoT::solvers::QPOases_sot(auto_stack->getStack(),
                                                  auto_stack->getBounds(), 1e10));

    if(left_arm_trj.frames.size() > 0){
        left_counter = -1;
        solve = true;
    }
    if(right_arm_trj.frames.size() > 0){
        right_counter = -1;
        solve = true;
    }

    qd.clear();
    qd.push_back(q);
    dqd.clear();
    dqd.push_back(dq);
    return true;
}

void left_cb(const trajectory_utils::CartesianTrj::ConstPtr& msg)
{
  left_arm_trj = *(msg.get());
}

void right_cb(const trajectory_utils::CartesianTrj::ConstPtr& msg)
{
  right_arm_trj = *(msg.get());
}

void publishJointState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                       const XBot::ModelInterface::Ptr model,
                       const ros::Publisher& joint_pub)
{
    sensor_msgs::JointState joints_state_msg;
    for(unsigned int i = 0; i < q.size(); ++i)
    {
        joints_state_msg.name.push_back(model->getJointByDofIndex(i)->getJointName());
        joints_state_msg.position.push_back(q[i]);
        joints_state_msg.velocity.push_back(dq[i]);
        joints_state_msg.effort.push_back(0.0);
    }
    joints_state_msg.header.stamp = ros::Time::now();

    joint_pub.publish(joints_state_msg);
}

bool service_cb2(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    trajectory_msgs::JointTrajectory joint_trajectory_msg;

    if(qd.size() > 0)
    {
        for(unsigned int i = 0; i < q.size(); ++i)
            joint_trajectory_msg.joint_names.push_back(model_ptr->getJointByDofIndex(i)->getJointName());

        for(unsigned int i = 0; i < qd.size(); ++i)
        {

            trajectory_msgs::JointTrajectoryPoint p;
            for(unsigned int j = 0; j < q.size(); ++j){
                p.positions.push_back(qd.at(i)[j]);
                p.velocities.push_back(dqd.at(i)[j]);
                p.accelerations.push_back(0.0);
                p.effort.push_back(0.0);
            }
            p.time_from_start = ros::Duration(dT);

            joint_trajectory_msg.points.push_back(p);

        }
        joint_trajectory_msg.header.stamp = ros::Time::now();
        joint_trajectory_desired_pub.publish(joint_trajectory_msg);
    }
    return true;
}

void resetTrj(trajectory_utils::CartesianTrj& trajectory_msg)
{
    trajectory_msg.frames.clear();
    trajectory_msg.accelerations.clear();
    trajectory_msg.twists.clear();
    trajectory_msg.dT = 0.;
}

bool service_cb3(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    q0 = q;
    qd.clear();
    resetTrj(left_arm_trj);
    resetTrj(right_arm_trj);
    return true;
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example_open_sot");
    ros::NodeHandle nh("~");

    std::string config_path;
    nh.getParam("config_path", config_path);
    ROS_INFO("CONFIG PATH: %s", config_path.c_str());
    std::string urdf_path;
    nh.getParam("urdf_path", urdf_path);
    ROS_INFO("URDF PATH: %s", urdf_path.c_str());
    std::string srdf_path;
    nh.getParam("srdf_path", srdf_path);
    ROS_INFO("SRDF PATH: %s", srdf_path.c_str());


    nh.getParam("left_arm_distal_link", left_arm_distal_link);
    ROS_INFO("left_arm_distal_link: %s", left_arm_distal_link.c_str());
    nh.getParam("right_arm_distal_link", right_arm_distal_link);
    ROS_INFO("right_arm_distal_link: %s", right_arm_distal_link.c_str());
    nh.getParam("left_arm_base_link", left_arm_base_link);
    ROS_INFO("left_arm_base_link: %s", left_arm_base_link.c_str());
    nh.getParam("right_arm_base_link", right_arm_base_link);
    ROS_INFO("right_arm_base_link: %s", right_arm_base_link.c_str());

    nh.getParam("dT", dT);
    ROS_INFO("dT is : %f [sec]", dT);

    model_ptr = XBot::ModelInterface::getModel(config_path);
    if(!model_ptr)
        std::cout<<"pointer is NULL "<<model_ptr.get()<<std::endl;

    q0.resize(model_ptr->getJointNum()); q0.setZero(q0.size());
    std::map<std::string,double> joints_initial_value;
    nh.getParam("/zeros", joints_initial_value);
    if(joints_initial_value.size() > 0)
    {
        ROS_INFO("zeros:");
        std::map<std::string, double>::iterator it = joints_initial_value.begin();
        while (it != joints_initial_value.end()) {
            ROS_INFO("  %s --> %f", it->first.c_str(), it->second);
            q0[model_ptr->getDofIndex(it->first)] = it->second;
            it++;
        }
    }
    q = q0;
    qd.push_back(q);
    dq.setZero(q.size());
    dqd.push_back(dq);

    model_ptr->setJointPosition(q);
    model_ptr->update();

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
    joint_trajectory_desired_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory", 1000);


    ros::Subscriber sub_left_arm =  nh.subscribe("/"+left_arm_distal_link+"_trj", 1000, left_cb);
    ros::Subscriber sub_right_arm = nh.subscribe("/"+right_arm_distal_link+"_trj", 1000, right_cb);
    ros::ServiceServer service = nh.advertiseService("/IK", service_cb);
    ros::ServiceServer service2 = nh.advertiseService("/publish_joint_desired", service_cb2);
    ros::ServiceServer service3 = nh.advertiseService("/reset", service_cb3);


    ROS_INFO("Running example_open_sot_node");
    while(ros::ok())
    {
        KDL::Frame l_goal; l_goal.Identity();
        KDL::Frame r_goal; r_goal.Identity();
        KDL::Twist vl_goal; vl_goal.Zero();
        KDL::Twist vr_goal; vr_goal.Zero();

        if(solve)
        {
            if(left_arm_trj.frames.size() > 0){
                left_counter++;
                if(left_counter < left_arm_trj.frames.size()){
                    geometry_msgs::PoseStamped p = left_arm_trj.frames[left_counter].frame;
                    geometry_msgs::Twist v = left_arm_trj.twists[left_counter];

                    tf::PoseMsgToKDL(p.pose, l_goal);
                    tf::TwistMsgToKDL(v, vl_goal);


                    left_arm->setReference(l_goal, vl_goal*dT);
                }
            }

            if(right_arm_trj.frames.size() > 0){
                right_counter++;
                if(right_counter < right_arm_trj.frames.size()){
                    geometry_msgs::PoseStamped p = right_arm_trj.frames[right_counter].frame;
                    geometry_msgs::Twist v = right_arm_trj.twists[right_counter];

                    tf::PoseMsgToKDL(p.pose, r_goal);
                    tf::TwistMsgToKDL(v, vr_goal);

                    right_arm->setReference(r_goal, vr_goal*dT);
                }
            }

            if(right_counter == right_arm_trj.frames.size() &&
               left_counter == left_arm_trj.frames.size() ||
               right_counter == right_arm_trj.frames.size() &&
               left_counter == -1 ||
               left_counter == left_arm_trj.frames.size() &&
               right_counter == -1 ){
                solve = false;
            }

        }

        if(solve){
            model_ptr->setJointPosition(q);
            model_ptr->update();

            auto_stack->update(q);

            if(solver->solve(dq)){
                q+=dq;
                qd.push_back(q);
                dqd.push_back(dq);}
            else
                ROS_ERROR("Solver error!!!");
        }

        publishJointState(q, dq, model_ptr, joint_pub);
        ros::spinOnce();

        ros::Duration(dT).sleep();
    }

}
