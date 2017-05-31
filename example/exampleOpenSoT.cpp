#include <ros/ros.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#include <advr_humanoids_common_utils/idynutils.h>
#include <sensor_msgs/JointState.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <nav_msgs/Path.h>
#include <kdl_conversions/kdl_msg.h>
#include <std_srvs/Empty.h>

static void null_deleter(idynutils2 *) {}

nav_msgs::Path left_arm_trj;
nav_msgs::Path right_arm_trj;

OpenSoT::AutoStack::Ptr auto_stack;
Eigen::VectorXd q;
Eigen::VectorXd q0;
boost::shared_ptr<idynutils2> robot;
OpenSoT::solvers::QPOases_sot::Ptr solver;
XBot::ModelInterfaceIDYNUTILS::Ptr model_ptr;
OpenSoT::tasks::velocity::Cartesian::Ptr left_arm;
OpenSoT::tasks::velocity::Cartesian::Ptr right_arm;
OpenSoT::tasks::velocity::Postural::Ptr postural;
OpenSoT::constraints::velocity::JointLimits::Ptr joint_lims;
OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_lims;
std::string left_arm_distal_link, left_arm_base_link;
std::string right_arm_distal_link, right_arm_base_link;

int left_counter = -1;
int right_counter = -1;

bool solve = false;

bool service_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    q = q0;
    robot->updateiDynTreeModel(q, true);


    left_arm.reset(new OpenSoT::tasks::velocity::Cartesian("left_arm", q, *(model_ptr.get()),
                                                           left_arm_distal_link,
                                                           left_arm_base_link));

    left_arm->setOrientationErrorGain(0.1);
    right_arm.reset(new OpenSoT::tasks::velocity::Cartesian("right_arm", q, *(model_ptr.get()),
                                                            right_arm_distal_link,
                                                            right_arm_base_link));

    right_arm->setOrientationErrorGain(0.1);
    postural.reset(new OpenSoT::tasks::velocity::Postural(q));



    Eigen::VectorXd qmin, qmax;
    model_ptr->getJointLimits(qmin, qmax);
    joint_lims.reset(new OpenSoT::constraints::velocity::JointLimits(q, qmax, qmin));

    //vel_lims.reset(new OpenSoT::constraints::velocity::VelocityLimits(M_PI, 0.001, q.size()));

    auto_stack = (left_arm + right_arm)/
                 (postural)<<joint_lims;//<<vel_lims;
    auto_stack->update(q);

    solver.reset(new OpenSoT::solvers::QPOases_sot(auto_stack->getStack(),
                                                  auto_stack->getBounds(), 1e10));

    if(left_arm_trj.poses.size() > 0 || right_arm_trj.poses.size() > 0){
        left_counter = -1;
        right_counter = -1;

        solve = true;
    }
}

void left_cb(const nav_msgs::Path::ConstPtr& msg)
{
  left_arm_trj = *(msg.get());
}

void right_cb(const nav_msgs::Path::ConstPtr& msg)
{
  right_arm_trj = *(msg.get());
}

void publishJointState(const Eigen::VectorXd& q, const XBot::ModelInterfaceIDYNUTILS::Ptr model,
                       const ros::Publisher& joint_pub)
{
    sensor_msgs::JointState joints_state_msg;
    for(unsigned int i = 0; i < q.size(); ++i)
    {
        joints_state_msg.name.push_back(model->getJointByDofIndex(i)->getJointName());
        joints_state_msg.position.push_back(q[i]);
        joints_state_msg.velocity.push_back(0.0);
        joints_state_msg.effort.push_back(0.0);
    }
    joints_state_msg.header.stamp = ros::Time::now();

    joint_pub.publish(joints_state_msg);
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



    robot.reset( new idynutils2("robot", urdf_path, srdf_path));
    model_ptr = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
            (XBot::ModelInterface::getModel(config_path));
    model_ptr->loadModel(boost::shared_ptr<idynutils2>(&(*(robot.get())), &null_deleter));
    if(!model_ptr)
        std::cout<<"pointer is NULL "<<model_ptr.get()<<std::endl;

    q0.resize(model_ptr->getJointNum()); q0.setZero(q0.size());
    std::map<std::string,double> joints_initial_value;
    nh.getParam("initial_configuration", joints_initial_value);
    if(joints_initial_value.size() > 0)
    {
        std::map<std::string, double>::iterator it = joints_initial_value.begin();
        while (it != joints_initial_value.end()) {
            q0[model_ptr->getDofIndex(it->first)] = it->second;
            it++;
        }
    }
    q = q0;

    robot->updateiDynTreeModel(q, true);

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);


    ros::Subscriber sub_left_arm =  nh.subscribe("/"+left_arm_distal_link+"_trj_viz", 1000, left_cb);
    ros::Subscriber sub_right_arm = nh.subscribe("/"+right_arm_distal_link+"_trj_viz", 1000, right_cb);
    ros::ServiceServer service = nh.advertiseService("/IK", service_cb);


    Eigen::VectorXd dq(q.size()); dq.setZero(dq.size());
    ROS_INFO("Running example_open_sot_node");
    while(ros::ok())
    {
        KDL::Frame l_goal; l_goal.Identity();
        KDL::Frame r_goal; r_goal.Identity();


        if(solve)
        {
            if(left_arm_trj.poses.size() > 0){
                left_counter++;
                if(left_counter < left_arm_trj.poses.size()){
                    geometry_msgs::PoseStamped p = left_arm_trj.poses[left_counter];
                    tf::PoseMsgToKDL(p.pose, l_goal);
                    left_arm->setReference(l_goal);
                }
            }

            if(right_arm_trj.poses.size() > 0){
                right_counter++;
                if(right_counter < right_arm_trj.poses.size()){
                    geometry_msgs::PoseStamped p = right_arm_trj.poses[right_counter];
                    tf::PoseMsgToKDL(p.pose, r_goal);
                    right_arm->setReference(r_goal);
                }
            }

            if(right_counter == right_arm_trj.poses.size() &&
               left_counter == left_arm_trj.poses.size()){
                solve = false;
            }



        }

        if(solve){
            robot->updateiDynTreeModel(q, true);

            auto_stack->update(q);

            if(solver->solve(dq))
                q+=dq;
            else
                ROS_ERROR("Solver error!!!");
        }

        publishJointState(q, model_ptr, joint_pub);
        ros::spinOnce();

        usleep(100000);
    }

}
