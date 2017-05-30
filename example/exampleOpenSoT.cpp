#include <ros/ros.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#include <advr_humanoids_common_utils/idynutils.h>
#include <sensor_msgs/JointState.h>

static void null_deleter(idynutils2 *) {}

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

    idynutils2 robot("robot", urdf_path, srdf_path);
    XBot::ModelInterfaceIDYNUTILS::Ptr model_ptr;
    model_ptr = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
            (XBot::ModelInterface::getModel(config_path));
    model_ptr->loadModel(boost::shared_ptr<idynutils2>(&robot, &null_deleter));
    if(!model_ptr)
        std::cout<<"pointer is NULL "<<model_ptr.get()<<std::endl;

    Eigen::VectorXd q(model_ptr->getJointNum()); q.setZero(q.size());
    q[model_ptr->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
    q[model_ptr->getDofIndex("LShLat")] = 10.0*M_PI/180.0;
    q[model_ptr->getDofIndex("LElbj")] = -80.0*M_PI/180.0;
    q[model_ptr->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
    q[model_ptr->getDofIndex("RShLat")] = -10.0*M_PI/180.0;
    q[model_ptr->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);

    ROS_INFO("Running example_open_sot_node");
    while(ros::ok())
    {
        publishJointState(q, model_ptr, joint_pub);
        ros::spinOnce();
    }

}
