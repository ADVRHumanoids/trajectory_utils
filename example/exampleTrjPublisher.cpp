#include <trajectory_utils/utils/ros_trj_publisher.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_trj_publisher");

    trajectory_utils::trajectory_generator trj(0.001);

    KDL::Frame start; start = start.Identity();
    KDL::Frame end = start;
    end.p.x(0.4);

    double T = 5.;
    double alpha = M_PI;

    KDL::Vector n;
    n[0] = 1.;
    n[1] = 0.;
    n[2] = 0.;

    KDL::Rotation end2 = end.M;
    end2.DoRotX(M_PI);

    KDL::Vector c = end.p;
    c[1] = 1.0;

    trj.addLineTrj(start, end, T);
    trj.addArcTrj(end, end2, alpha, c,n,T);

    trajectory_utils::trajectory_publisher trj_publisher("trj");
    trj_publisher.setTrj(trj.getTrajectory(), "world", "hand");

    while(ros::ok()){
        trj_publisher.publish();
        ros::spinOnce();}


    return 0;
}
