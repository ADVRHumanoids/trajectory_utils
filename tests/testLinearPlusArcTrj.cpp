#include <gtest/gtest.h>
#include <trajectory_utils/trajectory_utils.h>
#include <fstream>

#define dt 0.001

namespace {

class testLinearPlusArcTrj: public ::testing::Test {
public:
    testLinearPlusArcTrj():
        trj(dt)
    {
        start = start.Identity();
        end_rot = start.M;

        angle = M_PI_2;

        circle_center[0] = 0.0;
        circle_center[1] = 1.0;
        circle_center[2] = 0.0;

        plane_normal[0] = 1.0;
        plane_normal[1] = 0.0;
        plane_normal[2] = 0.0;
    }

    virtual ~testLinearPlusArcTrj() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    void logFrame(const KDL::Frame& F) {
        _stored_frames.push_back(F);
    }

    void writeMatlabFile(double k = -1) {
        std::string file_name = "stored_trj.m";
        file1.open(file_name.c_str());
        file1<<"trj = {"<<std::endl;

        for(unsigned int i = 0; i < _stored_frames.size(); ++i){
            KDL::Frame T = _stored_frames[i];
            file1<<"["<<T(0,0)<<" "<<T(0,1)<<" "<<T(0,2)<<" "<<T(0,3)<<"; "<<
                   T(1,0)<<" "<<T(1,1)<<" "<<T(1,2)<<" "<<T(1,3)<<"; "<<
                   T(2,0)<<" "<<T(2,1)<<" "<<T(2,2)<<" "<<T(2,3)<<"; "<<
                   T(3,0)<<" "<<T(3,1)<<" "<<T(3,2)<<" "<<T(3,3)<<"],"<<std::endl;
        }
        file1<<"};"<<std::endl;
        file1.close();

        file_name = "plot_trj.m";
        file2.open(file_name.c_str());

        file2<<"clear all; clc"<<std::endl;
        file2<<"run('startup_rvc.m') %Needs Matlab Robotics Toolbox to run!"<<std::endl;
        file2<<"run('stored_trj.m')"<<std::endl;

        file2<<"X = []; Y = []; Z = [];"<<std::endl;
        file2<<"for i = 1:1:length(trj)"<<std::endl;
        file2<<"    X(i) = trj{i}(1,4);"<<std::endl;
        file2<<"    Y(i) = trj{i}(2,4);"<<std::endl;
        file2<<"    Z(i) = trj{i}(3,4);"<<std::endl;
        file2<<"end"<<std::endl;

        file2<<"plot3(X,Y,Z,'.'); axis([-2,2,-2,2,-2,2]); hold on"<<std::endl;
        file2<<"trplot(trj{1},'axis',[-2,2,-2,2,-2,2],'color',[0 0 0]); hold on;"<<std::endl;
        file2<<"trplot(trj{length(trj)},'axis',[-2,2,-2,2,-2,2],'color',[0 0 0]); hold on;"<<std::endl;

        file2<<"for i = 2:"<<k<<":length(trj)-1"<<std::endl;
        file2<<"    trplot(trj{i},'axis',[-2,2,-2,2,-2,2]); hold on;"<<std::endl;
        file2<<"end"<<std::endl;
        file2.close();
    }



    std::ofstream file1;
    std::ofstream file2;

    std::vector<KDL::Frame> _stored_frames;

    trajectory_utils::trajectory_generator trj;
    KDL::Frame start;
    KDL::Rotation end_rot;

    double angle;

    KDL::Vector circle_center;

    KDL::Vector plane_normal;

};

TEST_F(testLinearPlusArcTrj, testLinearPlusArcTrj_)
{
    KDL::Frame start; start.Identity();
    KDL::Frame end = start; end.p.x(start.p.x()+0.3);

    double T = 5.;

    this->trj.addLineTrj(start, end, T);


    this->circle_center = end.p;
    this->circle_center[1] = 1.0;

    this->angle = M_PI_2;
    end_rot = end.M;
    this->end_rot.DoRotX(M_PI_2);

    this->trj.addArcTrj(end, this->end_rot, this->angle, this->circle_center,
                        this->plane_normal, T);


    for(double t = 0.0; t <= this->trj.Duration(); t+=dt){
        this->trj.updateTrj();
        this->logFrame(this->trj.Pos());}
    this->writeMatlabFile(int((T*1000)/2.));
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
