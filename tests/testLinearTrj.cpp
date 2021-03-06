#include <gtest/gtest.h>
#include <trajectory_utils/trajectory_utils.h>
#include <fstream>

#define dt 0.001

namespace {

class test_trajectory_generator: public trajectory_utils::trajectory_generator
{
public:
    test_trajectory_generator():
    trajectory_generator(dt)
    {}

    boost::shared_ptr<KDL::Path> _createLinePath(const KDL::Frame& start, const KDL::Frame& end){
        return createLinePath(start, end);
    }

    boost::shared_ptr<KDL::VelocityProfile> _createTrapezoidalVelProfile(const double max_vel, const double max_acc,
                                                                        const double L){
        return createTrapezoidalVelProfile(max_vel, max_acc, L);
    }

    bool _checkIfCoastPhaseExists(const double max_vel, const double max_acc, const double L){
        return checkIfCoastPhaseExists(max_vel, max_acc, L);
    }

    void _normalizeQuaternion(KDL::Frame& T){
        normalizeQuaternion(T);
    }

    void _computeMaxVelAndMaxAccForBCB(const double T, const double L, double& max_vel, double& max_acc){
        computeMaxVelAndMaxAccForBCB(T,L,max_vel, max_acc);
    }
};

class testLinearTrj: public ::testing::Test {
public:
    testLinearTrj()
    {
        start.Identity();
        start.p.x(0.871571 );
        start.p.y(-0.156488);
        start.p.z( 0.98008);
        start.M = start.M.Quaternion(0.425109, 0.654533, -0.364139, 0.508202);

        end.Identity();
        end.p.x(0.978272 );
        end.p.y(-0.429117 );
        end.p.z(0.914601);
        end.M = end.M.Quaternion(0.425109, 0.654533, -0.364139, 0.508202);

        KDL::Frame tmp; tmp.Identity();
        way_points.push_back(tmp);
        tmp.p.x(tmp.p.x()+0.1);
        way_points.push_back(tmp);
        tmp.p.z(tmp.p.z()+0.1);
        way_points.push_back(tmp);
        tmp.p.x(tmp.p.x()-0.1);
        way_points.push_back(tmp);
        tmp.p.z(tmp.p.z()-0.1);
        way_points.push_back(tmp);
    }

    virtual ~testLinearTrj() {

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

    test_trajectory_generator trj;
    KDL::Frame start;
    KDL::Frame end;
    std::vector<KDL::Frame> way_points;

};

/**
 * @brief KDLFramesAreEqual perform GTEST check in 2 frames
 * @param a first frame
 * @param b second frame
 */
static inline void KDLFramesAreEqual(const KDL::Frame& a, const KDL::Frame& b,
                                     const double near = 1e-10)
{
    EXPECT_NEAR(a.p.x(), b.p.x(), near);
    EXPECT_NEAR(a.p.y(), b.p.y(), near);
    EXPECT_NEAR(a.p.z(), b.p.z(), near);

    double x,y,z,w; a.M.GetQuaternion(x,y,z,w);
    double xx,yy,zz,ww; b.M.GetQuaternion(xx,yy,zz,ww);

    EXPECT_NEAR(x,xx, near);
    EXPECT_NEAR(y,yy, near);
    EXPECT_NEAR(z,zz, near);
    EXPECT_NEAR(w,ww, near);
}

TEST_F(testLinearTrj, testNormalizeQuaternion)
{
    double x_start, y_start, z_start, w_start;
    double x_end, y_end, z_end, w_end;

    this->trj._normalizeQuaternion(this->start);
    this->trj._normalizeQuaternion(this->end);

    this->start.M.GetQuaternion(x_start, y_start, z_start, w_start);

    this->end.M.GetQuaternion(x_end, y_end, z_end, w_end);

    EXPECT_DOUBLE_EQ(sqrt(x_start*x_start + y_start*y_start + z_start*z_start
                          + w_start*w_start),1.);
    EXPECT_DOUBLE_EQ(sqrt(x_end*x_end + y_end*y_end + z_end*z_end
                          + w_end*w_end),1.);
}

TEST_F(testLinearTrj, testCheckIfCoastPhaseExists)
{
    double L = 1.;
    double max_vel = 1.;
    double max_acc = 0.1;

    EXPECT_FALSE(this->trj._checkIfCoastPhaseExists(max_vel, max_acc, L));

    max_acc = 10.0;
    EXPECT_TRUE(this->trj._checkIfCoastPhaseExists(max_vel, max_acc, L));
}

TEST_F(testLinearTrj, testcheckCreateLinePath)
{
    boost::shared_ptr<KDL::Path> path = this->trj._createLinePath(this->start, this->end);

    EXPECT_FALSE(path == NULL);

    EXPECT_TRUE(path->PathLength() > 1e-10) << path->PathLength();
    std::cout<<"PathLength: "<<path->PathLength()<<std::endl;
}

TEST_F(testLinearTrj, testCreateTrapezoidalVelProfile)
{
    double max_vel = 1.;
    double max_acc = 10.0;

    boost::shared_ptr<KDL::Path> path = this->trj._createLinePath(this->start, this->end);
    boost::shared_ptr<KDL::VelocityProfile> velocity_profile = this->trj._createTrapezoidalVelProfile(
                max_vel, max_acc, path->PathLength());

    EXPECT_FALSE(velocity_profile == NULL);

    EXPECT_TRUE(velocity_profile->Duration() > 0.0);
    std::cout<<"Duration: "<<velocity_profile->Duration()<<std::endl;
}

TEST_F(testLinearTrj, testTrajectoryGenerator){
    EXPECT_DOUBLE_EQ(this->trj.getTime(), 0.0);

    EXPECT_TRUE(this->trj.addLineTrj(this->way_points[0], this->way_points[1], 1., 20.));

    EXPECT_TRUE(this->trj.isInited());
    EXPECT_TRUE(this->trj.Duration() > 0.0);
    std::cout<<"Duration: "<<this->trj.Duration()<<std::endl;

    this->trj.resetTrajectory();
    EXPECT_DOUBLE_EQ(this->trj.Duration(), 0.0);
    EXPECT_FALSE(this->trj.isInited());

    double T = 1.0;
    EXPECT_TRUE(this->trj.addLineTrj(this->way_points[0], this->way_points[1], T));
    EXPECT_DOUBLE_EQ(this->trj.Duration(), T);

    KDL::Frame pose = this->trj.Pos(0.0);
    KDLFramesAreEqual(pose, this->way_points[0]);

    for(unsigned int i = 0; i < 1000-1; ++i){
        this->trj.updateTrj();
        EXPECT_FALSE(this->trj.isFinished());
        EXPECT_TRUE(this->trj.isStarted());
        EXPECT_TRUE(this->trj.isRunning());
    }

    this->trj.updateTrj();
    EXPECT_DOUBLE_EQ(this->trj.getTime(),1.);
    EXPECT_TRUE(this->trj.isFinished());

    pose = this->trj.Pos();
    KDLFramesAreEqual(pose, this->way_points[1]);

    pose = this->trj.Pos(1.0);
    KDLFramesAreEqual(pose, this->way_points[1]);
}

TEST_F(testLinearTrj, test2Lines){
    double T = 1.0;
    EXPECT_TRUE(this->trj.addLineTrj(this->way_points[0], this->way_points[1], T));
    EXPECT_DOUBLE_EQ(this->trj.Duration(), T);

    EXPECT_TRUE(this->trj.addLineTrj(this->way_points[1], this->way_points[2], T));
    EXPECT_DOUBLE_EQ(this->trj.Duration(), 2.*T);

    KDL::Frame pose = this->trj.Pos(0.0);
    KDLFramesAreEqual(pose, this->way_points[0]);

    pose = this->trj.Pos(1.0);
    KDLFramesAreEqual(pose, this->way_points[1]);

    pose = this->trj.Pos(2.0);
    KDLFramesAreEqual(pose, this->way_points[2]);

    this->trj.resetTrajectory();
    std::vector<KDL::Frame> way_points;
    way_points.push_back(this->way_points[0]);
    way_points.push_back(this->way_points[1]);
    way_points.push_back(this->way_points[2]);
    EXPECT_TRUE(this->trj.addLineTrj(way_points, T));

    pose = this->trj.Pos(0.0);
    KDLFramesAreEqual(pose, this->way_points[0]);

    pose = this->trj.Pos(1.0);
    KDLFramesAreEqual(pose, this->way_points[1]);

    pose = this->trj.Pos(2.0);
    KDLFramesAreEqual(pose, this->way_points[2]);

    this->trj.resetInternalTime();
    for(unsigned int i = 0; i < 2000; ++i)
    {
        this->trj.updateTrj();
        this->logFrame(this->trj.Pos());
    }
    writeMatlabFile(500);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
