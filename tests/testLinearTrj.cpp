#include <idynutils/tests_utils.h>
#include <gtest/gtest.h>
#include <trajectory_utils/trajectory_utils.h>
#include <idynutils/cartesian_utils.h>

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

    test_trajectory_generator trj;
    KDL::Frame start;
    KDL::Frame end;
    std::vector<KDL::Frame> way_points;

};

TEST_F(testLinearTrj, testNormalizeQuaternion)
{
    double x, y, z, w;

    this->start.M.GetQuaternion(x, y, z, w);
    quaternion q_start(x,y,z,w);

    this->end.M.GetQuaternion(x, y, z, w);
    quaternion q_end(x,y,z,w);

    q_start.normalize(q_start);
    q_end.normalize(q_end);

    EXPECT_DOUBLE_EQ(sqrt(q_start.x*q_start.x + q_start.y*q_start.y + q_start.z*q_start.z
                          + q_start.w*q_start.w),1.);
    EXPECT_DOUBLE_EQ(sqrt(q_end.x*q_end.x + q_end.y*q_end.y + q_end.z*q_end.z
                          + q_end.w*q_end.w),1.);

    this->trj._normalizeQuaternion(this->start);
    this->trj._normalizeQuaternion(this->end);

    this->start.M.GetQuaternion(x, y, z, w);
    quaternion _q_start(x,y,z,w);

    this->end.M.GetQuaternion(x, y, z, w);
    quaternion _q_end(x,y,z,w);

    EXPECT_DOUBLE_EQ(sqrt(_q_start.x*_q_start.x + _q_start.y*_q_start.y + _q_start.z*_q_start.z
                          + _q_start.w*_q_start.w),1.);
    EXPECT_DOUBLE_EQ(sqrt(_q_end.x*_q_end.x + _q_end.y*_q_end.y + _q_end.z*_q_end.z
                          + _q_end.w*_q_end.w),1.);
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

    EXPECT_TRUE(this->trj.addLineTrj(trajectory_utils::BANG_COAST_BANG,
                                     this->way_points[0], this->way_points[1], 1., 20.));

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
    tests_utils::KDLFramesAreEqual(pose, this->way_points[0]);

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
    tests_utils::KDLFramesAreEqual(pose, this->way_points[1]);

    pose = this->trj.Pos(1.0);
    tests_utils::KDLFramesAreEqual(pose, this->way_points[1]);
}

TEST_F(testLinearTrj, test2Lines){
    double T = 1.0;
    EXPECT_TRUE(this->trj.addLineTrj(this->way_points[0], this->way_points[1], T));
    EXPECT_DOUBLE_EQ(this->trj.Duration(), T);

    EXPECT_TRUE(this->trj.addLineTrj(this->way_points[1], this->way_points[2], T));
    EXPECT_DOUBLE_EQ(this->trj.Duration(), 2.*T);

    KDL::Frame pose = this->trj.Pos(0.0);
    tests_utils::KDLFramesAreEqual(pose, this->way_points[0]);

    pose = this->trj.Pos(1.0);
    tests_utils::KDLFramesAreEqual(pose, this->way_points[1]);

    pose = this->trj.Pos(2.0);
    tests_utils::KDLFramesAreEqual(pose, this->way_points[2]);

    this->trj.resetTrajectory();
    std::vector<KDL::Frame> way_points;
    way_points.push_back(this->way_points[0]);
    way_points.push_back(this->way_points[1]);
    way_points.push_back(this->way_points[2]);
    EXPECT_TRUE(this->trj.addLineTrj(way_points, T));

    pose = this->trj.Pos(0.0);
    tests_utils::KDLFramesAreEqual(pose, this->way_points[0]);

    pose = this->trj.Pos(1.0);
    tests_utils::KDLFramesAreEqual(pose, this->way_points[1]);

    pose = this->trj.Pos(2.0);
    tests_utils::KDLFramesAreEqual(pose, this->way_points[2]);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
