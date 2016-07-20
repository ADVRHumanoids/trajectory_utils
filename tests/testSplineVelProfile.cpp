#include <idynutils/tests_utils.h>
#include <gtest/gtest.h>
#include <trajectory_utils/trajectory_utils.h>
#include <idynutils/cartesian_utils.h>
#include <fstream>

#define dt 0.001

namespace {

class test_trajectory_generator: public trajectory_utils::trajectory_generator
{
public:
    test_trajectory_generator():
    trajectory_generator(dt)
    {}

    boost::shared_ptr<KDL::VelocityProfile> _createSplineVelProfile(const double L, const double T){
        return createSplineVelProfile(L, T);
    }
    boost::shared_ptr<KDL::VelocityProfile> _createSplineVelProfile(const double L, const double T,
                                                                    const double v0, const double v1){
        return createSplineVelProfile(L, T, v0, v1);
    }
    boost::shared_ptr<KDL::VelocityProfile> _createSplineVelProfile(const double L, const double T,
                                                                    const double v0, const double v1,
                                                                    const double a0, const double a1){
        return createSplineVelProfile(L, T, v0, v1,a0, a1);
    }
};

    class testSplineVelProfile: public ::testing::Test {
    public:
        testSplineVelProfile():
        trj(),
        L(1.0),
        v_max(M_PI),
        a_max(10.*M_PI)
        {

        }

        virtual ~testSplineVelProfile() {

        }

        virtual void SetUp() {

        }

        virtual void TearDown() {

        }

        test_trajectory_generator trj;
        double L;
        double v_max;
        double a_max;


};

    TEST_F(testSplineVelProfile, testSplineVelProfile_)
    {
        std::ofstream file1;
        file1.open("vel_prof1.m");
        file1<<"velocity_profile1 = {"<<std::endl;
        std::ofstream file2;
        file2.open("vel_prof2.m");
        file2<<"velocity_profile2 = {"<<std::endl;


        double T = this->L/this->v_max;
        std::cout<<"T: "<<T<<std::endl;
        boost::shared_ptr<KDL::VelocityProfile> vel_prof1 = this->trj._createSplineVelProfile(L,T);
        for(double t = 0; t <= T; t +=dt){
            EXPECT_LE(vel_prof1->Vel(t), this->v_max);
            //std::cout<<vel_prof1->Vel(t)<<" <= "<<this->v_max<<std::endl;
            //std::cout<<vel_prof1->Acc(t)<<std::endl;

            file1<<"["<<vel_prof1->Pos(t)<<", "<<vel_prof1->Vel(t)<<", "<<vel_prof1->Acc(t)<<"]"<<std::endl;
        }
        file1<<"};"<<std::endl;
        EXPECT_DOUBLE_EQ(vel_prof1->Pos(T),L);

        T = (3./2.)*(this->L/this->v_max);
        std::cout<<"T: "<<T<<std::endl;
        boost::shared_ptr<KDL::VelocityProfile> vel_prof2 = this->trj._createSplineVelProfile(L,T,0.,0.);
        for(double t = 0; t <= T; t +=dt){
            EXPECT_LE(vel_prof2->Vel(t), this->v_max);
            //std::cout<<vel_prof1->Vel(t)<<" <= "<<this->v_max<<std::endl;
            //std::cout<<vel_prof1->Acc(t)<<std::endl;
            file2<<"["<<vel_prof2->Pos(t)<<", "<<vel_prof2->Vel(t)<<", "<<vel_prof2->Acc(t)<<"]"<<std::endl;
        }
        file2<<"};"<<std::endl;
        EXPECT_DOUBLE_EQ(vel_prof2->Pos(T),L);
        EXPECT_DOUBLE_EQ(vel_prof2->Vel(T/2.),this->v_max);
    }


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
