#include <gtest/gtest.h>
#include <trajectory_utils/trajectory_utils.h>

namespace {
class testTrjDesigOpenSoT: public ::testing::Test {
public:
    testTrjDesigOpenSoT()
    {

    }

    virtual ~testTrjDesigOpenSoT() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
