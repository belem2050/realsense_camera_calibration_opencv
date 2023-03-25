#include <gtest/gtest.h>
#include <vector>

class MyFixture : public ::testing::Test {
protected:
    void SetUp() override {
        // set up fixture object
    }
    void TearDown() override {
        // tear down fixture object
    }
};

// Define a parameterized test fixture class
class MyParameterizedTest : public MyFixture, public ::testing::WithParamInterface<int> {
public:
    static std::vector<int> values() {
        return {1, 2, 3};
    }
};

// Instantiate the tests with parameter values
INSTANTIATE_TEST_SUITE_P(MyParameters, MyParameterizedTest, ::testing::ValuesIn(MyParameterizedTest::values()));

// Define the parameterized test case
TEST_P(MyParameterizedTest, MyTest) {
    int value = GetParam();
    ASSERT_GT(value, 0);
}

// Define the fixture-based test case
TEST_F(MyFixture, MyFixtureTest) {
    ASSERT_TRUE(true);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  // Initialize ROS2
  //rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();

  // Shutdown ROS2
  //rclcpp::shutdown();

  return ret;

}
