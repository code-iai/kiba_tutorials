#include <gtest/gtest.h>
#include <kiba_trajectory/kiba_trajectory.hpp>

class ConversionTests : public ::testing::Test
{
  protected:
    virtual void SetUp() {}
    virtual void TearDown() {}
};

void equals(const kiba_trajectory::SingleDOFTrajectoryPoint& a, 
    const kiba_trajectory::SingleDOFTrajectoryPoint& b)
{
  EXPECT_DOUBLE_EQ(a.position, b.position);
  EXPECT_DOUBLE_EQ(a.velocity, b.velocity);
  EXPECT_DOUBLE_EQ(a.acceleration, b.acceleration);
  EXPECT_DOUBLE_EQ(a.time, b.time);
}

void equals(const kiba_trajectory::SingleDOFCubicSpline& a, 
    const kiba_trajectory::SingleDOFCubicSpline& b)
{
  EXPECT_DOUBLE_EQ(a.start_time, b.start_time);
  EXPECT_DOUBLE_EQ(a.end_time, b.end_time);
  EXPECT_DOUBLE_EQ(a.start_position, b.start_position);
  EXPECT_DOUBLE_EQ(a.end_position, b.end_position);
  EXPECT_DOUBLE_EQ(a.start_velocity, b.start_velocity);
  EXPECT_DOUBLE_EQ(a.end_velocity, b.end_velocity);
}

TEST_F(ConversionTests, JointTrajectoryPoint)
{
  using namespace kiba_trajectory;
  SingleDOFTrajectoryPoint p;
  p.position = 1.23;
  p.velocity = -0.1;
  p.acceleration = 9.81;
  p.time = 42.31;
  equals(p, from_msg(to_msg(p)));

  trajectory_msgs::JointTrajectoryPoint msg;
  EXPECT_THROW(from_msg(msg), std::runtime_error);
  msg.positions.push_back(0);
  EXPECT_THROW(from_msg(msg), std::runtime_error);
  msg.velocities.push_back(0);
  EXPECT_THROW(from_msg(msg), std::runtime_error);
  msg.accelerations.push_back(0);
  EXPECT_NO_THROW(from_msg(msg));
}

TEST_F(ConversionTests, CubicSpline)
{
  using namespace kiba_trajectory;
  SingleDOFCubicSpline s;
  s.start_time = 1.1;
  s.end_time = 15.3;
  s.start_position = 17.1;
  s.end_position = -3.22;
  s.start_velocity = 0.001;
  s.end_velocity = 123,45;
  equals(s, from_msg(to_msg(s)));
}
