#include <gtest/gtest.h>
#include <kiba_trajectory/kiba_trajectory.hpp>

class TestSpline : public kiba_trajectory::SingleDOFCubicSpline
{
  public:
    void test_check_times(double sample_time) const
    {
      check_times(sample_time);
    }

    double test_index(double sample_time) const
    {
      return index(sample_time);
    }

    double test_a_param() const
    {
      return a_param();
    }

    double test_b_param() const
    {
      return b_param();
    }
 
};

class SplineTests : public ::testing::Test
{
  protected:
    virtual void SetUp() { epsilon = 0.00000001; }
    virtual void TearDown() {}
    double epsilon;
};

void equals(const kiba_trajectory::SingleDOFTrajectoryPoint& a,
    const kiba_trajectory::SingleDOFTrajectoryPoint& b,
    double delta)
{
  EXPECT_NEAR(a.position, b.position, delta);
  EXPECT_NEAR(a.velocity, b.velocity, delta);
  EXPECT_NEAR(a.acceleration, b.acceleration, delta);
  EXPECT_NEAR(a.time, b.time, delta);
}

TEST_F(SplineTests, CheckTimes)
{
  TestSpline s;
  s.start_time = 1.1;
  s.end_time = 2.0;
  EXPECT_NO_THROW(s.test_check_times(1.1));
  EXPECT_NO_THROW(s.test_check_times(1.2));
  EXPECT_NO_THROW(s.test_check_times(2.0));
  EXPECT_ANY_THROW(s.test_check_times(1.0));
  EXPECT_ANY_THROW(s.test_check_times(2.1));
  s.end_time = 1.0;
  EXPECT_ANY_THROW(s.test_check_times(0.9));
  EXPECT_ANY_THROW(s.test_check_times(1.0));
  EXPECT_ANY_THROW(s.test_check_times(1.1));
  EXPECT_ANY_THROW(s.test_check_times(1.2));
}

TEST_F(SplineTests, Index)
{
  TestSpline s;
  s.start_time = 1.0;
  s.end_time = 2.0;
  EXPECT_NEAR(0, s.test_index(1), epsilon);
  EXPECT_NEAR(0.25, s.test_index(1.25), epsilon);
  EXPECT_NEAR(0.7, s.test_index(1.7), epsilon);
  EXPECT_NEAR(1, s.test_index(2), epsilon);
}

TEST_F(SplineTests, Aparam)
{
  TestSpline s;
  s.start_time = 1.0;
  s.end_time = 2.0;
  s.start_position = 0.0;
  s.end_position = 0.2;
  s.start_velocity = 0.0;
  s.end_velocity = 0.1;
  EXPECT_NEAR(-0.2, s.test_a_param(), epsilon);
  s.start_velocity = 0.15;
  EXPECT_NEAR(-0.05, s.test_a_param(), epsilon);
  s.start_velocity = -0.15;
  EXPECT_NEAR(-0.35, s.test_a_param(), epsilon);
}

TEST_F(SplineTests, Bparam)
{
  TestSpline s;
  s.start_time = 1.0;
  s.end_time = 2.0;
  s.start_position = 0.0;
  s.end_position = 0.2;
  s.start_velocity = 0.0;
  s.end_velocity = 0.1;
  EXPECT_NEAR(0.1, s.test_b_param(), epsilon);
  s.start_velocity = 0.15;
  EXPECT_NEAR(0.1, s.test_b_param(), epsilon);
  s.start_velocity = -0.15;
  EXPECT_NEAR(0.1, s.test_b_param(), epsilon);
  s.end_velocity = 0.5;
  EXPECT_NEAR(-0.3, s.test_b_param(), epsilon);
  s.end_velocity = -0.3;
  EXPECT_NEAR(0.5, s.test_b_param(), epsilon);
}

TEST_F(SplineTests, SampleSimple)
{
  TestSpline s;
  s.start_time = 1.0;
  s.end_time = 2.0;
  s.start_position = 0.0;
  s.end_position = 0.1;
  s.start_velocity = 0.1;
  s.end_velocity = 0.1;

  using namespace kiba_trajectory;
  SingleDOFTrajectoryPoint a, b, c;
  a.position = 0;
  a.velocity = 0.1;
  a.acceleration = 0.0;
  a.time = 1.0;
  b.position = 0.025;
  b.velocity = 0.1;
  b.acceleration = 0.0;
  b.time = 1.25;
  c.position = 0.1;
  c.velocity = 0.1;
  c.acceleration = 0.0;
  c.time = 2.0;
  equals(a, s.sample(1.0), epsilon);
  equals(b, s.sample(1.25), epsilon);
  equals(c, s.sample(2.0), epsilon);
}
