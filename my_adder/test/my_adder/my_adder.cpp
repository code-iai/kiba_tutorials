#include <gtest/gtest.h>
#include <my_adder/my_adder.hpp>

class TestMyAdder : public MyAdder::MyAdder
{
  public:
    const std::deque<double> buffer() const
    {
      return buffer_;
    }

};

class MyAdderTests : public ::testing::Test
{
  protected:
    virtual void SetUp() {}
    virtual void TearDown() {}
};

TEST_F(MyAdderTests, Failure1)
{
  TestMyAdder my_adder;
  EXPECT_EQ(0, my_adder.size());
  my_adder.new_summand(1.0);
  EXPECT_EQ(1, my_adder.size());
  my_adder.new_summand(2.0);
  ASSERT_EQ(2, my_adder.size());
  EXPECT_DOUBLE_EQ(my_adder.sum(), 3.0);
  EXPECT_DOUBLE_EQ(my_adder.buffer()[0], 1);
  EXPECT_DOUBLE_EQ(my_adder.buffer()[1], 2);
}
