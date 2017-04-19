#ifndef KIBA_TUTORIALS_DATATYPES_HPP
#define KIBA_TUTORIALS_DATATYPES_HPP

namespace kiba_trajectory
{
  class SingleDOFTrajectoryPoint
  {
    public:
      // member variables
      double position, velocity, acceleration, time;
  };

  class SingleDOFCubicSpline
  {
    public:
      // member variables
      double start_time, end_time, start_position, end_position, 
             start_velocity, end_velocity;
  };
}

#endif
