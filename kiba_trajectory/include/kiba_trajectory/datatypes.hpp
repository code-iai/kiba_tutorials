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
      
      // member functions
      SingleDOFTrajectoryPoint sample(double sample_time) const
      {
        SingleDOFTrajectoryPoint result;
        result.position = position(sample_time);
        result.velocity = velocity(sample_time);
        result.acceleration = acceleration(sample_time);
        result.time = sample_time;
        return result;
      }

    protected:
      // member functions
      double index(double sample_time) const
      {
        check_times(sample_time);
        return (sample_time - start_time) / (end_time - start_time);
      }

      double a_param() const
      {
        check_times(start_time); // small hack
        return start_velocity * (end_time - start_time) -
          (end_position - start_position);
      }

      double b_param() const
      {
        check_times(start_time); // small hack
        return -end_velocity * (end_time - start_time) +
          (end_position - start_position);
      }

      double position(double sample_time) const
      {
        double i = index(sample_time);
        double a = a_param();
        double b = b_param();
        return (1 - i)*start_position + i * end_position +
          i * (1 - i) * (a * (1 - i) + b * i);
      }

      double velocity(double sample_time) const
      {
        double i = index(sample_time);
        double a = a_param();
        double b = b_param();
        double dt = end_time - start_time;
        return (end_position - start_position) / dt +
          (1 - 2 * i) * (a * (1 - i) + b * i) / dt +
          i * (1 - i) * (b - a) / dt;
      }

      double acceleration(double sample_time) const
      {
        double i = index(sample_time);
        double a = a_param();
        double b = b_param();
        double dt2 = (end_time - start_time) * (end_time - start_time);
        return 2 * (b - 2 * a + (a - b) * 3 *i) / dt2;
      }

      void check_times(double sample_time) const
      {
        if (start_time > end_time)
          throw std::runtime_error("spline start_time after end_time: " +
              std::to_string(start_time) + " > " + std::to_string(end_time));

        if (start_time > sample_time)
          throw std::runtime_error("spline start_time after sample_time: " +
              std::to_string(start_time) + " > " + std::to_string(end_time));

        if (sample_time > end_time)
          throw std::runtime_error("spline sample_time after end_time: " +
              std::to_string(sample_time) + " > " + std::to_string(end_time));
      }
  };
}

#endif
