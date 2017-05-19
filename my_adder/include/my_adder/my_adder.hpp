#ifndef MY_ADDER_MY_ADDER_HPP
#define MY_ADDER_MY_ADDER_HPP

#include <deque>

class MyAdder
{
  public:
    void new_summand(double summand)
    {
      if (size() == 2)
        buffer_.pop_front();
      buffer_.push_back(summand);
    }

    size_t size() const
    {
      return buffer_.size();
    }

    double sum() const
    {
      double result = 0;
      for (size_t i=0; i<size(); ++i)
        result += buffer_[i];

      return result;
    }


  protected:
    std::deque<double> buffer_;
};

#endif
