#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <kiba_control/kiba_hw_sim.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_hw_main");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  boost::shared_ptr<kiba_control::KibaHwSim> hw_sim(new kiba_control::KibaHwSim(nh));
  hw_sim->init();
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, hw_sim);
      
  ros::waitForShutdown();
      
  return 0;
}
